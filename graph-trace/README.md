# graph-trace — capture the psys graph the Intel Camera HAL submits

The `ipu7` libcamera pipeline handler submits a **hardcoded** psys graph. It was
traced off the Intel Camera HAL on **Panther Lake** and there is no
platform-conditional code anywhere in it. On **Lunar Lake** the firmware runs
different program groups and rejects it:

```
intel-ipu7-psys ipu7-psys0: group 3, code 38, detail: 3, 0
intel-ipu7-psys ipu7-psys0: ipu7_psys_handle_graph_open_ack, num_items is 0
intel-ipu7-psys ipu7-psys0: Failed to set graph
ERROR IPU7 psys.cpp:162 GRAPH_OPEN failed: Invalid argument
```

Check which you have — no reboot needed:

```bash
sudo dmesg | grep -oE 'INTC10E1|INTC10DE' | head -1   # E1 = PTL, DE = LNL
glxinfo -B | grep -oE '\((PTL|LNL)\)'                 # or ask Mesa
```

Making the handler work on LNL needs that platform's graph. **You do not need
Windows for this.** The Intel Camera HAL already supports LNL and builds its
graph from the per-platform GCSS descriptor it ships:

```
/etc/camera/ipu75xa/gcss/*.IPU75XA.bin    Panther Lake
/etc/camera/ipu7x/gcss/*.IPU7X.bin        Lunar Lake
```

Rather than reverse-engineering that format, watch what the HAL sends the
kernel.

## What it does

`LD_PRELOAD` shim over `ioctl(2)`. When something issues `IPU_IOC_GRAPH_OPEN`
on `/dev/ipu7-psys0` it prints the whole `struct ipu_psys_graph_info` — every
node, its `teb`/`deb`/`rbm`/`reb` bitmaps, every terminal with its buffer size,
and every link — then passes the call through untouched. It writes nothing to
the device and changes nothing about what the HAL does.

Node profiles print as C initialisers so they can be pasted straight into a
`PsysNodeProfile`.

## Build

```bash
gcc -shared -fPIC -O2 -o graph-trace.so graph-trace.c -ldl
```

## Capture

The camera must be free — psys does not share, and PipeWire holds it:

```bash
systemctl --user stop wireplumber pipewire pipewire.socket
```

The HAL writes its tuning cache to `/run/camera`, which does not exist and
which a normal user cannot create -- `/run` is root-owned. Without it the HAL
errors out before it ever opens a graph:

```bash
sudo mkdir -p /run/camera && sudo chown "$(id -u):$(id -g)" /run/camera
```

Then run a capture through the HAL:

```bash
LD_PRELOAD=$PWD/graph-trace.so GRAPH_TRACE_OUT=/tmp/graph-hal.txt \
  gst-launch-1.0 icamerasrc num-buffers=5 ! fakesink
```

Restart PipeWire afterwards:

```bash
systemctl --user start pipewire.socket wireplumber
```

If `icamerasrc` produces frames, `/tmp/graph-hal.txt` holds the graph your
firmware accepts. That file is the whole deliverable.

## Sanity check

Point it at the libcamera handler instead, and you should see the PTL graph it
submits — useful to confirm the shim works before trusting a HAL run:

```bash
LD_PRELOAD=$PWD/graph-trace.so GRAPH_TRACE_OUT=/tmp/graph-libcamera.txt \
  LD_LIBRARY_PATH=/opt/libcamera-ipu7/lib cam --camera=1 --capture=1 --file=/tmp/x.raw
```

On a PTL machine that prints `teb = { 0x000c3d27, 0x00000000 }` for node 0,
matching `isaProfile` in `ipu7.cpp`.

## Reading the output when nothing is captured

The shim announces itself from a constructor and reports at exit, so an empty
or unhelpful file still tells you something:

```
graph-trace: loaded into pid N          <- shim is in the process
graph-trace: ioctl fd=21 ... [psys]     <- device is open, ioctls flowing
graph-trace: GRAPH_OPEN via ioctl()     <- caught it
```

| what you see | what it means |
|---|---|
| no `loaded into pid` at all | `LD_PRELOAD` did not take -- wrong path, or the pipeline runs the camera in a different process |
| `loaded` but no `[psys]` lines | that process never touched `/dev/ipu7-psys*` |
| `[psys]` lines but no `GRAPH_OPEN` | the device is open and the graph went out under a different ioctl number -- the logged `req=` values are then the interesting part |
| `GRAPH_OPEN via syscall()` | the caller bypassed the libc wrapper; caught anyway |

`GRAPH_TRACE_VERBOSE=1` logs every ioctl on every fd, not just psys ones.

## Caveats

- **Use a pipeline that actually streams on your machine.** The shim only
  fires on `IPU_IOC_GRAPH_OPEN`, so if the HAL errors out before it submits a
  graph you get an empty output file and it looks like the shim failed. A HAL
  run that ends in
  `SensorHwCtrl: sensor output sub device is not set` followed by `Got EOS`
  after about a second never got that far. If some other pipeline gives you
  frames -- `icamerasrc ! ... ! v4l2sink`, for instance -- trace *that* one.
- **`icamerasrc` may segfault**, and that is not this shim's fault — verify by
  running the same command without `LD_PRELOAD`. The HAL only works on boards
  it has an `.aiqb` and GCSS descriptor for. It segfaults on the author's PTL
  machine with or without the shim.
- A graph captured this way describes *your* silicon and sensor. It is not
  portable to a different generation, which is the entire reason this exists.
