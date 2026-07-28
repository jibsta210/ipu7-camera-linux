# Building the IPU7 pipeline handler

This replaces libcamera's stock `simple` pipeline handler for IPU7 cameras and
drives the **hardware ISP** through `/dev/ipu7-psys0` instead of libcamera's
software ISP.

## Why you want it

Stock libcamera falls back to the `simple` pipeline plus `SoftwareIsp`. That
works at the sensor's native resolution and **crashes at every other size**:

```
cam --stream width=640,height=480    -> Assertion '__n < this->size()' failed
cam --stream width=1280,height=720   -> Assertion '__n < this->size()' failed
cam --stream width=1920,height=1080  -> Assertion '__n < this->size()' failed
cam  (native 3856x2176)              -> 10 frames, fine
```

Ask for anything below roughly half the sensor width and libcamera selects the
binned `1928x1088` sensor mode, whose statistics pass indexes past the end of a
64-bin array and calls `abort()`. Inside WirePlumber that takes the whole
process down, and the application reports *"camera in use by another
application"*.

Browsers ask for 640x480 or 1280x720. So on stock libcamera the CLI looks
perfect and every real application fails.

With this handler, the same sizes work at full frame rate:

| resolution | stock `simple` | this handler |
|---|---|---|
| 1280x720  | crash | 28.57 fps |
| 1920x1080 | crash | 28.57 fps |
| native    | works | works |

It also costs far less CPU — the ISP does the debayer, not the host.

## Prerequisites

`/dev/ipu7-psys0` must exist. If it does not, fix that first — see
[svp7500-camera-fix-pack](https://github.com/jibsta210/svp7500-camera-fix-pack).
This handler checks for the node and declines to match without it.

## Build

```bash
git clone https://git.libcamera.org/libcamera/libcamera.git
cd libcamera
git checkout v0.7.2          # match your distro's libcamera ABI

mkdir -p src/libcamera/pipeline/ipu7
cp -r /path/to/this/repo/libcamera-pipeline/* src/libcamera/pipeline/ipu7/

# register the handler
sed -i "s/^\( *\)'simple',/\1'ipu7',\n\1'simple',/" meson_options.txt

meson setup build -Dpipelines=ipu7,simple,uvcvideo -Dipas= -Dtest=false
ninja -C build
```

Check your distro's libcamera version with `pacman -Q libcamera`,
`dnf list installed libcamera` or `apt list --installed libcamera*` and check
out that tag — the handler uses libcamera internals, so a mismatched tree will
not compile.

## Install without replacing your distro's libcamera

Do not overwrite the packaged libcamera; a package update would silently undo
it. Install alongside, and point only PipeWire at it:

```bash
sudo mkdir -p /opt/libcamera-ipu7/lib
sudo cp build/src/libcamera/libcamera.so.0.7.0 \
        build/src/libcamera/base/libcamera-base.so.0.7.0 \
        /opt/libcamera-ipu7/lib/
cd /opt/libcamera-ipu7/lib
sudo ln -sf libcamera.so.0.7.0      libcamera.so.0.7
sudo ln -sf libcamera-base.so.0.7.0 libcamera-base.so.0.7

for s in pipewire wireplumber; do
  mkdir -p ~/.config/systemd/user/$s.service.d
  cat > ~/.config/systemd/user/$s.service.d/libcamera.conf <<'CONF'
[Service]
Environment="LD_LIBRARY_PATH=/opt/libcamera-ipu7/lib"
CONF
done

systemctl --user daemon-reload
systemctl --user restart pipewire wireplumber
```

**Install to a system path, not a build tree.** Pointing the drop-in at
`~/src/libcamera/build` works until something cleans that directory, at which
point every camera app silently falls back to the software ISP and browsers
start crashing with nothing to explain why.

## Verify

```bash
grep -c /opt/libcamera-ipu7 /proc/$(pgrep -x wireplumber)/maps   # expect > 0
journalctl --user -u wireplumber -b | grep 'IPU7 ipu7.cpp'       # expect match lines
```

`cam --list` will still say `simple` — that is the CLI using your *system*
libcamera, which is untouched by design. What matters is what WirePlumber
loads.

## Notes

- The NPU denoiser was removed: the DnCNN model turned out to be an identity
  function and the ISP's own noise reduction is sufficient. That also drops a
  build dependency on OpenVINO at `/opt/openvino`, which made this handler
  unbuildable anywhere but the author's machine.
- The Dell mounting is inverted, so the handler reports
  `properties::Rotation = 180`. PipeWire and the compositor apply it on the
  GPU at no CPU cost. The kernel driver reports rotation 0, which is wrong.
- ISP parameters were captured from the Intel Camera HAL by intercepting its
  ioctls, and are compiled in from `params/*.h`. They are tuned for OV08X40;
  another sensor will need its own.
