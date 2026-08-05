/*
 * graph-trace.c -- dump the psys graph the Intel Camera HAL submits.
 *
 * LD_PRELOAD shim. Watches for IPU_IOC_GRAPH_OPEN on /dev/ipu7-psys* and prints
 * the whole struct ipu_psys_graph_info -- nodes, profile bitmaps, terminals,
 * links -- then passes the call through untouched.
 *
 * WHY
 *   The libcamera IPU7 pipeline handler submits a hardcoded graph captured on
 *   Panther Lake. Lunar Lake firmware rejects it (graph_open_ack returns
 *   num_items 0). The Intel Camera HAL *does* work on LNL, so watching what it
 *   sends is the shortest path to an LNL graph.
 *
 * DIAGNOSTICS
 *   A silent, empty output file has several possible causes, and they need
 *   different fixes. So this build is noisy on purpose:
 *
 *     - it announces itself from a constructor, so "did the shim even load?"
 *       is answered before anything else;
 *     - it records every open() of a psys device;
 *     - it logs EVERY ioctl on a psys fd, not just GRAPH_OPEN, with the
 *       request number -- so a HAL using different ioctl numbers is visible
 *       rather than silent;
 *     - it also hooks syscall(), because a caller that invokes SYS_ioctl
 *       directly bypasses the libc ioctl() wrapper entirely and would
 *       otherwise produce exactly the "nothing happened" symptom.
 *
 *   Set GRAPH_TRACE_VERBOSE=1 to log every ioctl on any fd.
 *
 * BUILD
 *   gcc -shared -fPIC -O2 -o graph-trace.so graph-trace.c -ldl
 *
 * USE
 *   LD_PRELOAD=$PWD/graph-trace.so GRAPH_TRACE_OUT=/tmp/graph.txt <pipeline>
 */

#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <unistd.h>

#define MAX_GRAPH_NODES          5U
#define MAX_GRAPH_LINKS          10U
#define MAX_GRAPH_NODE_PROFILES  1U
#define MAX_GRAPH_TERMINALS      64U

struct node_profile {
	uint32_t teb[2];
	uint32_t deb[4];
	uint32_t rbm[4];
	uint32_t reb[4];
} __attribute__((packed));

struct node_ternimal {
	uint8_t  term_id;
	uint32_t buf_size;
} __attribute__((packed));

struct graph_node {
	uint8_t node_rsrc_id;
	uint8_t node_ctx_id;
	uint8_t num_terms;
	struct node_profile  profiles[MAX_GRAPH_NODE_PROFILES];
	struct node_ternimal terminals[MAX_GRAPH_TERMINALS];
} __attribute__((packed));

struct graph_link_ep {
	uint8_t node_ctx_id;
	uint8_t term_id;
} __attribute__((packed));

struct graph_link {
	struct graph_link_ep ep_src;
	struct graph_link_ep ep_dst;
	uint16_t foreign_key;
	uint8_t  streaming_mode;
	uint8_t  pbk_id;
	uint8_t  pbk_slot_id;
	uint8_t  delayed_link;
} __attribute__((packed));

struct ipu_psys_graph_info {
	uint8_t graph_id;
	uint8_t num_nodes;
	struct graph_node *nodes;
	struct graph_link links[MAX_GRAPH_LINKS];
} __attribute__((packed));

#define IPU_IOC_GRAPH_OPEN _IOWR('A', 7, struct ipu_psys_graph_info)

static FILE *out;
static int   seen, verbose;

/* Bitmap of fds known to be psys devices. Small and fixed: an fd above this is
 * vanishingly unlikely for a camera process, and a missed one only costs a log
 * line, not correctness. */
#define MAXFD 4096
static unsigned char is_psys[MAXFD];

static FILE *sink(void)
{
	if (!out) {
		const char *p = getenv("GRAPH_TRACE_OUT");
		out = p ? fopen(p, "w") : NULL;
		if (!out)
			out = stderr;
		setvbuf(out, NULL, _IOLBF, 0);
	}
	return out;
}

__attribute__((constructor))
static void announce(void)
{
	verbose = getenv("GRAPH_TRACE_VERBOSE") != NULL;
	fprintf(sink(), "graph-trace: loaded into pid %d%s\n",
		(int)getpid(), verbose ? " (verbose)" : "");
	fprintf(sink(), "graph-trace: watching for ioctl 0x%lx (IPU_IOC_GRAPH_OPEN)\n",
		(unsigned long)IPU_IOC_GRAPH_OPEN);
}

__attribute__((destructor))
static void farewell(void)
{
	if (!out)
		return;
	if (!seen)
		fprintf(out, "graph-trace: pid %d exiting -- NO GRAPH_OPEN was seen.\n"
			     "  If [psys] ioctl lines appeared, the device was open but the\n"
			     "  graph went out under a different ioctl number.\n"
			     "  If none did, this process never touched /dev/ipu7-psys*.\n",
			(int)getpid());
	else
		fprintf(out, "graph-trace: pid %d exiting, %d graph(s) captured\n",
			(int)getpid(), seen);
}

static void note_open(int fd, const char *path)
{
	if (fd < 0 || fd >= MAXFD || !path)
		return;
	if (strstr(path, "ipu7-psys")) {
		is_psys[fd] = 1;
		fprintf(sink(), "graph-trace: psys opened: fd=%d %s\n", fd, path);
	}
}

int open(const char *path, int flags, ...)
{
	static int (*real)(const char *, int, ...);
	mode_t m = 0;
	va_list ap;
	if (!real) real = dlsym(RTLD_NEXT, "open");
	if (flags & O_CREAT) { va_start(ap, flags); m = va_arg(ap, mode_t); va_end(ap); }
	int fd = real(path, flags, m);
	note_open(fd, path);
	return fd;
}

int open64(const char *path, int flags, ...)
{
	static int (*real)(const char *, int, ...);
	mode_t m = 0;
	va_list ap;
	if (!real) real = dlsym(RTLD_NEXT, "open64");
	if (!real) return open(path, flags);
	if (flags & O_CREAT) { va_start(ap, flags); m = va_arg(ap, mode_t); va_end(ap); }
	int fd = real(path, flags, m);
	note_open(fd, path);
	return fd;
}

int openat(int dirfd, const char *path, int flags, ...)
{
	static int (*real)(int, const char *, int, ...);
	mode_t m = 0;
	va_list ap;
	if (!real) real = dlsym(RTLD_NEXT, "openat");
	if (flags & O_CREAT) { va_start(ap, flags); m = va_arg(ap, mode_t); va_end(ap); }
	int fd = real(dirfd, path, flags, m);
	note_open(fd, path);
	return fd;
}

static void dump(const struct ipu_psys_graph_info *g)
{
	FILE *f = sink();

	fprintf(f, "\n=== IPU_IOC_GRAPH_OPEN #%d ===\n", ++seen);
	fprintf(f, "graph_id=0x%02x num_nodes=%u\n", g->graph_id, g->num_nodes);

	if (!g->nodes) { fprintf(f, "  (nodes pointer is NULL)\n"); return; }

	unsigned n = g->num_nodes > MAX_GRAPH_NODES ? MAX_GRAPH_NODES : g->num_nodes;
	for (unsigned i = 0; i < n; i++) {
		const struct graph_node *nd = &g->nodes[i];
		unsigned t = nd->num_terms > MAX_GRAPH_TERMINALS
			   ? MAX_GRAPH_TERMINALS : nd->num_terms;
		const struct node_profile *p = &nd->profiles[0];

		fprintf(f, "\nNode[%u]: rsrc=%u ctx=%u terms=%u\n",
			i, nd->node_rsrc_id, nd->node_ctx_id, nd->num_terms);
		fprintf(f, "  { 0x%08x, 0x%08x },\t\t\t/* teb */\n", p->teb[0], p->teb[1]);
		fprintf(f, "  { 0x%08x, 0x%08x, 0x%08x, 0x%08x },\t/* deb */\n",
			p->deb[0], p->deb[1], p->deb[2], p->deb[3]);
		fprintf(f, "  { 0x%08x, 0x%08x, 0x%08x, 0x%08x },\t/* rbm */\n",
			p->rbm[0], p->rbm[1], p->rbm[2], p->rbm[3]);
		fprintf(f, "  { 0x%08x, 0x%08x, 0x%08x, 0x%08x },\t/* reb */\n",
			p->reb[0], p->reb[1], p->reb[2], p->reb[3]);
		fprintf(f, "  terminals:\n");
		for (unsigned k = 0; k < t; k++)
			fprintf(f, "    { %2u, %10u },\n",
				nd->terminals[k].term_id, nd->terminals[k].buf_size);
	}

	fprintf(f, "\nlinks:\n");
	for (unsigned i = 0; i < MAX_GRAPH_LINKS; i++) {
		const struct graph_link *l = &g->links[i];
		if (!l->ep_src.node_ctx_id && !l->ep_src.term_id &&
		    !l->ep_dst.node_ctx_id && !l->ep_dst.term_id &&
		    !l->foreign_key && !l->streaming_mode)
			continue;
		fprintf(f, "  { {%u,%2u}, {%u,%2u}, fk=0x%04x smode=%u pbk=%u slot=%u delayed=%u },\n",
			l->ep_src.node_ctx_id, l->ep_src.term_id,
			l->ep_dst.node_ctx_id, l->ep_dst.term_id,
			l->foreign_key, l->streaming_mode,
			l->pbk_id, l->pbk_slot_id, l->delayed_link);
	}
	fprintf(f, "=== end #%d ===\n", seen);
}

/* Shared by ioctl() and syscall(SYS_ioctl, ...). */
/* Resolve an fd to its path. Hooking open() proved unreliable -- a self-test
 * caught GRAPH_OPEN on an fd whose open() never came through any of the
 * wrappers below, so is_psys[] was empty on a run that plainly had the device
 * open. /proc/self/fd is what the kernel actually thinks, so ask that. */
static int fd_is_psys(int fd)
{
	char link[64], target[256];
	ssize_t n;

	if (fd < 0)
		return 0;
	if (fd < MAXFD && is_psys[fd])
		return 1;
	snprintf(link, sizeof(link), "/proc/self/fd/%d", fd);
	n = readlink(link, target, sizeof(target) - 1);
	if (n <= 0)
		return 0;
	target[n] = 0;
	if (strstr(target, "ipu7-psys")) {
		if (fd < MAXFD)
			is_psys[fd] = 1;
		return 1;
	}
	return 0;
}

static void watch(int fd, unsigned long req, void *arg, const char *via)
{
	int psys = fd_is_psys(fd);

	if (req == (unsigned long)IPU_IOC_GRAPH_OPEN && arg) {
		fprintf(sink(), "graph-trace: GRAPH_OPEN via %s on fd=%d\n", via, fd);
		dump((const struct ipu_psys_graph_info *)arg);
	} else if (psys || verbose) {
		/* Every psys ioctl, so a HAL using different numbers is visible
		 * rather than producing a silent empty file. */
		fprintf(sink(), "graph-trace: ioctl fd=%d req=0x%lx (dir=%lu type='%c' nr=%lu size=%lu) via %s%s\n",
			fd, req,
			(req >> 30) & 3UL, (char)((req >> 8) & 0xff),
			req & 0xffUL, (req >> 16) & 0x3fffUL, via,
			psys ? " [psys]" : "");
	}
}

int ioctl(int fd, unsigned long req, ...)
{
	static int (*real)(int, unsigned long, ...);
	va_list ap;
	void *arg;

	if (!real) real = dlsym(RTLD_NEXT, "ioctl");
	va_start(ap, req);
	arg = va_arg(ap, void *);
	va_end(ap);

	watch(fd, req, arg, "ioctl()");
	int ret = real(fd, req, arg);
	if (req == (unsigned long)IPU_IOC_GRAPH_OPEN)
		fprintf(sink(), "graph-trace: GRAPH_OPEN returned %d%s\n",
			ret, ret < 0 ? " (FAILED)" : "");
	return ret;
}

/* A caller using syscall(SYS_ioctl, ...) never reaches the wrapper above. */
long syscall(long num, ...)
{
	static long (*real)(long, ...);
	va_list ap;
	long a[6];

	if (!real) real = dlsym(RTLD_NEXT, "syscall");
	va_start(ap, num);
	for (int i = 0; i < 6; i++) a[i] = va_arg(ap, long);
	va_end(ap);

	if (num == SYS_ioctl)
		watch((int)a[0], (unsigned long)a[1], (void *)a[2], "syscall()");

	return real(num, a[0], a[1], a[2], a[3], a[4], a[5]);
}
