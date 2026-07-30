/*
 * graph-trace.c -- dump the psys graph the Intel Camera HAL submits.
 *
 * LD_PRELOAD shim over ioctl(2). When the HAL issues IPU_IOC_GRAPH_OPEN on
 * /dev/ipu7-psys0 we print the whole struct ipu_psys_graph_info -- every node,
 * its profile bitmaps, every terminal and buffer size, and every link -- then
 * pass the call through untouched.
 *
 * WHY THIS EXISTS
 *   The libcamera IPU7 pipeline handler submits a hardcoded graph captured
 *   from Panther Lake silicon. Lunar Lake runs different psys firmware with
 *   different program groups and rejects it:
 *
 *     ipu7_psys_handle_graph_open_ack, num_items is 0
 *     Failed to set graph
 *     GRAPH_OPEN failed: Invalid argument
 *
 *   The Intel Camera HAL *does* work on LNL, because it builds its graph from
 *   the per-platform GCSS descriptor in /etc/camera/ipu7x/gcss/. Rather than
 *   reverse-engineering that binary format, just watch what the HAL sends.
 *
 * WHAT IT IS NOT
 *   It does not modify anything, write to the device, or change what the HAL
 *   does. It reads the struct on its way past and prints it.
 *
 * BUILD
 *   gcc -shared -fPIC -O2 -o graph-trace.so graph-trace.c -ldl
 *
 * USE
 *   LD_PRELOAD=./graph-trace.so GRAPH_TRACE_OUT=/tmp/graph.txt \
 *     gst-launch-1.0 icamerasrc ! fakesink num-buffers=5
 */

#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>

/* Mirrors uapi/ipu7-psys.h. Layouts must match exactly -- everything is packed
 * and the kernel copies these byte for byte. */
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
static int   seen;

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

static void dump(const struct ipu_psys_graph_info *g)
{
	FILE *f = sink();

	fprintf(f, "\n=== IPU_IOC_GRAPH_OPEN #%d ===\n", ++seen);
	fprintf(f, "graph_id=0x%02x num_nodes=%u\n", g->graph_id, g->num_nodes);

	if (!g->nodes) {
		fprintf(f, "  (nodes pointer is NULL)\n");
		return;
	}

	unsigned n = g->num_nodes > MAX_GRAPH_NODES ? MAX_GRAPH_NODES : g->num_nodes;
	for (unsigned i = 0; i < n; i++) {
		const struct graph_node *nd = &g->nodes[i];
		unsigned t = nd->num_terms > MAX_GRAPH_TERMINALS
			   ? MAX_GRAPH_TERMINALS : nd->num_terms;

		fprintf(f, "\nNode[%u]: rsrc=%u ctx=%u terms=%u\n",
			i, nd->node_rsrc_id, nd->node_ctx_id, nd->num_terms);

		/* Printed as C initialisers so they can be pasted straight into
		 * a PsysNodeProfile in the pipeline handler. */
		const struct node_profile *p = &nd->profiles[0];
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
		/* The array is fixed-size and zero-padded; a link with every
		 * field zero is an empty slot, not a real link. */
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

int ioctl(int fd, unsigned long req, ...)
{
	static int (*real)(int, unsigned long, ...);
	va_list ap;
	void *arg;

	if (!real)
		real = dlsym(RTLD_NEXT, "ioctl");

	va_start(ap, req);
	arg = va_arg(ap, void *);
	va_end(ap);

	/* Dump BEFORE the call: the kernel writes graph_id back on success, but
	 * on failure it may leave the struct untouched, and a rejected graph is
	 * exactly the one worth seeing. */
	if (req == (unsigned long)IPU_IOC_GRAPH_OPEN && arg)
		dump((const struct ipu_psys_graph_info *)arg);

	int ret = real(fd, req, arg);

	if (req == (unsigned long)IPU_IOC_GRAPH_OPEN)
		fprintf(sink(), "GRAPH_OPEN returned %d%s\n", ret,
			ret < 0 ? " (FAILED)" : "");

	return ret;
}
