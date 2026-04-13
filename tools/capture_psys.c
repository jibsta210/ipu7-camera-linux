#define _GNU_SOURCE
#include <dlfcn.h>
#include <sys/ioctl.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

struct ipu_psys_buffer {
    uint64_t len;
    union { int fd; void *userptr; uint64_t reserved; } base;
    uint32_t data_offset;
    uint32_t bytes_used;
    uint32_t flags;
    uint32_t reserved[2];
} __attribute__((packed));

struct ipu_psys_term_buffers {
    uint8_t term_id;
    struct ipu_psys_buffer term_buf;
} __attribute__((packed));

struct ipu_psys_task_request {
    uint8_t graph_id;
    uint8_t node_ctx_id;
    uint8_t frame_id;
    uint32_t payload_reuse_bm[2];
    uint8_t term_buf_count;
    struct ipu_psys_term_buffers *task_buffers;
} __attribute__((packed));

/* Map fd → userptr from GETBUF */
#define MAX_BUFS 256
static struct { int fd; void *userptr; uint64_t len; } bufs[MAX_BUFS];
static int nbuf = 0;

static int (*real_ioctl)(int fd, unsigned long request, ...) = NULL;
static int psys_fd = -1;
static int nodes_captured = 0;

static void *find_userptr(int target_fd) {
    for (int i = 0; i < nbuf; i++)
        if (bufs[i].fd == target_fd) return bufs[i].userptr;
    return NULL;
}

int ioctl(int fd, unsigned long request, ...) {
    if (!real_ioctl)
        real_ioctl = dlsym(RTLD_NEXT, "ioctl");

    va_list ap;
    va_start(ap, request);
    void *arg = va_arg(ap, void *);
    va_end(ap);

    unsigned int nr = request & 0xff;
    unsigned int type = (request >> 8) & 0xff;

    /* Detect PSYS fd */
    if (psys_fd < 0 && type == 'A') {
        char path[256], target[256];
        snprintf(path, sizeof(path), "/proc/self/fd/%d", fd);
        ssize_t len = readlink(path, target, sizeof(target)-1);
        if (len > 0) { target[len] = 0; if (strstr(target, "psys")) psys_fd = fd; }
    }

    if (fd != psys_fd || type != 'A')
        return real_ioctl(fd, request, arg);

    /* GETBUF (nr=4): record userptr before, fd after */
    if (nr == 4 && arg) {
        struct ipu_psys_buffer *buf = arg;
        void *saved_userptr = buf->base.userptr;
        uint64_t saved_len = buf->len;

        int ret = real_ioctl(fd, request, arg);
        if (ret == 0 && nbuf < MAX_BUFS) {
            bufs[nbuf].fd = buf->base.fd;  /* after ioctl, base = fd */
            bufs[nbuf].userptr = saved_userptr;
            bufs[nbuf].len = saved_len;
            fprintf(stderr, "[CAP] GETBUF: fd=%d userptr=%p len=%lu\n",
                    buf->base.fd, saved_userptr, (unsigned long)saved_len);
            nbuf++;
        }
        return ret;
    }

    /* TASK_REQUEST (nr=8): dump param buffers */
    if (nr == 8 && arg && nodes_captured < 2) {
        struct ipu_psys_task_request *req = arg;

        if (req->frame_id == 0 && req->task_buffers) {
            fprintf(stderr, "[CAP] TASK node=%d terms=%d\n",
                    req->node_ctx_id, req->term_buf_count);

            for (int i = 0; i < req->term_buf_count; i++) {
                struct ipu_psys_term_buffers *tb = &req->task_buffers[i];
                int buf_fd = tb->term_buf.base.fd;
                void *uptr = find_userptr(buf_fd);
                uint64_t len = tb->term_buf.len;

                fprintf(stderr, "[CAP]   term %d: fd=%d len=%lu flags=0x%x uptr=%p\n",
                        tb->term_id, buf_fd, (unsigned long)len,
                        tb->term_buf.flags, uptr);

                if (uptr && len > 0 && len < 200000) {
                    char fname[256];
                    snprintf(fname, sizeof(fname), "/tmp/hal4k_node%d_term%02d.bin",
                             req->node_ctx_id, tb->term_id);
                    FILE *f = fopen(fname, "wb");
                    if (f) {
                        fwrite(uptr, 1, len, f);
                        fclose(f);
                        fprintf(stderr, "[CAP]     -> %s\n", fname);
                    }
                }
            }
            nodes_captured++;
        }
    }

    return real_ioctl(fd, request, arg);
}
