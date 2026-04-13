/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Contributors
 *
 * Intel IPU7 PSYS device wrapper
 */

#include "psys.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <libcamera/base/log.h>

/*
 * Include the PSYS UAPI header. This defines the ioctl command codes
 * and the packed structs used to communicate with the kernel driver.
 */
#define __user
#include "uapi/ipu7-psys.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPU7)

PsysDevice::PsysDevice()
{
}

PsysDevice::~PsysDevice()
{
	close();
}

int PsysDevice::open(const std::string &deviceNode)
{
	if (fd_.isValid()) {
		LOG(IPU7, Error) << "PSYS device already open";
		return -EBUSY;
	}

	int fd = ::open(deviceNode.c_str(), O_RDWR);
	if (fd < 0) {
		int ret = -errno;
		LOG(IPU7, Error) << "Failed to open " << deviceNode
				 << ": " << strerror(-ret);
		return ret;
	}

	fd_ = UniqueFD(fd);

	LOG(IPU7, Debug) << "Opened PSYS device " << deviceNode;
	return 0;
}

void PsysDevice::close()
{
	fd_.reset();
}

int PsysDevice::ioctlWrapper(unsigned long request, void *arg)
{
	int ret;
	do {
		ret = ::ioctl(fd_.get(), request, arg);
	} while (ret < 0 && errno == EINTR);

	return ret < 0 ? -errno : ret;
}

/**
 * \brief Open a processing graph on the PSYS device
 * \param[in] nodes Vector of processing nodes (ISA, OFS, etc.)
 * \param[in] links Vector of links connecting node terminals
 * \param[out] graphId Graph ID assigned by the driver
 *
 * The graph describes the ISP processing topology: which nodes are active,
 * their terminal configurations (profile bitmaps), and how data flows
 * between them via links.
 *
 * \return 0 on success, negative error code on failure
 */
int PsysDevice::graphOpen(std::vector<PsysNode> &nodes,
			   const std::vector<PsysLink> &links,
			   uint8_t *graphId)
{
	if (nodes.size() > MAX_GRAPH_NODES) {
		LOG(IPU7, Error) << "Too many nodes: " << nodes.size();
		return -EINVAL;
	}

	LOG(IPU7, Debug) << "GRAPH_OPEN: " << nodes.size() << " nodes, "
			 << links.size() << " links";

	/* Build the kernel graph_node array */
	std::vector<struct graph_node> knodes(nodes.size());
	for (size_t n = 0; n < nodes.size(); n++) {
		struct graph_node &kn = knodes[n];
		memset(&kn, 0, sizeof(kn));

		kn.node_rsrc_id = nodes[n].nodeRsrcId;
		kn.node_ctx_id = nodes[n].nodeCtxId;
		kn.num_terms = nodes[n].terminals.size();

		LOG(IPU7, Debug) << "  Node[" << n << "]: rsrc=" << static_cast<int>(kn.node_rsrc_id)
				 << " ctx=" << static_cast<int>(kn.node_ctx_id)
				 << " terms=" << static_cast<int>(kn.num_terms);
		LOG(IPU7, Debug) << "    teb=[0x" << std::hex << nodes[n].profile.teb[0]
				 << ",0x" << nodes[n].profile.teb[1] << "]" << std::dec;

		/* Copy profile bitmaps */
		memcpy(kn.profiles[0].teb, nodes[n].profile.teb, sizeof(kn.profiles[0].teb));
		memcpy(kn.profiles[0].deb, nodes[n].profile.deb, sizeof(kn.profiles[0].deb));
		memcpy(kn.profiles[0].rbm, nodes[n].profile.rbm, sizeof(kn.profiles[0].rbm));
		memcpy(kn.profiles[0].reb, nodes[n].profile.reb, sizeof(kn.profiles[0].reb));

		/* Copy terminals */
		for (size_t t = 0; t < nodes[n].terminals.size() && t < MAX_GRAPH_TERMINALS; t++) {
			kn.terminals[t].term_id = nodes[n].terminals[t].termId;
			kn.terminals[t].buf_size = nodes[n].terminals[t].bufSize;
			LOG(IPU7, Debug) << "    term[" << t << "]: id="
					 << static_cast<int>(kn.terminals[t].term_id)
					 << " size=" << kn.terminals[t].buf_size;
		}
	}

	/* Build the graph info struct */
	struct ipu_psys_graph_info gi;
	memset(&gi, 0, sizeof(gi));
	gi.graph_id = 0xFF; /* Driver assigns the actual ID */
	gi.num_nodes = nodes.size();
	gi.nodes = knodes.data();

	/* Copy links */
	LOG(IPU7, Debug) << "  Links:";
	for (size_t i = 0; i < links.size() && i < MAX_GRAPH_LINKS; i++) {
		LOG(IPU7, Debug) << "    link[" << i << "]: src(node="
				 << static_cast<int>(links[i].src.nodeCtxId)
				 << ",term=" << static_cast<int>(links[i].src.termId)
				 << ") -> dst(node="
				 << static_cast<int>(links[i].dst.nodeCtxId)
				 << ",term=" << static_cast<int>(links[i].dst.termId)
				 << ") smode=" << static_cast<int>(links[i].streamingMode)
				 << " delayed=" << static_cast<int>(links[i].delayedLink);
		gi.links[i].ep_src.node_ctx_id = links[i].src.nodeCtxId;
		gi.links[i].ep_src.term_id = links[i].src.termId;
		gi.links[i].ep_dst.node_ctx_id = links[i].dst.nodeCtxId;
		gi.links[i].ep_dst.term_id = links[i].dst.termId;
		gi.links[i].foreign_key = links[i].foreignKey;
		gi.links[i].streaming_mode = links[i].streamingMode;
		gi.links[i].pbk_id = links[i].pbkId;
		gi.links[i].pbk_slot_id = links[i].pbkSlotId;
		gi.links[i].delayed_link = links[i].delayedLink;
	}

	int ret = ioctlWrapper(IPU_IOC_GRAPH_OPEN, &gi);
	if (ret < 0) {
		LOG(IPU7, Error) << "GRAPH_OPEN failed: " << strerror(-ret);
		return ret;
	}

	*graphId = gi.graph_id;
	LOG(IPU7, Debug) << "Graph opened, id=" << static_cast<int>(*graphId);
	return 0;
}

/**
 * \brief Close a processing graph
 * \param[in] graphId The graph ID returned by graphOpen()
 */
int PsysDevice::graphClose(uint8_t graphId)
{
	int id = graphId;
	int ret = ioctlWrapper(IPU_IOC_GRAPH_CLOSE, &id);
	if (ret < 0)
		LOG(IPU7, Error) << "GRAPH_CLOSE failed: " << strerror(-ret);
	return ret;
}

/**
 * \brief Allocate a DMA-BUF buffer for terminal data
 * \param[in] len Buffer size in bytes
 * \param[in] flags Buffer flags (IPU_BUFFER_FLAG_*)
 * \param[out] buf Allocated buffer descriptor
 */
int PsysDevice::getBuf(uint64_t len, uint32_t flags, PsysBuffer *buf)
{
	/*
	 * The PSYS GETBUF ioctl requires a page-aligned userptr — it wraps
	 * user-allocated memory into a DMA-BUF. We allocate page-aligned
	 * memory, pass it to the driver, and get back a DMA-BUF fd.
	 */
	size_t pageSize = sysconf(_SC_PAGESIZE);
	size_t alignedLen = ((len + pageSize - 1) / pageSize) * pageSize;

	void *userptr = nullptr;
	int ret = posix_memalign(&userptr, pageSize, alignedLen);
	if (ret || !userptr) {
		LOG(IPU7, Error) << "Failed to allocate " << alignedLen
				 << " bytes for GETBUF: " << strerror(ret);
		return -ENOMEM;
	}
	memset(userptr, 0, alignedLen);

	struct ipu_psys_buffer kb;
	memset(&kb, 0, sizeof(kb));
	kb.len = alignedLen;
	kb.base.userptr = userptr;
	kb.flags = flags | IPU_BUFFER_FLAG_USERPTR;

	LOG(IPU7, Debug) << "GETBUF request: len=" << len << " (aligned=" << alignedLen
			 << ") userptr=" << userptr
			 << " flags=0x" << std::hex << kb.flags << std::dec;

	ret = ioctlWrapper(IPU_IOC_GETBUF, &kb);
	if (ret < 0) {
		LOG(IPU7, Error) << "GETBUF failed for size " << len
				 << ": " << strerror(-ret);
		free(userptr);
		return ret;
	}

	buf->fd = UniqueFD(kb.base.fd);
	buf->len = alignedLen;
	buf->dataOffset = kb.data_offset;
	buf->bytesUsed = kb.bytes_used;
	buf->flags = kb.flags;
	buf->userptr = userptr;

	LOG(IPU7, Debug) << "GETBUF: len=" << len << " -> fd=" << buf->fd.get()
			 << " userptr=" << userptr;

	return 0;
}

/**
 * \brief Map a buffer FD into the PSYS device's IOMMU
 * \param[in] bufFd DMA-BUF file descriptor
 */
int PsysDevice::mapBuf(int bufFd)
{
	/*
	 * MAPBUF is special — the kernel reads arg directly as an int, NOT
	 * through copy_from_user (see ipu-psys.c line 1072). So we pass
	 * the fd value itself cast to a pointer, not a pointer to the fd.
	 */
	int ret = ioctlWrapper(IPU_IOC_MAPBUF,
			       reinterpret_cast<void *>(static_cast<intptr_t>(bufFd)));
	if (ret < 0)
		LOG(IPU7, Error) << "MAPBUF failed for fd " << bufFd
				 << ": " << strerror(-ret);
	else
		LOG(IPU7, Debug) << "MAPBUF: fd=" << bufFd << " mapped";
	return ret;
}

/**
 * \brief Unmap a buffer FD from the PSYS device's IOMMU
 * \param[in] bufFd DMA-BUF file descriptor
 */
int PsysDevice::unmapBuf(int bufFd)
{
	/* Same as MAPBUF — arg is the raw fd value, not a pointer */
	int ret = ioctlWrapper(IPU_IOC_UNMAPBUF,
			       reinterpret_cast<void *>(static_cast<intptr_t>(bufFd)));
	if (ret < 0)
		LOG(IPU7, Error) << "UNMAPBUF failed for fd " << bufFd
				 << ": " << strerror(-ret);
	return ret;
}

/**
 * \brief Submit a frame processing task for one node
 * \param[in] graphId Graph ID
 * \param[in] nodeCtxId Node context ID (0=ISA, 1=OFS)
 * \param[in] frameId Frame sequence number
 * \param[in] payloadReuseBm Payload reuse bitmap (usually {0,0})
 * \param[in] termBufs Terminal ID → buffer pairs
 *
 * Each node in the graph must have a separate task request per frame.
 * The ISA node is submitted first, then the OFS node. Completion is
 * signaled via dqEvent().
 */
int PsysDevice::taskRequest(uint8_t graphId, uint8_t nodeCtxId, uint8_t frameId,
			     const uint32_t payloadReuseBm[2],
			     const std::vector<std::pair<uint8_t, PsysBuffer *>> &termBufs)
{
	/* Build the terminal buffer array */
	std::vector<struct ipu_psys_term_buffers> kbufs(termBufs.size());
	for (size_t i = 0; i < termBufs.size(); i++) {
		memset(&kbufs[i], 0, sizeof(kbufs[i]));
		kbufs[i].term_id = termBufs[i].first;
		kbufs[i].term_buf.len = termBufs[i].second->len;
		kbufs[i].term_buf.base.fd = termBufs[i].second->fd.get();
		kbufs[i].term_buf.data_offset = termBufs[i].second->dataOffset;
		kbufs[i].term_buf.bytes_used = termBufs[i].second->bytesUsed;
		kbufs[i].term_buf.flags = termBufs[i].second->flags;
	}

	struct ipu_psys_task_request tr;
	memset(&tr, 0, sizeof(tr));
	tr.graph_id = graphId;
	tr.node_ctx_id = nodeCtxId;
	tr.frame_id = frameId;
	tr.payload_reuse_bm[0] = payloadReuseBm[0];
	tr.payload_reuse_bm[1] = payloadReuseBm[1];
	tr.term_buf_count = termBufs.size();
	tr.task_buffers = kbufs.data();

	LOG(IPU7, Debug) << "TASK_REQUEST: graph=" << static_cast<int>(graphId)
			 << " node=" << static_cast<int>(nodeCtxId)
			 << " frame=" << static_cast<int>(frameId)
			 << " bufs=" << termBufs.size()
			 << " — submitting ioctl...";

	int ret = ioctlWrapper(IPU_IOC_TASK_REQUEST, &tr);
	if (ret < 0) {
		LOG(IPU7, Error) << "TASK_REQUEST failed for node " << static_cast<int>(nodeCtxId)
				 << " frame " << static_cast<int>(frameId)
				 << ": " << strerror(-ret);
		return ret;
	}

	return 0;
}

/**
 * \brief Dequeue a completion event
 * \param[out] graphId Graph that completed
 * \param[out] nodeCtxId Node that completed
 * \param[out] frameId Frame that completed
 * \param[out] error Error code (0 = success)
 *
 * This call blocks until a task completes. In a pipeline handler,
 * it should be driven by poll/epoll on the PSYS fd.
 */
int PsysDevice::dqEvent(uint8_t *graphId, uint8_t *nodeCtxId,
			  uint8_t *frameId, uint32_t *error)
{
	struct ipu_psys_event ev;
	memset(&ev, 0, sizeof(ev));

	int ret = ioctlWrapper(IPU_IOC_DQEVENT, &ev);
	if (ret < 0) {
		if (ret != -EAGAIN)
			LOG(IPU7, Error) << "DQEVENT failed: " << strerror(-ret);
		return ret;
	}

	*graphId = ev.graph_id;
	*nodeCtxId = ev.node_ctx_id;
	*frameId = ev.frame_id;
	*error = ev.error;

	return 0;
}

} /* namespace libcamera */
