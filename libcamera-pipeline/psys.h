/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Contributors
 *
 * Intel IPU7 PSYS device wrapper
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/unique_fd.h>

namespace libcamera {

/**
 * \brief IPU7 PSYS node profile bitmaps
 *
 * Hardware routing configuration for a processing node. These bitmaps
 * control which internal processing blocks are enabled and how data
 * flows through the firmware pipeline.
 */
struct PsysNodeProfile {
	uint32_t teb[2];  /**< Terminal Enable Bitmap */
	uint32_t deb[4];  /**< Device Enable Bitmap */
	uint32_t rbm[4];  /**< Routing Bitmap */
	uint32_t reb[4];  /**< Routing Enable Bitmap */
};

/**
 * \brief Terminal descriptor for a PSYS processing node
 *
 * Each terminal represents a data connection point on a node —
 * either an input (raw frame, parameters) or output (processed frame,
 * statistics).
 */
struct PsysTerminal {
	uint8_t termId;
	uint32_t bufSize;
};

/**
 * \brief Processing node in a PSYS graph
 *
 * A node represents one stage of the ISP pipeline. The IPU7 typically
 * uses two nodes: ISA (pre-processing/demosaic) and OFS (scaling/output).
 */
struct PsysNode {
	uint8_t nodeRsrcId;  /**< Physical node ID */
	uint8_t nodeCtxId;   /**< Logical context ID (unique per graph) */
	PsysNodeProfile profile;
	std::vector<PsysTerminal> terminals;
};

/**
 * \brief Link endpoint in a PSYS graph
 */
struct PsysLinkEndpoint {
	uint8_t nodeCtxId;
	uint8_t termId;
};

/**
 * \brief Link between terminals in a PSYS graph
 */
struct PsysLink {
	PsysLinkEndpoint src;
	PsysLinkEndpoint dst;
	uint16_t foreignKey;
	uint8_t streamingMode;
	uint8_t pbkId;
	uint8_t pbkSlotId;
	uint8_t delayedLink;
};

/**
 * \brief DMA-BUF backed buffer for PSYS terminal data
 */
struct PsysBuffer {
	UniqueFD fd;
	uint64_t len;
	uint32_t dataOffset;
	uint32_t bytesUsed;
	uint32_t flags;
	void *userptr = nullptr; /**< User-space backing memory (page-aligned) */
};

/**
 * \brief Wrapper around the /dev/ipu7-psys0 device
 *
 * The PSYS device uses custom ioctls (not V4L2) to manage processing
 * graphs, allocate buffers, submit frame processing tasks, and receive
 * completion events.
 *
 * Typical sequence:
 *  1. open() the device
 *  2. getbuf() + mapbuf() for each terminal buffer
 *  3. graphOpen() with node/link topology
 *  4. Per frame: taskRequest() for each node, then dqevent() for completion
 *  5. graphClose() when done
 */
class PsysDevice
{
public:
	PsysDevice();
	~PsysDevice();

	int open(const std::string &deviceNode);
	void close();
	bool isOpen() const { return fd_.isValid(); }
	int fd() const { return fd_.isValid() ? fd_.get() : -1; }

	int graphOpen(std::vector<PsysNode> &nodes,
		      const std::vector<PsysLink> &links,
		      uint8_t *graphId);
	int graphClose(uint8_t graphId);

	int getBuf(uint64_t len, uint32_t flags, PsysBuffer *buf);
	int mapBuf(int bufFd);
	int unmapBuf(int bufFd);

	int taskRequest(uint8_t graphId, uint8_t nodeCtxId, uint8_t frameId,
			const uint32_t payloadReuseBm[2],
			const std::vector<std::pair<uint8_t, PsysBuffer *>> &termBufs);

	int dqEvent(uint8_t *graphId, uint8_t *nodeCtxId,
		    uint8_t *frameId, uint32_t *error);

private:
	int ioctlWrapper(unsigned long request, void *arg);

	UniqueFD fd_;
};

} /* namespace libcamera */
