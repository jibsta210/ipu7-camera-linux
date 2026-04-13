/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Contributors
 *
 * Pipeline handler for Intel IPU7
 *
 * The IPU7 ISP has two subsystems:
 *  - ISYS: Input System — captures raw Bayer frames from the sensor via CSI-2.
 *    Exposed as standard V4L2 video devices (/dev/videoN) under a media
 *    controller (/dev/media0).
 *  - PSYS: Processing System — runs the ISP pipeline (demosaic, NR, scaling).
 *    Exposed as a custom char device (/dev/ipu7-psys0) with proprietary ioctls,
 *    NOT V4L2.
 *
 * The pipeline handler captures raw frames from ISYS, submits them to PSYS
 * for processing through a two-node graph (ISA → OFS), and delivers the
 * processed NV12 output to the application.
 */

#include <algorithm>
#include <cstring>
#include <linux/dma-buf.h>
#include <memory>
#include <queue>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/dma_buf_allocator.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/camera_sensor_properties.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/request.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "psys.h"

#define __user
#include "uapi/ipu7-psys.h"

/* Captured ISP parameter data from Camera HAL */
#include "params/isp_params.h"

/*
 * Verbose per-frame tracing. Set to 1 for development/debugging,
 * 0 for production builds. Even at Debug log level, per-frame
 * logging can be overwhelming — this gates the noisiest output.
 *
 * Runtime log levels (no recompile needed):
 *   LIBCAMERA_LOG_LEVELS=IPU7:0  — show all debug messages
 *   LIBCAMERA_LOG_LEVELS=IPU7:1  — info and above (default)
 *   LIBCAMERA_LOG_LEVELS=IPU7:2  — warnings and errors only
 */
#define IPU7_VERBOSE_FRAME_LOG 1

namespace libcamera {

LOG_DEFINE_CATEGORY(IPU7)

/* --------------------------------------------------------------------------
 * Graph topology constants — captured from Camera HAL via ioctl intercept.
 *
 * These define the ISA+OFS two-node processing graph that the IPU7 firmware
 * expects for standard camera capture with the OV08X40 sensor.
 *
 * \todo Make these configurable per sensor/resolution instead of hardcoded.
 * The Camera HAL reads these from JSON config files; we'll need to either
 * parse those or maintain our own per-sensor configurations.
 * -------------------------------------------------------------------------- */

/* ISA node (pre-processing / demosaic) */
static const PsysNodeProfile isaProfile = {
	{ 0x000c3d27, 0x00000000 },			/* teb */
	{ 0xd41bf1f1, 0x00001fff, 0, 0 },		/* deb */
	{ 0xa81f9009, 0x0000001e, 0, 0 },		/* rbm */
	{ 0x007b7fe7, 0x00000000, 0, 0 },		/* reb */
};

/*
 * Terminal buffer sizes for 1920x1080 (captured from Camera HAL).
 * Full-resolution (3856x2176) sizes are computed by scaling image-dependent
 * terminals by the area ratio, keeping non-image terminals unchanged.
 */
static const PsysTerminal isaTerminals_1080p[] = {
	{  2,     2688 },
	{  1,      896 },
	{ 19,  3113472 },	/* output: processed image → OFS */
	{  0,    28288 },
	{ 13,     4096 },
	{ 12,    32640 },
	{ 11,     1920 },
	{ 10,     8192 },
	{ 18,   196352 },	/* output: statistics → OFS */
	{  5, 16850944 },	/* input: raw Bayer frame */
	{  8,    17920 },
};

/*
 * 4K ISA terminals — captured from Intel Camera HAL at 3840x2160.
 * Buffer sizes differ significantly from 1080p (not a simple ratio).
 */
static const PsysTerminal isaTerminals_4k[] = {
	{  2,     2688 },
	{  1,      896 },
	{ 19, 24894720 },	/* output: processed image → OFS */
	{  0,    28288 },
	{ 13,     4032 },
	{ 12,    32192 },
	{ 11,     1792 },
	{ 10,     8192 },
	{ 18,  1558080 },	/* output: statistics → OFS */
	{  5, 16858688 },	/* input: raw Bayer frame */
	{  8,    17920 },
};

/* OFS node (output formatting / scaler) */
static const PsysNodeProfile ofsProfile = {
	{ 0x00007fef, 0x00000000 },			/* teb */
	{ 0x0000ffff, 0x00000000, 0, 0 },		/* deb */
	{ 0x0000001f, 0x00000000, 0, 0 },		/* rbm */
	{ 0x0000000f, 0x00000000, 0, 0 },		/* reb */
};

static const PsysTerminal ofsTerminals_1080p[] = {
	{  3,     2624 },
	{  9,  3113472 },	/* input: image from ISA */
	{  7,   196352 },	/* input: statistics from ISA */
	{ 12,  3113472 },	/* temporal ref output → self delayed */
	{ 10,  3113472 },	/* temporal ref input (delayed) */
	{ 13,   196352 },	/* temporal stats output → self delayed */
	{  0,     2176 },
	{  5,   196352 },	/* temporal stats input (delayed) */
	{  8,   130560 },	/* NR working → self */
	{ 11,   130560 },	/* NR input from self */
	{  6,   130560 },	/* NR delayed input */
	{ 14,  3110400 },	/* OUTPUT: final NV12 1920×1080 */
	{  1,       64 },
	{  2,     1728 },
};

/*
 * 4K OFS terminals — captured from Intel Camera HAL at 3840x2160.
 */
static const PsysTerminal ofsTerminals_4k[] = {
	{  3,     2624 },
	{  9, 24894720 },	/* input: image from ISA */
	{  7,  1558080 },	/* input: statistics from ISA */
	{ 12, 12441600 },	/* temporal ref output → self delayed */
	{ 10, 12441600 },	/* temporal ref input (delayed) */
	{ 13,   785408 },	/* temporal stats output → self delayed */
	{  0,     2176 },
	{  5,   785408 },	/* temporal stats input (delayed) */
	{  8,   522240 },	/* NR working → self */
	{ 11,   522240 },	/* NR input from self */
	{  6,   522240 },	/* NR delayed input */
	{ 14, 12447360 },	/* OUTPUT: final NV12 3840×2160 */
	{  1,       64 },
	{  2,     1728 },
};

/* Links between nodes */
static const PsysLink graphLinks[] = {
	/* ISA → OFS: processed image */
	{ { 0, 19 }, { 1, 9 }, 0xFFFF, 2, 255, 255, 0 },
	/* ISA → OFS: statistics */
	{ { 0, 18 }, { 1, 7 }, 0xFFFF, 2, 255, 255, 0 },
	/* OFS → OFS: temporal reference (delayed) */
	{ { 1, 12 }, { 1, 10 }, 0xFFFF, 0, 255, 255, 1 },
	/* OFS → OFS: temporal statistics (delayed) */
	{ { 1, 13 }, { 1, 5 }, 0xFFFF, 2, 255, 255, 1 },
	/* OFS → OFS: NR working (same frame) */
	{ { 1, 8 }, { 1, 11 }, 0xFFFF, 2, 255, 255, 0 },
	/* OFS → OFS: NR working (delayed) */
	{ { 1, 8 }, { 1, 6 }, 0xFFFF, 0, 255, 255, 1 },
};

/* -------------------------------------------------------------------------- */

class IPU7CameraData : public Camera::Private
{
public:
	IPU7CameraData(PipelineHandler *pipe)
		: Camera::Private(pipe)
	{
	}

	~IPU7CameraData()
	{
		freeBuffers();
		if (graphOpen_)
			psys_.graphClose(graphId_);
		psys_.close();
	}

	int init(std::shared_ptr<MediaDevice> media, unsigned int csiIndex);
	int setupPsys(const std::string &psysDevNode);

	void isysBufferReady(FrameBuffer *buffer);

	/* ISYS (V4L2) */
	std::unique_ptr<CameraSensor> sensor_;
	std::unique_ptr<V4L2Subdevice> csi2_;
	std::unique_ptr<V4L2VideoDevice> isysCapture_;

	/* DMA-BUF allocator for NV12 output buffers */
	DmaBufAllocator dmaBufAllocator_{
		DmaBufAllocator::DmaBufAllocatorFlag::CmaHeap |
		DmaBufAllocator::DmaBufAllocatorFlag::SystemHeap |
		DmaBufAllocator::DmaBufAllocatorFlag::UDmaBuf
	};

	/* PSYS (custom ioctls) */
	PsysDevice psys_;
	uint8_t graphId_ = 0;
	bool graphOpen_ = false;

	Stream outputStream_;
	MediaLink *isysCaptureLink_ = nullptr;
	Size outputSize_{ 1920, 1080 };	/* configured output resolution */
	Size psysSize_;			/* resolution PSYS was initialized for */

	/* PSYS terminal buffers — one set per node per terminal */
	struct TermBufSet {
		uint8_t termId;
		PsysBuffer buf;
	};
	std::vector<TermBufSet> isaTermBufs_;
	std::vector<TermBufSet> ofsTermBufs_;

	/* Raw frame buffers from ISYS */
	std::vector<std::unique_ptr<FrameBuffer>> isysBuffers_;
	std::queue<FrameBuffer *> availableIsysBuffers_;

	/* Pending requests */
	std::queue<Request *> pendingRequests_;
	uint8_t frameCounter_ = 0;

	/* Simple auto-exposure state */
	static constexpr int32_t kAeExposureMin = 4;
	static constexpr int32_t kAeExposureMax = 4992;
	static constexpr int32_t kAeAGainMin = 128;
	static constexpr int32_t kAeAGainMax = 1984;
	static constexpr int32_t kAeDGainMin = 1024;
	static constexpr int32_t kAeDGainMax = 4095;
	static constexpr uint8_t kAeTargetLuma = 115;
	static constexpr unsigned int kAeSkipFrames = 3;

	struct {
		int32_t exposure = 4992;
		int32_t analogueGain = 800;
		int32_t digitalGain = 2560;
		unsigned int framesSinceUpdate = 0;
		bool enabled = true;
		float brightnessTarget = 0.45f; /* 0.0-1.0, maps to luma */
	} ae_;

	void updateAutoExposure(const void *nv12Data, unsigned int width,
				unsigned int height);

	int allocateTerminalBuffers();
	void loadTerminalParams();
	void freeBuffers();

private:
	int openGraph();
	void processRawFrame(FrameBuffer *rawBuffer, Request *request);
};

class IPU7CameraConfiguration : public CameraConfiguration
{
public:
	static constexpr unsigned int kBufferCount = 4;

	IPU7CameraConfiguration(IPU7CameraData *data);

	Status validate() override;

private:
	const IPU7CameraData *data_;
};

class PipelineHandlerIPU7 : public PipelineHandler
{
public:
	PipelineHandlerIPU7(CameraManager *manager);

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								    Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

private:
	IPU7CameraData *cameraData(Camera *camera)
	{
		return static_cast<IPU7CameraData *>(camera->_d());
	}

	int registerCameras();

	std::shared_ptr<MediaDevice> isysMedia_;
	std::string psysDevNode_;
};

/* --------------------------------------------------------------------------
 * IPU7CameraConfiguration
 * -------------------------------------------------------------------------- */

IPU7CameraConfiguration::IPU7CameraConfiguration(IPU7CameraData *data)
	: CameraConfiguration(), data_(data)
{
}

CameraConfiguration::Status IPU7CameraConfiguration::validate()
{
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	/* Only support a single output stream for now */
	if (config_.size() > 1) {
		config_.resize(1);
		status = Adjusted;
	}

	StreamConfiguration &cfg = config_[0];

	/* Force NV12 output — that's what the OFS node produces */
	if (cfg.pixelFormat != formats::NV12) {
		cfg.pixelFormat = formats::NV12;
		status = Adjusted;
	}

	/* Accept 1080p, 1440p, or full-resolution output */
	if (cfg.size != Size(1920, 1080) &&
	    cfg.size != Size(2560, 1440) &&
	    cfg.size != Size(3840, 2160)) {
		cfg.size = Size(1920, 1080);
		status = Adjusted;
	}

	cfg.bufferCount = kBufferCount;

	const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);
	cfg.stride = info.stride(cfg.size.width, 0, 1);
	cfg.frameSize = info.frameSize(cfg.size, 1);

	/* Sensor is mounted upside-down; tell clients to rotate 180° */
	orientation = Orientation::Rotate180;

	return status;
}

/* --------------------------------------------------------------------------
 * PipelineHandlerIPU7
 * -------------------------------------------------------------------------- */

PipelineHandlerIPU7::PipelineHandlerIPU7(CameraManager *manager)
	: PipelineHandler(manager)
{
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerIPU7::generateConfiguration(Camera *camera, Span<const StreamRole> roles)
{
	IPU7CameraData *data = cameraData(camera);
	auto config = std::make_unique<IPU7CameraConfiguration>(data);

	if (roles.empty())
		return config;

	for (const StreamRole role : roles) {
		Size defaultSize;
		switch (role) {
		case StreamRole::StillCapture:
		case StreamRole::VideoRecording:
		case StreamRole::Viewfinder:
			defaultSize = Size(3840, 2160);
			break;
		default:
			LOG(IPU7, Error) << "Unsupported stream role: " << role;
			return nullptr;
		}

		std::map<PixelFormat, std::vector<SizeRange>> streamFormats;
		streamFormats[formats::NV12] = {
			{ Size(1920, 1080), Size(1920, 1080) },
			{ Size(2560, 1440), Size(2560, 1440) },
			{ Size(3840, 2160), Size(3840, 2160) },
		};

		StreamFormats formats(streamFormats);
		StreamConfiguration cfg(formats);
		cfg.size = defaultSize;
		cfg.pixelFormat = formats::NV12;
		cfg.bufferCount = IPU7CameraConfiguration::kBufferCount;
		config->addConfiguration(cfg);
	}

	if (config->validate() == CameraConfiguration::Invalid)
		return {};

	return config;
}

int PipelineHandlerIPU7::configure(Camera *camera, CameraConfiguration *c)
{
	IPU7CameraData *data = cameraData(camera);
	StreamConfiguration &cfg = (*c)[0];

	/*
	 * Configure the sensor for the raw Bayer output that PSYS expects.
	 * The OV08X40 outputs SGRBG10 3856×2176 natively.
	 */
	V4L2SubdeviceFormat sensorFormat{};
	sensorFormat.code = MEDIA_BUS_FMT_SGRBG10_1X10;
	sensorFormat.size = data->sensor_->resolution();

	LOG(IPU7, Debug) << "configure: setting sensor format "
			 << sensorFormat.size << " SGRBG10";

	int ret = data->sensor_->setFormat(&sensorFormat);
	if (ret) {
		LOG(IPU7, Error) << "Failed to set sensor format: " << strerror(-ret);
		return ret;
	}

	LOG(IPU7, Debug) << "configure: sensor format set to "
			 << sensorFormat.size << " code=0x"
			 << std::hex << sensorFormat.code << std::dec;

	/* Configure the CSI-2 receiver to match */
	V4L2SubdeviceFormat csi2Format = sensorFormat;
	ret = data->csi2_->setFormat(0, &csi2Format);
	if (ret) {
		LOG(IPU7, Error) << "Failed to set CSI-2 format: " << strerror(-ret);
		return ret;
	}

	LOG(IPU7, Debug) << "configure: CSI-2 format set to "
			 << csi2Format.size;

	/* Configure ISYS capture device for raw output */
	V4L2DeviceFormat isysFormat{};
	isysFormat.fourcc = V4L2PixelFormat(V4L2_PIX_FMT_SGRBG10);
	isysFormat.size = sensorFormat.size;

	LOG(IPU7, Debug) << "configure: setting ISYS capture format "
			 << isysFormat.size << " " << isysFormat.fourcc;

	ret = data->isysCapture_->setFormat(&isysFormat);
	if (ret) {
		LOG(IPU7, Error) << "Failed to set ISYS capture format: " << strerror(-ret);
		return ret;
	}

	LOG(IPU7, Debug) << "configure: ISYS capture format set to "
			 << isysFormat.size << " " << isysFormat.fourcc;

	cfg.setStream(&data->outputStream_);
	data->outputSize_ = cfg.size;

	LOG(IPU7, Info) << "Configured: sensor " << sensorFormat.size
			<< " SGRBG10 → PSYS → " << cfg.size << " NV12";

	return 0;
}

int PipelineHandlerIPU7::exportFrameBuffers(Camera *camera, Stream *stream,
					     std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	IPU7CameraData *data = cameraData(camera);

	/*
	 * Export NV12-sized DMA-BUF buffers for the application/PipeWire.
	 * We use the DmaBufAllocator to create properly-sized NV12 buffers
	 * rather than exporting raw ISYS buffers which are the wrong format/size.
	 */
	unsigned int count = stream->configuration().bufferCount;
	const StreamConfiguration &cfg = stream->configuration();

	LOG(IPU7, Debug) << "exportFrameBuffers: count=" << count
			 << " size=" << cfg.size << " format=" << cfg.pixelFormat
			 << " frameSize=" << cfg.frameSize;

	return data->dmaBufAllocator_.exportBuffers(count,
		{ cfg.frameSize }, buffers);
}

int PipelineHandlerIPU7::start(Camera *camera,
				[[maybe_unused]] const ControlList *controls)
{
	IPU7CameraData *data = cameraData(camera);
	int ret;

	/*
	 * Enable the media link from CSI-2 to the ISYS capture device.
	 * disableLinks() was called during match(), so we must re-enable.
	 */
	if (data->isysCaptureLink_) {
		ret = data->isysCaptureLink_->setEnabled(true);
		if (ret) {
			LOG(IPU7, Error) << "Failed to enable ISYS media link: "
					 << strerror(-ret);
			return ret;
		}
		LOG(IPU7, Debug) << "Enabled ISYS capture media link";
	}

	/* Open PSYS and set up the processing graph */
	ret = data->setupPsys(psysDevNode_);
	if (ret)
		return ret;

	/* Allocate ISYS raw capture buffers */
	ret = data->isysCapture_->allocateBuffers(
		IPU7CameraConfiguration::kBufferCount, &data->isysBuffers_);
	if (ret < 0) {
		LOG(IPU7, Error) << "Failed to allocate ISYS buffers";
		return ret;
	}

	/* Queue all ISYS buffers */
	for (auto &buf : data->isysBuffers_)
		data->availableIsysBuffers_.push(buf.get());

	/* Start ISYS capture */
	ret = data->isysCapture_->streamOn();
	if (ret) {
		LOG(IPU7, Error) << "Failed to start ISYS capture";
		data->isysCapture_->releaseBuffers();
		return ret;
	}

	/* Reset frame counter — firmware expects frame IDs starting at 0 */
	data->frameCounter_ = 0;

	/* Set initial sensor exposure/gain — AE loop adjusts per-frame */
	data->ae_ = {};
	ControlList sensorCtrls(data->sensor_->controls());
	sensorCtrls.set(V4L2_CID_EXPOSURE, data->ae_.exposure);
	sensorCtrls.set(V4L2_CID_ANALOGUE_GAIN, data->ae_.analogueGain);
	sensorCtrls.set(V4L2_CID_DIGITAL_GAIN, data->ae_.digitalGain);
	data->sensor_->setControls(&sensorCtrls);

	LOG(IPU7, Info) << "Pipeline started";
	return 0;
}

void PipelineHandlerIPU7::stopDevice(Camera *camera)
{
	IPU7CameraData *data = cameraData(camera);

	/* Cancel pending requests */
	while (!data->pendingRequests_.empty()) {
		Request *request = data->pendingRequests_.front();
		data->pendingRequests_.pop();

		for (const auto &[stream, buffer] : request->buffers()) {
			buffer->_d()->cancel();
			completeBuffer(request, buffer);
		}
		completeRequest(request);
	}

	data->isysCapture_->streamOff();
	data->isysCapture_->releaseBuffers();
	data->isysBuffers_.clear();
	while (!data->availableIsysBuffers_.empty())
		data->availableIsysBuffers_.pop();

	if (data->graphOpen_) {
		data->psys_.graphClose(data->graphId_);
		data->graphOpen_ = false;
	}

	data->freeBuffers();

	LOG(IPU7, Info) << "Pipeline stopped";
}

int PipelineHandlerIPU7::queueRequestDevice(Camera *camera, Request *request)
{
	IPU7CameraData *data = cameraData(camera);

	/* Apply per-request controls */
	const ControlList &reqCtrls = request->controls();

	if (reqCtrls.contains(controls::AeEnable.id()))
		data->ae_.enabled = reqCtrls.get(controls::AeEnable).value_or(true);

	if (reqCtrls.contains(controls::Brightness.id())) {
		data->ae_.brightnessTarget =
			std::clamp(reqCtrls.get(controls::Brightness).value_or(0.45f),
				   0.0f, 1.0f);
	}

	if (!data->ae_.enabled) {
		/* Manual mode: apply explicit exposure/gain if provided */
		bool changed = false;

		if (reqCtrls.contains(controls::ExposureTime.id())) {
			data->ae_.exposure = std::clamp(
				reqCtrls.get(controls::ExposureTime).value_or(data->ae_.exposure),
				IPU7CameraData::kAeExposureMin,
				IPU7CameraData::kAeExposureMax);
			changed = true;
		}

		if (reqCtrls.contains(controls::AnalogueGain.id())) {
			float gain = reqCtrls.get(controls::AnalogueGain).value_or(6.25f);
			data->ae_.analogueGain = std::clamp(
				static_cast<int32_t>(gain * 128.0f),
				IPU7CameraData::kAeAGainMin,
				IPU7CameraData::kAeAGainMax);
			changed = true;
		}

		if (changed) {
			ControlList ctrls(data->sensor_->controls());
			ctrls.set(V4L2_CID_EXPOSURE, data->ae_.exposure);
			ctrls.set(V4L2_CID_ANALOGUE_GAIN, data->ae_.analogueGain);
			ctrls.set(V4L2_CID_DIGITAL_GAIN, data->ae_.digitalGain);
			data->sensor_->setControls(&ctrls);
		}
	}

	data->pendingRequests_.push(request);

	/* If we have a raw buffer available, start processing */
	if (!data->availableIsysBuffers_.empty()) {
		FrameBuffer *rawBuf = data->availableIsysBuffers_.front();
		data->availableIsysBuffers_.pop();
		data->isysCapture_->queueBuffer(rawBuf);
	}

	return 0;
}

bool PipelineHandlerIPU7::match(DeviceEnumerator *enumerator)
{
	LOG(IPU7, Debug) << "match: looking for Intel IPU7 ISYS media device";

	/* Look for the IPU7 ISYS media device */
	DeviceMatch isysDm("intel-ipu7");
	isysDm.add("Intel IPU7 CSI2 0");
	isysDm.add("Intel IPU7 ISYS Capture 0");

	isysMedia_ = acquireMediaDevice(enumerator, isysDm);
	if (!isysMedia_) {
		LOG(IPU7, Debug) << "match: IPU7 ISYS media device not found";
		return false;
	}

	LOG(IPU7, Info) << "match: found IPU7 ISYS media device: "
			<< isysMedia_->driver();

	/*
	 * Check for the PSYS device node. It's not a media device, so we
	 * can't use the enumerator — just check if it exists.
	 */
	psysDevNode_ = "/dev/ipu7-psys0";
	if (access(psysDevNode_.c_str(), R_OK | W_OK) < 0) {
		LOG(IPU7, Warning) << "PSYS device not accessible: " << psysDevNode_
				   << " (errno=" << errno << " " << strerror(errno) << ")"
				   << " — check permissions: chmod 666 " << psysDevNode_;
		return false;
	}

	LOG(IPU7, Info) << "match: PSYS device accessible: " << psysDevNode_;

	if (isysMedia_->disableLinks())
		return false;

	int ret = registerCameras();
	LOG(IPU7, Debug) << "match: registerCameras returned " << ret;
	return ret == 0;
}

int PipelineHandlerIPU7::registerCameras()
{
	/*
	 * Walk the CSI-2 receivers looking for connected sensors.
	 * The IPU7 has 4 CSI-2 ports (CSI2 0-3).
	 */
	unsigned int numCameras = 0;

	for (unsigned int csiIndex = 0; csiIndex < 4; csiIndex++) {
		auto data = std::make_unique<IPU7CameraData>(this);

		int ret = data->init(isysMedia_, csiIndex);
		if (ret)
			continue;

		/* Set up properties from the sensor */
		data->properties_ = data->sensor_->properties();

		/* Advertise supported controls */
		ControlInfoMap::Map ctrls;
		ctrls[&controls::Brightness] =
			ControlInfo(0.0f, 1.0f, 0.45f);
		ctrls[&controls::ExposureTime] =
			ControlInfo(static_cast<int32_t>(IPU7CameraData::kAeExposureMin),
				    static_cast<int32_t>(IPU7CameraData::kAeExposureMax),
				    static_cast<int32_t>(IPU7CameraData::kAeExposureMax));
		ctrls[&controls::AnalogueGain] =
			ControlInfo(static_cast<float>(IPU7CameraData::kAeAGainMin) / 128.0f,
				    static_cast<float>(IPU7CameraData::kAeAGainMax) / 128.0f,
				    static_cast<float>(800) / 128.0f);
		ctrls[&controls::AeEnable] =
			ControlInfo(false, true, true);
		data->controlInfo_ =
			ControlInfoMap(std::move(ctrls), controls::controls);

		/* Connect ISYS buffer completion signal */
		data->isysCapture_->bufferReady.connect(
			data.get(), &IPU7CameraData::isysBufferReady);

		/* Create the camera */
		std::set<Stream *> streams = { &data->outputStream_ };
		const std::string &cameraId = data->sensor_->id();

		std::shared_ptr<Camera> camera =
			Camera::create(std::move(data), cameraId, streams);

		registerCamera(std::move(camera));

		LOG(IPU7, Info) << "Registered camera \"" << cameraId
				<< "\" on CSI-2 port " << csiIndex;
		numCameras++;
	}

	return numCameras ? 0 : -ENODEV;
}

/* --------------------------------------------------------------------------
 * IPU7CameraData
 * -------------------------------------------------------------------------- */

int IPU7CameraData::init(std::shared_ptr<MediaDevice> media, unsigned int csiIndex)
{
	std::string csi2Name = "Intel IPU7 CSI2 " + std::to_string(csiIndex);
	LOG(IPU7, Debug) << "init: probing " << csi2Name;

	/* Find the CSI-2 subdevice */
	MediaEntity *csi2Entity = media->getEntityByName(csi2Name);
	if (!csi2Entity) {
		LOG(IPU7, Debug) << "init: " << csi2Name << " entity not found";
		return -ENODEV;
	}

	/* Find a sensor connected to this CSI-2 port */
	const std::vector<MediaPad *> &pads = csi2Entity->pads();
	if (pads.empty())
		return -ENODEV;

	MediaPad *sinkPad = pads[0]; /* pad 0 is the sink */
	const std::vector<MediaLink *> &links = sinkPad->links();

	MediaEntity *sensorEntity = nullptr;
	for (const MediaLink *link : links) {
		MediaEntity *entity = link->source()->entity();
		if (entity->function() == MEDIA_ENT_F_CAM_SENSOR) {
			sensorEntity = entity;
			break;
		}
	}

	if (!sensorEntity) {
		LOG(IPU7, Debug) << "init: no sensor connected to " << csi2Name;
		return -ENODEV;
	}

	LOG(IPU7, Info) << "init: found sensor " << sensorEntity->name()
			<< " on " << csi2Name;

	/* Create the sensor */
	sensor_ = CameraSensorFactoryBase::create(sensorEntity);
	if (!sensor_) {
		LOG(IPU7, Error) << "Failed to create sensor for "
				 << sensorEntity->name();
		return -ENODEV;
	}

	/* Open the CSI-2 subdevice */
	csi2_ = std::make_unique<V4L2Subdevice>(csi2Entity);
	int ret = csi2_->open();
	if (ret)
		return ret;

	/* Find and open the first capture device connected to this CSI-2 */
	std::string captureName = "Intel IPU7 ISYS Capture " +
				  std::to_string(csiIndex * 8); /* 8 captures per CSI port */
	MediaEntity *captureEntity = media->getEntityByName(captureName);
	if (!captureEntity) {
		LOG(IPU7, Error) << "Capture device not found: " << captureName;
		return -ENODEV;
	}

	isysCapture_ = std::make_unique<V4L2VideoDevice>(captureEntity);
	ret = isysCapture_->open();
	if (ret) {
		LOG(IPU7, Error) << "Failed to open ISYS capture device";
		return ret;
	}

	/* Find and save the media link to the capture device for enabling later */
	const std::vector<MediaPad *> &capPads = captureEntity->pads();
	if (!capPads.empty()) {
		const std::vector<MediaLink *> &capLinks = capPads[0]->links();
		if (!capLinks.empty())
			isysCaptureLink_ = capLinks[0];
	}

	LOG(IPU7, Info) << "Initialized sensor " << sensor_->id()
			<< " on " << csi2Name;

	return 0;
}

int IPU7CameraData::setupPsys(const std::string &psysDevNode)
{
	int ret;

	/*
	 * If PSYS was initialized for a different resolution, tear it down
	 * completely. The firmware retains internal state from the previous
	 * graph that causes error=2 on task requests if reused at a different
	 * output resolution.
	 */
	if (psys_.isOpen() && psysSize_ != outputSize_) {
		LOG(IPU7, Info) << "Resolution changed from " << psysSize_
				<< " to " << outputSize_ << " — reinitializing PSYS";
		if (graphOpen_) {
			psys_.graphClose(graphId_);
			graphOpen_ = false;
		}
		freeBuffers();
		psys_.close();
	}

	if (!psys_.isOpen()) {
		ret = psys_.open(psysDevNode);
		if (ret)
			return ret;
	}

	/* Re-allocate buffers if they were freed by stopDevice() */
	if (isaTermBufs_.empty()) {
		ret = allocateTerminalBuffers();
		if (ret) {
			psys_.close();
			return ret;
		}

		/* Load captured ISP tuning parameters into terminal buffers */
		loadTerminalParams();
	}

	if (!graphOpen_) {
		ret = openGraph();
		if (ret) {
			freeBuffers();
			psys_.close();
			return ret;
		}
	}

	psysSize_ = outputSize_;
	return 0;
}

int IPU7CameraData::allocateTerminalBuffers()
{
	/* Select terminal tables based on output resolution */
	bool use4k = (outputSize_.width > 1920);
	const PsysTerminal *isaTerms = use4k ? isaTerminals_4k : isaTerminals_1080p;
	size_t isaTermCount = use4k ? std::size(isaTerminals_4k) : std::size(isaTerminals_1080p);
	const PsysTerminal *ofsTerms = use4k ? ofsTerminals_4k : ofsTerminals_1080p;
	size_t ofsTermCount = use4k ? std::size(ofsTerminals_4k) : std::size(ofsTerminals_1080p);

	/*
	 * Allocate buffers for ISA node terminals.
	 *
	 * The Camera HAL uses DMA_HANDLE | NO_FLUSH (0x18) flags for parameter
	 * terminals, and DMA_HANDLE (0x10) for I/O terminals. Match this exactly
	 * to ensure the firmware handles buffers correctly.
	 */
	for (size_t i = 0; i < isaTermCount; i++) {
		const auto &term = isaTerms[i];
		TermBufSet tbs;
		tbs.termId = term.termId;

		int ret = psys_.getBuf(term.bufSize, 0, &tbs.buf);
		if (ret)
			return ret;

		/*
		 * Set flags for TASK_REQUEST. GETBUF returns with USERPTR flag,
		 * but the firmware expects DMA_HANDLE | NO_FLUSH for parameter
		 * terminals and DMA_HANDLE for I/O terminals.
		 */
		if (term.termId == 5)
			tbs.buf.flags = IPU_BUFFER_FLAG_DMA_HANDLE; /* raw input */
		else if (term.termId == 19 || term.termId == 18)
			tbs.buf.flags = IPU_BUFFER_FLAG_DMA_HANDLE; /* output */
		else
			tbs.buf.flags = IPU_BUFFER_FLAG_DMA_HANDLE |
					IPU_BUFFER_FLAG_NO_FLUSH; /* params */

		ret = psys_.mapBuf(tbs.buf.fd.get());
		if (ret)
			return ret;

		isaTermBufs_.push_back(std::move(tbs));
	}

	/*
	 * Allocate buffers for OFS node terminals.
	 *
	 * Linked terminals (ISA output → OFS input) share the same buffer.
	 * We skip allocating for OFS terms 7 and 9 here — they'll reference
	 * ISA terms 18 and 19 directly when building the task request.
	 */
	for (size_t i = 0; i < ofsTermCount; i++) {
		const auto &term = ofsTerms[i];
		/* Skip linked terminals — they share ISA buffers */
		if (term.termId == 9 || term.termId == 7)
			continue;

		TermBufSet tbs;
		tbs.termId = term.termId;

		int ret = psys_.getBuf(term.bufSize, 0, &tbs.buf);
		if (ret)
			return ret;

		if (term.termId == 14)
			tbs.buf.flags = IPU_BUFFER_FLAG_DMA_HANDLE; /* NV12 output */
		else
			tbs.buf.flags = IPU_BUFFER_FLAG_DMA_HANDLE |
					IPU_BUFFER_FLAG_NO_FLUSH; /* params */

		ret = psys_.mapBuf(tbs.buf.fd.get());
		if (ret)
			return ret;

		ofsTermBufs_.push_back(std::move(tbs));
	}

	LOG(IPU7, Debug) << "Allocated " << isaTermBufs_.size()
			 << " ISA + " << ofsTermBufs_.size()
			 << " OFS terminal buffers";
	return 0;
}

/**
 * \brief Load captured ISP tuning parameters into terminal buffers
 *
 * The ISA and OFS nodes require specific tuning parameters in their terminal
 * buffers for the firmware to process frames. These parameters are normally
 * generated by Intel's 3A library (ia_aiq) from the .aiqb calibration blob.
 *
 * We use static parameter data captured from the Camera HAL. This gives us
 * a known-good ISP configuration for the OV08X40 sensor at 3856x2176.
 */
void IPU7CameraData::loadTerminalParams()
{
	/* Helper to find and populate a terminal buffer by ID */
	auto loadParam = [](std::vector<TermBufSet> &termBufs, uint8_t termId,
			     const uint8_t *data, size_t dataSize) {
		for (auto &tbs : termBufs) {
			if (tbs.termId == termId && tbs.buf.userptr) {
				size_t copySize = std::min(dataSize,
							   static_cast<size_t>(tbs.buf.len));
				memcpy(tbs.buf.userptr, data, copySize);
				return true;
			}
		}
		return false;
	};

	/* Select param blobs based on output resolution */
	bool use4k = (outputSize_.width > 1920);

	/* ISA node parameters */
	if (use4k) {
		loadParam(isaTermBufs_, 0, ISA_TERM00_4K_DATA, sizeof(ISA_TERM00_4K_DATA));
		loadParam(isaTermBufs_, 1, ISA_TERM01_4K_DATA, sizeof(ISA_TERM01_4K_DATA));
		loadParam(isaTermBufs_, 2, ISA_TERM02_4K_DATA, sizeof(ISA_TERM02_4K_DATA));
		loadParam(isaTermBufs_, 8, ISA_TERM08_4K_DATA, sizeof(ISA_TERM08_4K_DATA));
	} else {
		loadParam(isaTermBufs_, 0, ISA_TERM00_DATA, sizeof(ISA_TERM00_DATA));
		loadParam(isaTermBufs_, 1, ISA_TERM01_DATA, sizeof(ISA_TERM01_DATA));
		loadParam(isaTermBufs_, 2, ISA_TERM02_DATA, sizeof(ISA_TERM02_DATA));
		loadParam(isaTermBufs_, 8, ISA_TERM08_DATA, sizeof(ISA_TERM08_DATA));
	}

	/* OFS node parameters */
	if (use4k) {
		loadParam(ofsTermBufs_, 0, OFS_TERM00_4K_DATA, sizeof(OFS_TERM00_4K_DATA));
		loadParam(ofsTermBufs_, 1, OFS_TERM01_4K_DATA, sizeof(OFS_TERM01_4K_DATA));
		loadParam(ofsTermBufs_, 2, OFS_TERM02_4K_DATA, sizeof(OFS_TERM02_4K_DATA));
		loadParam(ofsTermBufs_, 3, OFS_TERM03_4K_DATA, sizeof(OFS_TERM03_4K_DATA));
	} else {
		loadParam(ofsTermBufs_, 0, OFS_TERM00_DATA, sizeof(OFS_TERM00_DATA));
		loadParam(ofsTermBufs_, 1, OFS_TERM01_DATA, sizeof(OFS_TERM01_DATA));
		loadParam(ofsTermBufs_, 2, OFS_TERM02_DATA, sizeof(OFS_TERM02_DATA));
		loadParam(ofsTermBufs_, 3, OFS_TERM03_DATA, sizeof(OFS_TERM03_DATA));
	}

	LOG(IPU7, Info) << "Loaded ISP tuning parameters ("
			<< outputSize_.width << "x" << outputSize_.height << ")";
}

void IPU7CameraData::freeBuffers()
{
	for (auto &tbs : isaTermBufs_) {
		if (tbs.buf.fd.isValid())
			psys_.unmapBuf(tbs.buf.fd.get());
		if (tbs.buf.userptr)
			free(tbs.buf.userptr);
	}
	isaTermBufs_.clear();

	for (auto &tbs : ofsTermBufs_) {
		if (tbs.buf.fd.isValid())
			psys_.unmapBuf(tbs.buf.fd.get());
		if (tbs.buf.userptr)
			free(tbs.buf.userptr);
	}
	ofsTermBufs_.clear();
}

int IPU7CameraData::openGraph()
{
	/* Build node descriptors — select terminal tables by resolution */
	std::vector<PsysNode> nodes(2);

	bool use4k = (outputSize_.width > 1920);
	const PsysTerminal *isaTerms = use4k ? isaTerminals_4k : isaTerminals_1080p;
	size_t isaCount = use4k ? std::size(isaTerminals_4k) : std::size(isaTerminals_1080p);
	const PsysTerminal *ofsTerms = use4k ? ofsTerminals_4k : ofsTerminals_1080p;
	size_t ofsCount = use4k ? std::size(ofsTerminals_4k) : std::size(ofsTerminals_1080p);

	nodes[0].nodeRsrcId = 0;
	nodes[0].nodeCtxId = 0;
	nodes[0].profile = isaProfile;
	for (size_t i = 0; i < isaCount; i++)
		nodes[0].terminals.push_back({ isaTerms[i].termId, isaTerms[i].bufSize });

	nodes[1].nodeRsrcId = 1;
	nodes[1].nodeCtxId = 1;
	nodes[1].profile = ofsProfile;
	for (size_t i = 0; i < ofsCount; i++)
		nodes[1].terminals.push_back({ ofsTerms[i].termId, ofsTerms[i].bufSize });

	/* Build links */
	std::vector<PsysLink> links(std::begin(graphLinks), std::end(graphLinks));

	int ret = psys_.graphOpen(nodes, links, &graphId_);
	if (ret)
		return ret;

	graphOpen_ = true;
	return 0;
}

/**
 * \brief Handle a completed raw frame from ISYS
 *
 * When ISYS delivers a raw Bayer frame, we submit it to the PSYS
 * processing graph (ISA node first, then OFS node) and wait for
 * completion events.
 */
void IPU7CameraData::isysBufferReady(FrameBuffer *buffer)
{
#if IPU7_VERBOSE_FRAME_LOG
	LOG(IPU7, Debug) << "isysBufferReady: seq=" << buffer->metadata().sequence
			 << " status=" << static_cast<int>(buffer->metadata().status)
			 << " pending=" << pendingRequests_.size();
#endif

	if (pendingRequests_.empty()) {
		/* No request waiting — recycle the buffer */
		LOG(IPU7, Debug) << "isysBufferReady: no pending request, recycling buffer";
		availableIsysBuffers_.push(buffer);
		return;
	}

	Request *request = pendingRequests_.front();
	pendingRequests_.pop();

	if (buffer->metadata().status == FrameMetadata::FrameCancelled) {
		LOG(IPU7, Debug) << "isysBufferReady: frame cancelled";
		for (const auto &[stream, buf] : request->buffers()) {
			buf->_d()->cancel();
			pipe()->completeBuffer(request, buf);
		}
		pipe()->completeRequest(request);
		availableIsysBuffers_.push(buffer);
		return;
	}

	processRawFrame(buffer, request);
}

void IPU7CameraData::processRawFrame(FrameBuffer *rawBuffer, Request *request)
{
	uint8_t frameId = frameCounter_++;
	uint32_t reuse[2] = { 0, 0 };
	int ret;

#if IPU7_VERBOSE_FRAME_LOG
	LOG(IPU7, Debug) << "processRawFrame: frameId=" << static_cast<int>(frameId)
			 << " rawBuffer seq=" << rawBuffer->metadata().sequence;
#endif

	/*
	 * Step 1: Copy raw Bayer data from the ISYS DMA-BUF into the ISA
	 * input terminal buffer (term_id=5).
	 *
	 * The ISYS capture device gives us a DMA-BUF FD containing the raw
	 * sensor data. We mmap both it and the PSYS terminal buffer, then
	 * memcpy the data across. This is not zero-copy but is correct.
	 *
	 * \todo Use DMA-BUF import to avoid the copy — pass the ISYS FD
	 * directly as the ISA input terminal buffer.
	 */
	{
		const FrameBuffer::Plane &rawPlane = rawBuffer->planes()[0];
		unsigned int rawSize = rawPlane.length;

		/* Find the ISA input terminal buffer (term_id=5) */
		PsysBuffer *isaInputBuf = nullptr;
		for (auto &tbs : isaTermBufs_) {
			if (tbs.termId == 5) {
				isaInputBuf = &tbs.buf;
				break;
			}
		}

		if (!isaInputBuf) {
			LOG(IPU7, Error) << "ISA input terminal buffer not found";
			goto fail;
		}

		/* mmap the raw ISYS buffer to read sensor data */
		void *rawData = mmap(nullptr, rawSize, PROT_READ,
				     MAP_SHARED, rawPlane.fd.get(), 0);
		if (rawData == MAP_FAILED) {
			LOG(IPU7, Error) << "Failed to mmap raw buffer: " << strerror(errno);
			goto fail;
		}

		/*
		 * Copy raw frame data into ISA input terminal buffer.
		 * PSYS buffers are backed by userptr — we own the memory directly.
		 */
		size_t copySize = std::min(static_cast<size_t>(rawSize),
					   static_cast<size_t>(isaInputBuf->len));
		memcpy(isaInputBuf->userptr, rawData, copySize);

#if IPU7_VERBOSE_FRAME_LOG
		LOG(IPU7, Debug) << "Copied " << copySize << " bytes raw data"
				 << " (ISYS=" << rawSize << " ISA=" << isaInputBuf->len << ")";
#endif

		munmap(rawData, rawSize);
	}

	/*
	 * Step 2: Submit ISA node task request with all terminal buffers.
	 */
	{
		std::vector<std::pair<uint8_t, PsysBuffer *>> isaTermBufPairs;
		for (auto &tbs : isaTermBufs_)
			isaTermBufPairs.push_back({ tbs.termId, &tbs.buf });

		ret = psys_.taskRequest(graphId_, 0, frameId, reuse, isaTermBufPairs);
		if (ret) {
			LOG(IPU7, Error) << "ISA task request failed: " << strerror(-ret);
			goto fail;
		}
	}

	/*
	 * Step 3: Submit OFS node task request.
	 *
	 * OFS terms 7 and 9 are linked from ISA terms 18 and 19 — they
	 * share the same physical buffers so the firmware can pass data
	 * from ISA to OFS.
	 */
	{
		std::vector<std::pair<uint8_t, PsysBuffer *>> ofsTermBufPairs;
		for (auto &tbs : ofsTermBufs_)
			ofsTermBufPairs.push_back({ tbs.termId, &tbs.buf });

		/* Add shared ISA buffers for linked terminals */
		for (auto &isaTbs : isaTermBufs_) {
			if (isaTbs.termId == 19)
				ofsTermBufPairs.push_back({ 9, &isaTbs.buf }); /* ISA image → OFS */
			else if (isaTbs.termId == 18)
				ofsTermBufPairs.push_back({ 7, &isaTbs.buf }); /* ISA stats → OFS */
		}

		ret = psys_.taskRequest(graphId_, 1, frameId, reuse, ofsTermBufPairs);
		if (ret) {
			LOG(IPU7, Error) << "OFS task request failed: " << strerror(-ret);
			goto fail;
		}
	}

	/*
	 * Step 4: Wait for both nodes to complete.
	 *
	 * \todo This is synchronous and blocks the pipeline. Replace with
	 * event-driven completion using poll() on the PSYS fd and integrate
	 * with libcamera's event loop.
	 */
	for (int i = 0; i < 2; i++) {
		uint8_t gid, nid, fid;
		uint32_t error;
		ret = psys_.dqEvent(&gid, &nid, &fid, &error);
		if (ret || error) {
			LOG(IPU7, Error) << "PSYS event error: ret=" << ret
					 << " error=" << error
					 << " (waiting for event " << i << "/2)";
			goto fail;
		}
#if IPU7_VERBOSE_FRAME_LOG
		LOG(IPU7, Debug) << "PSYS event " << i << ": graph="
				 << static_cast<int>(gid) << " node="
				 << static_cast<int>(nid) << " frame="
				 << static_cast<int>(fid) << " error="
				 << error;
#endif
	}

#if IPU7_VERBOSE_FRAME_LOG
	LOG(IPU7, Debug) << "Frame " << static_cast<int>(frameId)
			 << " processing complete";
#endif

	/*
	 * Step 5: Copy the processed NV12 frame from OFS terminal 14 into the
	 * application's output buffer.
	 */
	{
		/* Find the OFS output terminal buffer (term_id=14) */
		PsysBuffer *ofsOutputBuf = nullptr;
		for (auto &tbs : ofsTermBufs_) {
			if (tbs.termId == 14) {
				ofsOutputBuf = &tbs.buf;
				break;
			}
		}

		if (!ofsOutputBuf) {
			LOG(IPU7, Error) << "OFS output terminal buffer not found";
			goto fail;
		}

		/*
		 * Run simple auto-exposure on the processed frame.
		 * Uses OFS output (NV12 Y plane) for luminance feedback.
		 */
		updateAutoExposure(ofsOutputBuf->userptr,
				   outputSize_.width, outputSize_.height);

		/*
		 * OFS output buffer is backed by userptr — read directly.
		 * Copy NV12 data into each application output buffer.
		 */
		for (const auto &[stream, buf] : request->buffers()) {
			const FrameBuffer::Plane &outPlane = buf->planes()[0];
			void *outData = mmap(nullptr, outPlane.length,
					     PROT_READ | PROT_WRITE,
					     MAP_SHARED, outPlane.fd.get(), 0);
			if (outData == MAP_FAILED) {
				LOG(IPU7, Error) << "Failed to mmap output buffer: "
						 << strerror(errno);
				continue;
			}

			size_t copySize = std::min(static_cast<size_t>(ofsOutputBuf->len),
						   static_cast<size_t>(outPlane.length));

			/* Begin DMA-BUF write — ensures cache coherency */
			struct dma_buf_sync syncStart = { DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE };
			ioctl(outPlane.fd.get(), DMA_BUF_IOCTL_SYNC, &syncStart);

			memcpy(outData, ofsOutputBuf->userptr, copySize);

			/* End DMA-BUF write — flush to consumer */
			struct dma_buf_sync syncEnd = { DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE };
			ioctl(outPlane.fd.get(), DMA_BUF_IOCTL_SYNC, &syncEnd);

#if IPU7_VERBOSE_FRAME_LOG
			LOG(IPU7, Debug) << "Copied " << copySize
					 << " bytes NV12 to output buffer";
#endif

			munmap(outData, outPlane.length);

			buf->_d()->metadata().status = FrameMetadata::FrameSuccess;
			buf->_d()->metadata().sequence = frameId;
			buf->_d()->metadata().timestamp = rawBuffer->metadata().timestamp;
			if (!buf->_d()->metadata().planes().empty())
				buf->_d()->metadata().planes()[0].bytesused = copySize;
			pipe()->completeBuffer(request, buf);
		}
	}

	request->_d()->metadata().set(controls::SensorTimestamp,
				      rawBuffer->metadata().timestamp);
	request->_d()->metadata().set(controls::AeEnable, ae_.enabled);
	request->_d()->metadata().set(controls::Brightness, ae_.brightnessTarget);
	request->_d()->metadata().set(controls::ExposureTime, ae_.exposure);
	request->_d()->metadata().set(controls::AnalogueGain,
				      static_cast<float>(ae_.analogueGain) / 128.0f);

	pipe()->completeRequest(request);
	availableIsysBuffers_.push(rawBuffer);
	return;

fail:
	for (const auto &[stream, buf] : request->buffers()) {
		buf->_d()->cancel();
		pipe()->completeBuffer(request, buf);
	}
	pipe()->completeRequest(request);
	availableIsysBuffers_.push(rawBuffer);
}

/**
 * \brief Simple auto-exposure: sample NV12 luma, adjust sensor controls
 *
 * Samples a grid of pixels from the Y plane, computes average luminance,
 * and adjusts exposure → analogue gain → digital gain in that priority
 * order to reach the target brightness. Runs every kSkipFrames frames
 * to let sensor changes take effect.
 */
void IPU7CameraData::updateAutoExposure(const void *nv12Data,
					 unsigned int width,
					 unsigned int height)
{
	if (!ae_.enabled)
		return;

	if (++ae_.framesSinceUpdate < kAeSkipFrames)
		return;
	ae_.framesSinceUpdate = 0;

	/* Sample a 32×32 grid of Y pixels for average luminance */
	const uint8_t *yPlane = static_cast<const uint8_t *>(nv12Data);
	unsigned int stepX = width / 32;
	unsigned int stepY = height / 32;
	uint64_t sum = 0;
	unsigned int count = 0;

	for (unsigned int y = 0; y < height; y += stepY) {
		for (unsigned int x = 0; x < width; x += stepX) {
			sum += yPlane[y * width + x];
			count++;
		}
	}

	uint8_t avgLuma = static_cast<uint8_t>(sum / count);
	uint8_t targetLuma = static_cast<uint8_t>(ae_.brightnessTarget * 255.0f);
	int32_t error = static_cast<int32_t>(targetLuma) - avgLuma;

	/* Dead zone: don't adjust if within ±5 of target */
	if (std::abs(error) < 5)
		return;

	/*
	 * Proportional control: compute a scale factor.
	 * Positive error = too dark → increase exposure.
	 * Negative error = too bright → decrease exposure.
	 *
	 * Use a ratio-based approach: multiply total "exposure product"
	 * by (target / current) but dampen to avoid oscillation.
	 */
	float ratio = static_cast<float>(targetLuma) / std::max(avgLuma, uint8_t(1));
	/* Dampen: move only 40% of the way toward the target */
	ratio = 1.0f + (ratio - 1.0f) * 0.4f;
	/* Clamp ratio to avoid wild swings */
	ratio = std::max(0.5f, std::min(ratio, 2.0f));

	/*
	 * Adjust exposure first (best quality), then analogue gain,
	 * then digital gain (worst quality, most noise).
	 */
	float newExposure = ae_.exposure * ratio;
	newExposure = std::max(static_cast<float>(kAeExposureMin),
			       std::min(newExposure, static_cast<float>(kAeExposureMax)));
	ae_.exposure = static_cast<int32_t>(newExposure);

	/* If exposure is saturated, spill into analogue gain */
	if (ae_.exposure >= kAeExposureMax && error > 0) {
		float gainRatio = ratio;
		float newAGain = ae_.analogueGain * gainRatio;
		newAGain = std::max(static_cast<float>(kAeAGainMin),
				    std::min(newAGain, static_cast<float>(kAeAGainMax)));
		ae_.analogueGain = static_cast<int32_t>(newAGain);

		/* If analogue gain also saturated, use digital gain */
		if (ae_.analogueGain >= kAeAGainMax && error > 0) {
			float newDGain = ae_.digitalGain * gainRatio;
			newDGain = std::max(static_cast<float>(kAeDGainMin),
					    std::min(newDGain, static_cast<float>(kAeDGainMax)));
			ae_.digitalGain = static_cast<int32_t>(newDGain);
		}
	}
	/* If too bright, reduce in reverse order */
	if (ae_.exposure <= kAeExposureMin && error < 0) {
		float newAGain = ae_.analogueGain * ratio;
		newAGain = std::max(static_cast<float>(kAeAGainMin),
				    std::min(newAGain, static_cast<float>(kAeAGainMax)));
		ae_.analogueGain = static_cast<int32_t>(newAGain);
	}

	/* Apply to sensor */
	ControlList ctrls(sensor_->controls());
	ctrls.set(V4L2_CID_EXPOSURE, ae_.exposure);
	ctrls.set(V4L2_CID_ANALOGUE_GAIN, ae_.analogueGain);
	ctrls.set(V4L2_CID_DIGITAL_GAIN, ae_.digitalGain);
	sensor_->setControls(&ctrls);

#if IPU7_VERBOSE_FRAME_LOG
	LOG(IPU7, Debug) << "AE: luma=" << static_cast<int>(avgLuma)
			 << " target=" << static_cast<int>(targetLuma)
			 << " exp=" << ae_.exposure
			 << " again=" << ae_.analogueGain
			 << " dgain=" << ae_.digitalGain;
#endif
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerIPU7, "ipu7")

} /* namespace libcamera */
