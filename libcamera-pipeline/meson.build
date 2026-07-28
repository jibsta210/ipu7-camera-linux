# SPDX-License-Identifier: CC0-1.0

# The NPU denoiser was dropped (the DnCNN model turned out to be an identity
# function and the ISP's own noise reduction is sufficient). Dropping it also
# removes a hard build dependency on OpenVINO at /opt/openvino, which nobody
# else has -- that dependency alone made this handler unbuildable elsewhere.
libcamera_internal_sources += files([
    'ipu7.cpp',
    'psys.cpp',
])
