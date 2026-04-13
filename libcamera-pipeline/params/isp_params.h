/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * ISP parameter data captured from Intel Camera HAL via ioctl intercept.
 *
 * These are the ISP tuning parameters that the Camera HAL populates from
 * the .aiqb calibration file (OV08X40_KAFE799_PTL.aiqb). They configure
 * the ISA and OFS processing nodes for basic demosaic + output formatting.
 *
 * Terminal data that changes per-frame (ISA term 8 = AE/AWB tuning) is
 * included as initial/default values from frame 0.
 */

#pragma once

#include <cstdint>

/* ISA node parameter terminal data */
#include "isa_term00.h"  /* term  0: 28288 bytes — ISA config */
#include "isa_term01.h"  /* term  1:   896 bytes — ISA config */
#include "isa_term02.h"  /* term  2:  2688 bytes — ISA config */
#include "isa_term08.h"  /* term  8: 17920 bytes — AE/AWB tuning (per-frame, use frame 0 default) */

/* OFS node parameter terminal data */
#include "ofs_term00.h"  /* term  0:  2176 bytes — OFS config */
#include "ofs_term01.h"  /* term  1:    64 bytes — OFS config */
#include "ofs_term02.h"  /* term  2:  1728 bytes — OFS config */
#include "ofs_term03.h"  /* term  3:  2624 bytes — OFS config */

/* 4K (3840x2160) params — captured from Intel Camera HAL via ioctl intercept */
#include "isa_term00_4k.h"
#include "isa_term01_4k.h"
#include "isa_term02_4k.h"
#include "isa_term08_4k.h"
#include "ofs_term00_4k.h"
#include "ofs_term01_4k.h"
#include "ofs_term02_4k.h"
#include "ofs_term03_4k.h"
