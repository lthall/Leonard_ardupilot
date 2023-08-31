#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define DO_EXPAND(VAL)  VAL ## 1
#define EXPAND(VAL)     DO_EXPAND(VAL)

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,2,3,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 4
#define FW_MINOR 2
#define FW_PATCH 3
#define FW_VER_COUNT 1
#define FW_TYPE_BASE FIRMWARE_VERSION_TYPE_OFFICIAL

#if (!defined(FW_VER_COUNT)) || (EXPAND(FW_VER_COUNT) == 1)
#define FW_TYPE FW_TYPE_BASE
#else  // if (!defined(FW_VER_COUNT)) || (EXPAND(FW_VER_COUNT) == 1)
#define FW_TYPE (FW_TYPE_BASE + FW_VER_COUNT - 1)
#endif  // if (!defined(FW_VER_COUNT)) || (EXPAND(FW_VER_COUNT) == 1)


#ifdef FTS

#define FTS_MW_MAJOR 3
#define FTS_MW_MINOR 4
#define MW_NAME "FTS"
#define MW_MAJOR FTS_MW_MAJOR
#define MW_MINOR FTS_MW_MINOR
#define MW_PATCH 0
#define MW_VER_COUNT 1
#define MW_TYPE_BASE FIRMWARE_VERSION_TYPE_OFFICIAL

#if (!defined(MW_VER_COUNT)) || (EXPAND(MW_VER_COUNT) == 1)
#define MW_TYPE MW_TYPE_BASE
#else  // (!defined(MW_VER_COUNT)) || (EXPAND(MW_VER_COUNT) == 1)
#define MW_TYPE (MW_TYPE_BASE + MW_VER_COUNT - 1)
#endif  // (!defined(MW_VER_COUNT)) || (EXPAND(MW_VER_COUNT) == 1)

#else  // FTS

#define FLYHAWK_MW_MAJOR 3
#define FLYHAWK_MW_MINOR 9
#define MW_NAME "Flyhawk"
#define MW_MAJOR FLYHAWK_MW_MAJOR
#define MW_MINOR FLYHAWK_MW_MINOR
#define MW_PATCH 0
#define MW_VER_COUNT 1
#define MW_TYPE_BASE FIRMWARE_VERSION_TYPE_OFFICIAL

#if (!defined(MW_VER_COUNT)) || (EXPAND(MW_VER_COUNT) == 1)
#define MW_TYPE MW_TYPE_BASE
#else  // (!defined(MW_VER_COUNT)) || (EXPAND(MW_VER_COUNT) == 1)
#define MW_TYPE (MW_TYPE_BASE + MW_VER_COUNT - 1)
#endif  // (!defined(MW_VER_COUNT)) || (EXPAND(MW_VER_COUNT) == 1)

#endif  // FTS

#define THISFIRMWARE MW_NAME " V" STR(MW_MAJOR) "." STR(MW_MINOR) "." STR(MW_PATCH) " (ArduCopter V" STR(FW_MAJOR) "." STR(FW_MINOR) "." STR(FW_PATCH) ")"

#include <AP_Common/AP_FWVersionDefine.h>
