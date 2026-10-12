
#ifndef _LEJUSDK_LOWLEVEL_LOG_H_
#define _LEJUSDK_LOWLEVEL_LOG_H_

#define LEJU_SDK_PRINT_LOG 1
#ifdef LEJU_SDK_PRINT_LOG

#include "lejusdk-utils/leju_log.h"

// Fixed tag macros for LJSDK
#define LJSDK_LOG_I(...) LOG_I("LJSDK", __VA_ARGS__)
#define LJSDK_LOG_D(...) LOG_D("LJSDK", __VA_ARGS__)
#define LJSDK_LOG_E(...) LOG_E("LJSDK", __VA_ARGS__)
#define LJSDK_LOG_T(...) LOG_T("LJSDK", __VA_ARGS__)
#define LJSDK_LOG_W(...) LOG_W("LJSDK", __VA_ARGS__)

// Colored log macros
#define LJSDK_LOG_SUCCESS(format, ...) LOG_SUCCESS("LJSDK", format, ##__VA_ARGS__)
#define LJSDK_LOG_WARNING(format, ...) LOG_WARNING("LJSDK", format, ##__VA_ARGS__)
#define LJSDK_LOG_FAILURE(format, ...) LOG_FAILURE("LJSDK", format, ##__VA_ARGS__)

#else

// Empty macros when logging is disabled
#define LJSDK_LOG_I(...)
#define LJSDK_LOG_D(...)
#define LJSDK_LOG_E(...)
#define LJSDK_LOG_T(...)
#define LJSDK_LOG_W(...)
#define LJSDK_LOG_SUCCESS(...)
#define LJSDK_LOG_WARNING(...)
#define LJSDK_LOG_FAILURE(...)

#endif // LEJU_SDK_PRINT_LOG

#endif // _LEJUSDK_LOWLEVEL_LOG_H_