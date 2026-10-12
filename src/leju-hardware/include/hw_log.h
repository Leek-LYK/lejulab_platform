#pragma once

#include "lejusdk-utils/leju_log.h"

#define HW_TAG "HW"
#define HWLOG_I(...)    LOG_I(HW_TAG, __VA_ARGS__)
#define HWLOG_D(...)    LOG_D(HW_TAG, __VA_ARGS__)  
#define HWLOG_E(...)    LOG_E(HW_TAG, __VA_ARGS__)
#define HWLOG_T(...)    LOG_T(HW_TAG, __VA_ARGS__)
#define HWLOG_W(...)    LOG_W(HW_TAG, __VA_ARGS__)
#define HWLOG_SUCCESS(format, ...)  LOG_SUCCESS(HW_TAG, format, ##__VA_ARGS__)
#define HWLOG_WARNING(format, ...)  LOG_WARNING(HW_TAG, format, ##__VA_ARGS__)
#define HWLOG_FAILURE(format, ...)  LOG_FAILURE(HW_TAG, format, ##__VA_ARGS__)