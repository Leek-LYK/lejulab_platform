#include "stark_wrapper.h"
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>

/**
 * @brief 全局 StarkWrapper 实例
 */
static StarkWrapper g_stark_wrapper = {0};

/**
 * @brief 动态库句柄
 */
static void* g_dl_handle = NULL;

/**
 * @brief 检查文件是否存在
 * @param file_path 文件路径
 * @return int 1表示存在，0表示不存在
 */
static int file_exists(const char* file_path) {
    if (!file_path) {
        return 0;
    }

    struct stat st;
    return (stat(file_path, &st) == 0) && S_ISREG(st.st_mode);
}

/**
 * @brief 预设的SO文件搜索路径列表
 */
static const char* DEFAULT_SO_PATHS[] = {
    "/usr/local/lib/libstark.so",
    "/home/lab/kuavo-ros-opensource/installed/lib/libstark.so",
    NULL  // 结束标记
};

/**
 * @brief 动态生成用户本地库路径
 * @return char* 返回用户本地库路径，需要调用者释放内存
 */
static char* get_user_local_lib_path() {
    const char* home = getenv("HOME");
    if (!home) {
        return NULL;
    }

    char* user_lib_path = malloc(strlen(home) + strlen("/.local/lib/libstark.so") + 1);
    if (!user_lib_path) {
        return NULL;
    }

    strcpy(user_lib_path, home);
    strcat(user_lib_path, "/.local/lib/libstark.so");
    return user_lib_path;
}

/**
 * @brief 在预设路径中查找SO文件
 * @param filename 文件名
 * @return char* 找到的完整路径，需要调用者释放内存，未找到返回NULL
 */
static char* find_in_default_paths(const char* filename) {
    if (!filename) {
        return NULL;
    }

    // 如果是完整路径，跳过预设路径搜索
    if (strchr(filename, '/') != NULL) {
        return NULL;
    }

    printf("\033[33m[StarkWrapper] Searching in default paths...\033[0m\n");

    // 检查预设的静态路径
    for (int i = 0; DEFAULT_SO_PATHS[i] != NULL; i++) {
        printf("\033[36m[StarkWrapper] Checking: %s\033[0m\n", DEFAULT_SO_PATHS[i]);

        if (file_exists(DEFAULT_SO_PATHS[i])) {
            printf("\033[32m[StarkWrapper] Found SO in default paths: %s\033[0m\n", DEFAULT_SO_PATHS[i]);
            return strdup(DEFAULT_SO_PATHS[i]);
        }
    }

    // 检查用户本地库路径
    char* user_local_path = get_user_local_lib_path();
    if (user_local_path) {
        printf("\033[36m[StarkWrapper] Checking user local path: %s\033[0m\n", user_local_path);

        if (file_exists(user_local_path)) {
            printf("\033[32m[StarkWrapper] Found SO in user local path: %s\033[0m\n", user_local_path);
            return user_local_path;  // 直接返回，不需要strdup
        }

        free(user_local_path);  // 如果没找到，释放内存
    }

    printf("\033[33m[StarkWrapper] No SO found in default paths\033[0m\n");
    return NULL;
}

/**
 * @brief 在LD_LIBRARY_PATH中查找so文件
 * @param filename 文件名
 * @return char* 找到的完整路径，需要调用者释放内存，未找到返回NULL
 */
static char* find_in_ld_library_path(const char* filename) {
    if (!filename) {
        return NULL;
    }

    const char* ld_path = getenv("LD_LIBRARY_PATH");
    if (!ld_path) {
        return NULL;
    }

    // 复制环境变量字符串，因为strtok会修改原字符串
    char* ld_path_copy = strdup(ld_path);
    if (!ld_path_copy) {
        return NULL;
    }

    char* path = strtok(ld_path_copy, ":");
    while (path != NULL) {
        char full_path[512];
        snprintf(full_path, sizeof(full_path), "%s/%s", path, filename);

        if (file_exists(full_path)) {
            printf("\033[32m[StarkWrapper] Found SO in LD_LIBRARY_PATH: %s\033[0m\n", full_path);
            free(ld_path_copy);
            return strdup(full_path);
        }

        path = strtok(NULL, ":");
    }

    free(ld_path_copy);
    return NULL;
}

/**
 * @brief 从动态库中获取函数指针
 * @param handle 动态库句柄
 * @param func_name 函数名
 * @return void* 函数指针，失败返回NULL
 */
static void* get_function(void* handle, const char* func_name) {
    void* func = dlsym(handle, func_name);
    if (!func) {
        return NULL;
    }
    return func;
}

/**
 * @brief 初始化 StarkWrapper 结构体，从动态库中加载所有函数指针
 * @param handle 动态库句柄
 * @return int 0表示成功，-1表示失败
 */
static int init_stark_wrapper_from_lib(void* handle) {
    // SDK 初始化和配置接口
    g_stark_wrapper.get_sdk_version = (const char* (*)(void))get_function(handle, "stark_get_sdk_version");
    if (!g_stark_wrapper.get_sdk_version) return -1;

    g_stark_wrapper.set_log_level = (void (*)(StarkLogLevel))get_function(handle, "stark_set_log_level");
    if (!g_stark_wrapper.set_log_level) return -1;

    g_stark_wrapper.set_log_callback = (void (*)(StarkLogCB))get_function(handle, "stark_set_log_callback");
    if (!g_stark_wrapper.set_log_callback) return -1;

    g_stark_wrapper.set_write_data_callback = (void (*)(WriteDataCB))get_function(handle, "stark_set_write_data_callback");
    if (!g_stark_wrapper.set_write_data_callback) return -1;

    // 设备管理接口
    g_stark_wrapper.create_serial_device = (StarkDevice* (*)(const char*, int))get_function(handle, "stark_create_serial_device");
    if (!g_stark_wrapper.create_serial_device) return -1;

    // 设备配置接口
    g_stark_wrapper.set_error_callback = (int (*)(StarkDevice*, StarkErrorCB))get_function(handle, "stark_set_error_callback");
    if (!g_stark_wrapper.set_error_callback) return -1;

    // 数据通信接口
    g_stark_wrapper.did_receive_data = (int (*)(StarkDevice*, const uint8_t*, int))get_function(handle, "stark_did_receive_data");
    if (!g_stark_wrapper.did_receive_data) return -1;

    // 设备信息获取接口
    g_stark_wrapper.get_motorboard_info = (void (*)(StarkDevice*, MotorboardInfoCB))get_function(handle, "stark_get_motorboard_info");
    if (!g_stark_wrapper.get_motorboard_info) return -1;

    // 手指状态和设置接口
    g_stark_wrapper.get_finger_status = (void (*)(StarkDevice*, FingerStatusCB))get_function(handle, "stark_get_finger_status");
    if (!g_stark_wrapper.get_finger_status) return -1;

    g_stark_wrapper.set_finger_positions = (void (*)(StarkDevice*, const int*))get_function(handle, "stark_set_finger_positions");
    if (!g_stark_wrapper.set_finger_positions) return -1;

    g_stark_wrapper.set_finger_speeds = (void (*)(StarkDevice*, const int*))get_function(handle, "stark_set_finger_speeds");
    if (!g_stark_wrapper.set_finger_speeds) return -1;

    // 握力等级接口
    g_stark_wrapper.get_force_level = (void (*)(StarkDevice*, StarkIntValueCB))get_function(handle, "stark_get_force_level");
    if (!g_stark_wrapper.get_force_level) return -1;

    g_stark_wrapper.set_force_level = (void (*)(StarkDevice*, int))get_function(handle, "stark_set_force_level");
    if (!g_stark_wrapper.set_force_level) return -1;

    // 涡轮模式接口
    g_stark_wrapper.get_turbo_mode = (void (*)(StarkDevice*, StarkIntValueCB))get_function(handle, "stark_get_turbo_mode");
    if (!g_stark_wrapper.get_turbo_mode) return -1;

    g_stark_wrapper.set_turbo_mode = (void (*)(StarkDevice*, int))get_function(handle, "stark_set_turbo_mode");
    if (!g_stark_wrapper.set_turbo_mode) return -1;

    g_stark_wrapper.get_turbo_conf = (void (*)(StarkDevice*, StarkValue2CB))get_function(handle, "stark_get_turbo_conf");
    if (!g_stark_wrapper.get_turbo_conf) return -1;

    // 动作序列接口
    g_stark_wrapper.run_action_sequence = (void (*)(StarkDevice*, int))get_function(handle, "stark_run_action_sequence");
    if (!g_stark_wrapper.run_action_sequence) return -1;

    g_stark_wrapper.transfer_action_sequence = (void (*)(StarkDevice*, int, int, uint16_t**))get_function(handle, "stark_transfer_action_sequence");
    if (!g_stark_wrapper.transfer_action_sequence) return -1;

    return 0;
}

StarkWrapper* stark_wrapper_init(const char* so_path) {
    printf("\033[36m[StarkWrapper] Starting initialization with path: %s\033[0m\n", so_path ? so_path : "NULL");

    // 检查参数
    if (!so_path) {
        printf("\033[31m[StarkWrapper] Error: SO path is NULL\033[0m\n");
        return NULL;
    }

    char* actual_path = NULL;
    const char* path_to_load = so_path;

    // 检查文件是否存在，如果不存在则尝试多个搜索策略
    if (!file_exists(so_path)) {
        printf("\033[33m[StarkWrapper] File not found at: %s\033[0m\n", so_path);

        // 如果输入路径不包含'/'，则认为是文件名，尝试多种搜索方式
        if (strchr(so_path, '/') == NULL) {
            printf("\033[33m[StarkWrapper] Searching for filename using multiple strategies\033[0m\n");

            // 1. 首先尝试预设路径
            actual_path = find_in_default_paths(so_path);
            if (actual_path) {
                path_to_load = actual_path;
                printf("\033[32m[StarkWrapper] Found file in default paths: %s\033[0m\n", path_to_load);
            } else {
                // 2. 然后尝试LD_LIBRARY_PATH
                printf("\033[33m[StarkWrapper] Now searching in LD_LIBRARY_PATH\033[0m\n");
                actual_path = find_in_ld_library_path(so_path);
                if (!actual_path) {
                    printf("\033[31m[StarkWrapper] Error: File not found in any search locations\033[0m\n");
                    return NULL;
                }
                path_to_load = actual_path;
                printf("\033[32m[StarkWrapper] Found file in LD_LIBRARY_PATH: %s\033[0m\n", path_to_load);
            }
        } else {
            // 如果是完整路径但不存在，直接返回失败
            printf("\033[31m[StarkWrapper] Error: File not found at path: %s\033[0m\n", so_path);
            return NULL;
        }
    } else {
        printf("\033[32m[StarkWrapper] Found file at: %s\033[0m\n", so_path);
    }

    // 检查文件是否可读
    if (access(path_to_load, R_OK) != 0) {
        printf("\033[31m[StarkWrapper] Error: No read permission for file: %s\033[0m\n", path_to_load);
        if (actual_path) {
            free(actual_path);
        }
        return NULL;
    }

    printf("\033[32m[StarkWrapper] File permissions OK, proceeding to load\033[0m\n");

    // 加载动态库
    printf("\033[32m[StarkWrapper] Loading SO from: %s\033[0m\n", path_to_load);
    g_dl_handle = dlopen(path_to_load, RTLD_LAZY | RTLD_LOCAL);
    if (!g_dl_handle) {
        printf("\033[31m[StarkWrapper] Failed to load SO: %s\033[0m\n", path_to_load);
        if (actual_path) {
            free(actual_path);
        }
        return NULL;
    }
    printf("\033[32m[StarkWrapper] Successfully loaded SO: %s\033[0m\n", path_to_load);

    // 初始化函数指针
    printf("\033[36m[StarkWrapper] Initializing function pointers...\033[0m\n");
    if (init_stark_wrapper_from_lib(g_dl_handle) != 0) {
        printf("\033[31m[StarkWrapper] Error: Failed to initialize function pointers\033[0m\n");
        dlclose(g_dl_handle);
        g_dl_handle = NULL;
        if (actual_path) {
            free(actual_path);
        }
        return NULL;
    }
    printf("\033[32m[StarkWrapper] Function pointers initialized successfully\033[0m\n");

    // 释放分配的内存
    if (actual_path) {
        free(actual_path);
    }

    printf("\033[32m[StarkWrapper] StarkWrapper initialization completed successfully\033[0m\n");
    return &g_stark_wrapper;
}

void stark_wrapper_cleanup(StarkWrapper* wrapper) {
    // 清理动态库
    if (g_dl_handle) {
        dlclose(g_dl_handle);
        g_dl_handle = NULL;
    }

    // 重置函数指针
    if (wrapper != NULL) {
        memset(wrapper, 0, sizeof(StarkWrapper));
    }
}