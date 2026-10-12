#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "ruiwoSDK.h"
/**
 * @brief open can bus and initialize the BM library
 * @param ruiwo RUIWOTools struct
 * @param init_bmlib 1: initialize the BM library, 0: do not initialize the BM library, avoid multiple initialization.
 */

#define PRINT_RECEIVE_DATA 0
#define PRINT_SEND_DATA 0
int open_canbus(RUIWOTools* ruiwo, int init_bmlib, unsigned char *version) {
    BM_StatusTypeDef error;
    BM_BitrateTypeDef bitrate;
    
    memset(&bitrate, 0, sizeof(bitrate));       
    bitrate.nbitrate = ruiwo->dev_info.bit_rate;
    bitrate.dbitrate = ruiwo->dev_info.data_rate;
    bitrate.nsamplepos = 75;
    bitrate.dsamplepos = 75;
    
    /* Initialize the BM library  only once .*/
    if (init_bmlib == 1) {
        error = BM_Init();
        if (error != BM_ERROR_OK) {
            char buffer[256] = { 0 };
            BM_GetErrorText(error, buffer, sizeof(buffer), 0);
            printf("BM_Init error: %s\n", buffer);
            return -1;
        }
    }
    BM_SetLogLevel(4);
    BM_ChannelInfoTypeDef channelinfos[32];
    int nchannels = sizeof(channelinfos) / sizeof(channelinfos[0]);
    error = BM_Enumerate(channelinfos, &nchannels);
    if (error != BM_ERROR_OK) {
        char buffer[256] = { 0 };
        BM_GetErrorText(error, buffer, sizeof(buffer), 0);
        printf("BM_Enumerate error: %s\n", buffer);
        BM_UnInit();
        return -2;
    }

    // 打印可用通道数
    printf("Available channels: %d\n", nchannels);

    // 打印设备信息
    int target_channel = -1;
    for (int i = 0; i < nchannels; i++) {
        printf("Device %d Information:\n", i + 1);
        printf("  Name: %s\n", channelinfos[i].name);
        printf("  Serial Number: ");
        for (int j = 0; j < sizeof(channelinfos[i].sn); j++) {
            printf("%02X", channelinfos[i].sn[j]);
        }
        printf("\n");

        printf("  UID: ");
        for (int j = 0; j < sizeof(channelinfos[i].uid); j++) {
            printf("%02X", channelinfos[i].uid[j]);
        }
        printf("\n");

        printf("  Version: ");
        for (int j = 0; j < sizeof(channelinfos[i].version); j++) {
            printf("%02X", channelinfos[i].version[j]);
        }
        printf("\n");

        printf("  VID: 0x%04X\n", channelinfos[i].vid);
        printf("  PID: 0x%04X\n", channelinfos[i].pid);
        printf("  Port ID: %d\n", channelinfos[i].port);
        printf("  Capabilities: 0x%04X\n", channelinfos[i].cap);

        if (channelinfos[i].version[0] == version[0] &&
            channelinfos[i].version[1] == version[1] &&
            channelinfos[i].version[2] == version[2] &&
            channelinfos[i].version[3] == version[3]) {
            target_channel = i;
            break;
        }
    }

    if (target_channel == -1) {
        BM_UnInit();
        return -5; // No matching device found
    }

    error = BM_OpenEx(&ruiwo->channel, &channelinfos[target_channel], BM_CAN_NORMAL_MODE, BM_TRESISTOR_120, &bitrate, NULL, 0);
    if (error != BM_ERROR_OK) {
        char buffer[256] = { 0 };
        BM_GetErrorText(error, buffer, sizeof(buffer), 0);
        printf("BM_OpenEx error: %s\n", buffer);
        BM_UnInit();
        return -3;
    }

    error = BM_GetNotification(ruiwo->channel, &ruiwo->notification);
    if (error != BM_ERROR_OK) {
        BM_Close(ruiwo->channel);
        BM_UnInit();
        return -4;
    }

    return 0; // Success
}

// int open_canbus(RUIWOTools* ruiwo) {
//     BM_StatusTypeDef error;
//     BM_BitrateTypeDef bitrate;
    
//     memset(&bitrate, 0, sizeof(bitrate));       
//     bitrate.nbitrate = ruiwo->dev_info.bit_rate;
//     bitrate.dbitrate = ruiwo->dev_info.data_rate;
//     bitrate.nsamplepos = 75;
//     bitrate.dsamplepos = 75;

//     error = BM_Init();
//     if (error != BM_ERROR_OK) {
//         return -1;
//     }

//     BM_ChannelInfoTypeDef channelinfos[32];
//     int nchannels = sizeof(channelinfos) / sizeof(channelinfos[0]);
//     error = BM_Enumerate(channelinfos, &nchannels);
//     if (error != BM_ERROR_OK) {
//         BM_UnInit();
//         return -2;
//     }

//     // 打印可用通道数
//     printf("Available channels: %d\n", nchannels);

//     // 打印设备信息
//     for (int i = 0; i < nchannels; i++) {
//         printf("Device %d Information:\n", i + 1);
//         printf("  Name: %s\n", channelinfos[i].name);
//         printf("  Serial Number: ");
//         for (int j = 0; j < sizeof(channelinfos[i].sn); j++) {
//             printf("%02X", channelinfos[i].sn[j]);
//         }
//         printf("\n");

//         printf("  UID: ");
//         for (int j = 0; j < sizeof(channelinfos[i].uid); j++) {
//             printf("%02X", channelinfos[i].uid[j]);
//         }
//         printf("\n");

//         printf("  Version: ");
//         for (int j = 0; j < sizeof(channelinfos[i].version); j++) {
//             printf("%02X", channelinfos[i].version[j]);
//         }
//         printf("\n");

//         printf("  VID: 0x%04X\n", channelinfos[i].vid);
//         printf("  PID: 0x%04X\n", channelinfos[i].pid);
//         printf("  Port ID: %d\n", channelinfos[i].port);
//         printf("  Capabilities: 0x%04X\n", channelinfos[i].cap);
//     }

//     if (ruiwo->dev_info.dev_channel <= nchannels) {
//         error = BM_OpenEx(&ruiwo->channel, &channelinfos[ruiwo->dev_info.dev_channel], BM_CAN_NORMAL_MODE, BM_TRESISTOR_120, &bitrate, NULL, 0);
//         if (error != BM_ERROR_OK) {
//             BM_UnInit();
//             return -3;
//         }

//         error = BM_GetNotification(ruiwo->channel, &ruiwo->notification);
//         if (error != BM_ERROR_OK) {
//             BM_Close(ruiwo->channel);
//             BM_UnInit();
//             return -4;
//         }

//         return 0; // Success
//     } else {
//         BM_UnInit();
//         return -5; // Invalid channel
//     }
// }

// int open_canbus(RUIWOTools* ruiwo) {
//     BM_StatusTypeDef error;
//     BM_BitrateTypeDef bitrate;
    
//     memset(&bitrate, 0, sizeof(bitrate));       
//     bitrate.nbitrate = ruiwo->dev_info.bit_rate;
//     bitrate.dbitrate = ruiwo->dev_info.data_rate;
//     bitrate.nsamplepos = 75;
//     bitrate.dsamplepos = 75;

//     error = BM_Init();
//     if (error != BM_ERROR_OK) {
//         return -1;
//     }

//     BM_ChannelInfoTypeDef channelinfos[32];
//     int nchannels = sizeof(channelinfos) / sizeof(channelinfos[0]);
//     error = BM_Enumerate(channelinfos, &nchannels);
//     if (error != BM_ERROR_OK) {
//         BM_UnInit();
//         return -2;
//     }

// // 打印可用通道数
//     printf("Available channels: %d\n", nchannels);
//     if (ruiwo->dev_info.dev_channel <= nchannels) {
//         error = BM_OpenEx(&ruiwo->channel, &channelinfos[ruiwo->dev_info.dev_channel], BM_CAN_NORMAL_MODE, BM_TRESISTOR_120, &bitrate, NULL, 0);
//         if (error != BM_ERROR_OK) {
//             BM_UnInit();
//             return -3;
//         }

//         error = BM_GetNotification(ruiwo->channel, &ruiwo->notification);
//         if (error != BM_ERROR_OK) {
//             BM_Close(ruiwo->channel);
//             BM_UnInit();
//             return -4;
//         }

//         return 0; // Success
//     } else {
//         BM_UnInit();
//         return -5; // Invalid channel
//     }
// }

int close_canbus(RUIWOTools* ruiwo) {
    
    // BM_CanStatusInfoTypedef statusinfo;
    // BM_StatusTypeDef error2 = BM_GetStatus(ruiwo->channel, &statusinfo);
    // printf("wawawawawawawawa: %d\n", error2);
    BM_StatusTypeDef error = BM_Close(ruiwo->channel);
    if (error != BM_ERROR_OK) {
        char buffer[256] = { 0 };
        BM_GetErrorText(error, buffer, sizeof(buffer), 0);
        printf("BM_Close error: %s\n", buffer);
        return -1;
    }

    BM_UnInit();
    return 0;
}

uint32_t float_to_int(float float_num, float min_num, float max_num, uint8_t bit_num) {
    return (uint32_t)((float_num - min_num) / (max_num - min_num) * ((1 << bit_num) - 1));
}

float int_to_float(uint32_t int_num, float min_num, float max_num, uint8_t bit_num) {
    return ((float)int_num / ((1 << bit_num) - 1)) * (max_num - min_num) + min_num;
}

void return_motor_state(BM_CanMessageTypeDef* feedback_frame, float* state_list, uint8_t* errcode, CanComParam* params) {
#if PRINT_RECEIVE_DATA
    printf("Received CAN ID: %u\n", feedback_frame->id.SID);
#endif
    state_list[0] = feedback_frame->id.SID;

    uint32_t pos = (feedback_frame->payload[1] << 8) | feedback_frame->payload[2];
    state_list[1] = int_to_float(pos, params->CAN_COM_THETA_MIN, params->CAN_COM_THETA_MAX, 16);
#if PRINT_MOTOR_DATA
    printf("Position (rad): %.2f\n", state_list[1]);
#endif

    uint32_t vel = (feedback_frame->payload[3] << 4) | (feedback_frame->payload[4] >> 4);
    state_list[2] = int_to_float(vel, params->CAN_COM_VELOCITY_MIN, params->CAN_COM_VELOCITY_MAX, 12);
#if PRINT_MOTOR_DATA
    printf("Velocity (rad/s): %.2f\n", state_list[2]);
#endif

    uint32_t torque = ((feedback_frame->payload[4] & 0x0F) << 8) | feedback_frame->payload[5];
    state_list[3] = int_to_float(torque, params->CAN_COM_TORQUE_MIN, params->CAN_COM_TORQUE_MAX, 12);
#if PRINT_MOTOR_DATA
    printf("Torque (N.m): %.2f\n", state_list[3]);
#endif

    state_list[4] = feedback_frame->payload[7]; // Temperature
#if PRINT_MOTOR_DATA
    printf("Temperature: %d°C\n", feedback_frame->payload[7]);
#endif

    state_list[5] = feedback_frame->payload[6]; // Error code
    if (errcode != NULL) {
        *errcode = feedback_frame->payload[6];
    }
#if PRINT_MOTOR_DATA
    printf("Error Code: %d\n", feedback_frame->payload[6]);
#endif
}

int enter_motor_state(RUIWOTools* ruiwo, uint32_t dev_id, float* state_list, uint8_t* errcode) {
    BM_CanMessageTypeDef msg;
    memset(&msg, 0, sizeof(msg));
    msg.id.SID = dev_id;
    msg.ctrl.tx.DLC = 8;
    msg.payload[0] = 0xFF;
    msg.payload[1] = 0xFF;
    msg.payload[2] = 0xFF;
    msg.payload[3] = 0xFF;
    msg.payload[4] = 0xFF;
    msg.payload[5] = 0xFF;
    msg.payload[6] = 0xFF;
    msg.payload[7] = 0xFC;

#if PRINT_SEND_DATA
    printf("Sending message to device ID %u:\n", dev_id);
    for (int i = 0; i < 8; i++) {
        printf("payload[%d]: %02X\n", i, msg.payload[i]);
    }
#endif

    BM_StatusTypeDef error = BM_WriteCanMessage(ruiwo->channel, &msg, 0, ruiwo->dev_info.timeout, NULL);
    if (error != BM_ERROR_OK) {
#if PRINT_SEND_DATA
        printf("Failed to send message, error code: 0x%x\n", error);
#endif
        char buffer[256] = { 0 };
        BM_GetErrorText(error, buffer, sizeof(buffer), 0);
        printf("BM_WriteCanMessage errcode:0x%x, error: %s\n", error, buffer);
        return -1;
    }

    BM_CanMessageTypeDef rx_msg;
    uint32_t port;
    uint32_t timestamp;
    // Wait for notifications with a timeout
    if (BM_WaitForNotifications(&ruiwo->notification, 1, TEST_MSG_RX_TIMEOUT) < 0) {
#if PRINT_RECEIVE_DATA
        printf("Receive timeout.\n");
#endif
        return -2; // Timeout error code
    }
    error = BM_ReadCanMessage(ruiwo->channel, &rx_msg, &port, &timestamp);
    if (error != BM_ERROR_OK) {
#if PRINT_RECEIVE_DATA
        printf("Failed to receive message, error code: %d\n", error);
#endif
        char buffer[256] = { 0 };
        BM_GetErrorText(error, buffer, sizeof(buffer), 0);
        printf("BM_ReadCanMessage errcode:0x%x, error: %s\n", error, buffer);
        return -2;
    }

#if PRINT_RECEIVE_DATA
    printf("Received message from device ID %u:\n", rx_msg.id.SID);
    printf("Payload received:\n");
    for (int i = 0; i < 8; i++) {
        printf("payload[%d]: %02X\n", i, rx_msg.payload[i]);
    }
#endif

    if (rx_msg.id.SID == dev_id) {
        return_motor_state(&rx_msg, state_list, errcode, &ruiwo->can_com_param);
        return 0; // Success
    } else {
#if PRINT_RECEIVE_DATA
        printf("Device ID mismatch, expected %u, got %u\n", dev_id, rx_msg.id.SID);
#endif
        return -3; // ID mismatch
    }
}

int enter_reset_state(RUIWOTools* ruiwo, uint32_t dev_id, float* state_list, uint8_t* errcode) {
    BM_CanMessageTypeDef msg;
    memset(&msg, 0, sizeof(msg));
    msg.id.SID = dev_id;
    msg.ctrl.tx.DLC = 8;
    msg.payload[0] = 0xFF;
    msg.payload[1] = 0xFF;
    msg.payload[2] = 0xFF;
    msg.payload[3] = 0xFF;
    msg.payload[4] = 0xFF;
    msg.payload[5] = 0xFF;
    msg.payload[6] = 0xFF;
    msg.payload[7] = 0xFD;

    BM_StatusTypeDef error = BM_WriteCanMessage(ruiwo->channel, &msg, 0, ruiwo->dev_info.timeout, NULL);
    if (error != BM_ERROR_OK) {
        char buffer[256] = { 0 };
        BM_GetErrorText(error, buffer, sizeof(buffer), 0);
        printf("enter_reset_state errcode:0x%x, error: %s\n", error, buffer);
        return -1;
    }

    BM_CanMessageTypeDef rx_msg;
    uint32_t port;
    uint32_t timestamp;
    // Wait for notifications with a timeout
    if (BM_WaitForNotifications(&ruiwo->notification, 1, TEST_MSG_RX_TIMEOUT) < 0) {
#if PRINT_RECEIVE_DATA
        printf("Receive timeout.\n");
#endif
        return -2; // Timeout error code
    }
    error = BM_ReadCanMessage(ruiwo->channel, &rx_msg, &port, &timestamp);
    if (error != BM_ERROR_OK) {
#if PRINT_RECEIVE_DATA
        printf("Failed to receive message, error code: %d\n", error);
#endif
        char buffer[256] = { 0 };
        BM_GetErrorText(error, buffer, sizeof(buffer), 0);
        printf("enter_reset_state BM_ReadCanMessage errcode:0x%x, error: %s\n", error, buffer);
        return -3;
    }

#if PRINT_RECEIVE_DATA
    printf("Received message from device ID %u:\n", rx_msg.id.SID);
    printf("Payload received:\n");
    for (int i = 0; i < 8; i++) {
        printf("payload[%d]: %02X\n", i, rx_msg.payload[i]);
    }
#endif

    if (rx_msg.id.SID == dev_id) {
        return_motor_state(&rx_msg, state_list, errcode, &ruiwo->can_com_param);
        return 0; // Success
    } else {
#if PRINT_RECEIVE_DATA
        printf("Device ID mismatch, expected %u, got %u\n", dev_id, rx_msg.id.SID);
#endif
        return -4; // ID mismatch
    }
}

int set_zero_position(RUIWOTools* ruiwo, uint32_t dev_id, float* state_list) {
    BM_CanMessageTypeDef msg;
    memset(&msg, 0, sizeof(msg));
    msg.id.SID = dev_id;
    msg.ctrl.tx.DLC = 8;
    msg.payload[0] = 0xFF;
    msg.payload[1] = 0xFF;
    msg.payload[2] = 0xFF;
    msg.payload[3] = 0xFF;
    msg.payload[4] = 0xFF;
    msg.payload[5] = 0xFF;
    msg.payload[6] = 0xFF;
    msg.payload[7] = 0xFE;

    BM_StatusTypeDef error = BM_WriteCanMessage(ruiwo->channel, &msg, 0, ruiwo->dev_info.timeout, NULL);
    if (error != BM_ERROR_OK) {
        return -1;
    }

    BM_CanMessageTypeDef rx_msg;
    uint32_t port;
    uint32_t timestamp;
    // Wait for notifications with a timeout
    if (BM_WaitForNotifications(&ruiwo->notification, 1, TEST_MSG_RX_TIMEOUT) < 0) {
#if PRINT_RECEIVE_DATA
        printf("Receive timeout.\n");
#endif
        return -2; // Timeout error code
    }
    error = BM_ReadCanMessage(ruiwo->channel, &rx_msg, &port, &timestamp);
    if (error != BM_ERROR_OK) {
#if PRINT_RECEIVE_DATA
        printf("Failed to receive message, error code: %d\n", error);
#endif
        return -2;
    }

#if PRINT_RECEIVE_DATA
    printf("Received message from device ID %u:\n", rx_msg.id.SID);
    printf("Payload received:\n");
    for (int i = 0; i < 8; i++) {
        printf("payload[%d]: %02X\n", i, rx_msg.payload[i]);
    }
#endif

    if (rx_msg.id.SID == dev_id) {
        uint8_t errcode; // unused
        return_motor_state(&rx_msg, state_list, &errcode, &ruiwo->can_com_param);
        return 0; // Success
    } else {
#if PRINT_RECEIVE_DATA
        printf("Device ID mismatch, expected %u, got %u\n", dev_id, rx_msg.id.SID);
#endif
        return -3; // ID mismatch
    }
}

int run_servo_mode(RUIWOTools* ruiwo, uint32_t dev_id, float pos, float vel, float pos_kp, float pos_kd, float vel_kp, float vel_kd, float vel_ki, float* state_list) {
    BM_CanMessageTypeDef msg;
    memset(&msg, 0, sizeof(msg));
    msg.id.SID = dev_id;
    msg.ctrl.tx.DLC = 8;

    uint16_t pos_int = float_to_int(pos, ruiwo->can_com_param.CAN_COM_THETA_MIN, ruiwo->can_com_param.CAN_COM_THETA_MAX, 16);
    uint8_t vel_int = float_to_int(vel, ruiwo->can_com_param.CAN_COM_VELOCITY_MIN, ruiwo->can_com_param.CAN_COM_VELOCITY_MAX, 8);
    uint8_t pos_kp_int = float_to_int(pos_kp, ruiwo->can_com_param.CAN_COM_POS_KP_MIN, ruiwo->can_com_param.CAN_COM_POS_KP_MAX, 8);
    uint8_t pos_kd_int = float_to_int(pos_kd, ruiwo->can_com_param.CAN_COM_POS_KD_MIN, ruiwo->can_com_param.CAN_COM_POS_KD_MAX, 8);
    uint8_t vel_kp_int = float_to_int(vel_kp, ruiwo->can_com_param.CAN_COM_VEL_KP_MIN, ruiwo->can_com_param.CAN_COM_VEL_KP_MAX, 8);
    uint8_t vel_kd_int = float_to_int(vel_kd, ruiwo->can_com_param.CAN_COM_VEL_KD_MIN, ruiwo->can_com_param.CAN_COM_VEL_KD_MAX, 8);
    uint8_t vel_ki_int = float_to_int(vel_ki, ruiwo->can_com_param.CAN_COM_VEL_KI_MIN, ruiwo->can_com_param.CAN_COM_VEL_KI_MAX, 8);

    msg.payload[0] = (pos_int >> 8) & 0xFF;
    msg.payload[1] = pos_int & 0xFF;
    msg.payload[2] = vel_int;
    msg.payload[3] = pos_kp_int;
    msg.payload[4] = pos_kd_int;
    msg.payload[5] = vel_kp_int;
    msg.payload[6] = vel_kd_int;
    msg.payload[7] = vel_ki_int;

    BM_StatusTypeDef error = BM_WriteCanMessage(ruiwo->channel, &msg, 0, ruiwo->dev_info.timeout, NULL);
    if (error != BM_ERROR_OK) {
        return -1;
    }

    BM_CanMessageTypeDef rx_msg;
    uint32_t port;
    uint32_t timestamp;
    // Wait for notifications with a timeout
    if (BM_WaitForNotifications(&ruiwo->notification, 1, TEST_MSG_RX_TIMEOUT) < 0) {
#if PRINT_RECEIVE_DATA
        printf("Receive timeout.\n");
#endif
        return -2; // Timeout error code
    }
    error = BM_ReadCanMessage(ruiwo->channel, &rx_msg, &port, &timestamp);
    if (error != BM_ERROR_OK) {
#if PRINT_RECEIVE_DATA
        printf("Failed to receive message, error code: %d\n", error);
#endif
        return -2;
    }

#if PRINT_RECEIVE_DATA
    printf("Received message from device ID %u:\n", rx_msg.id.SID);
    printf("Payload received:\n");
    for (int i = 0; i < 8; i++) {
        printf("payload[%d]: %02X\n", i, rx_msg.payload[i]);
    }
#endif

    if (rx_msg.id.SID == dev_id) {
        uint8_t errcode; // unused
        return_motor_state(&rx_msg, state_list, &errcode, &ruiwo->can_com_param);
        return 0; // Success
    } else {
#if PRINT_RECEIVE_DATA
        printf("Device ID mismatch, expected %u, got %u\n", dev_id, rx_msg.id.SID);
#endif
        return -3; // ID mismatch
    }
}

int run_ptm_mode_No_response(RUIWOTools* ruiwo, uint32_t dev_id, float pos, float vel, float pos_kp, float pos_kd, float torque, float* state_list) {
    BM_CanMessageTypeDef msg;
    memset(&msg, 0, sizeof(msg));
    msg.id.SID = dev_id;
    msg.ctrl.tx.DLC = 8;

    uint16_t pos_int = float_to_int(pos, ruiwo->can_com_param.CAN_COM_THETA_MIN, ruiwo->can_com_param.CAN_COM_THETA_MAX, 16);
    uint16_t vel_int = float_to_int(vel, ruiwo->can_com_param.CAN_COM_VELOCITY_MIN, ruiwo->can_com_param.CAN_COM_VELOCITY_MAX, 12);
    uint16_t pos_kp_int = float_to_int(pos_kp, ruiwo->can_com_param.CAN_COM_POS_KP_MIN, ruiwo->can_com_param.CAN_COM_POS_KP_MAX, 12);
    uint16_t pos_kd_int = float_to_int(pos_kd, ruiwo->can_com_param.CAN_COM_POS_KD_MIN, ruiwo->can_com_param.CAN_COM_POS_KD_MAX, 12);
    uint16_t torque_int = float_to_int(torque, ruiwo->can_com_param.CAN_COM_TORQUE_MIN, ruiwo->can_com_param.CAN_COM_TORQUE_MAX, 12);

    msg.payload[0] = (pos_int >> 8) & 0xFF;
    msg.payload[1] = pos_int & 0xFF;
    msg.payload[2] = (vel_int >> 4) & 0xFF;
    msg.payload[3] = ((vel_int & 0x0F) << 4) | ((pos_kp_int >> 8) & 0x0F);
    msg.payload[4] = pos_kp_int & 0xFF;
    msg.payload[5] = (pos_kd_int >> 4) & 0xFF;
    msg.payload[6] = ((pos_kd_int & 0x0F) << 4) | ((torque_int >> 8) & 0x0F);
    msg.payload[7] = torque_int & 0xFF;

    BM_StatusTypeDef error = BM_WriteCanMessage(ruiwo->channel, &msg, 0, 0, NULL);
    if (error != BM_ERROR_OK) {
        char buffer[256] = { 0 };
        BM_GetErrorText(error, buffer, sizeof(buffer), 0);
        printf("run_ptm_mode_No_response BM_WriteCanMessage errcode:0x%x, error: %s\n", error, buffer);
        return -1;
    }
}


int run_ptm_mode(RUIWOTools* ruiwo, uint32_t dev_id, float pos, float vel, float pos_kp, float pos_kd, float torque, float* state_list) {
    BM_CanMessageTypeDef msg;
    memset(&msg, 0, sizeof(msg));
    msg.id.SID = dev_id;
    msg.ctrl.tx.DLC = 8;

    uint16_t pos_int = float_to_int(pos, ruiwo->can_com_param.CAN_COM_THETA_MIN, ruiwo->can_com_param.CAN_COM_THETA_MAX, 16);
    uint16_t vel_int = float_to_int(vel, ruiwo->can_com_param.CAN_COM_VELOCITY_MIN, ruiwo->can_com_param.CAN_COM_VELOCITY_MAX, 12);
    uint16_t pos_kp_int = float_to_int(pos_kp, ruiwo->can_com_param.CAN_COM_POS_KP_MIN, ruiwo->can_com_param.CAN_COM_POS_KP_MAX, 12);
    uint16_t pos_kd_int = float_to_int(pos_kd, ruiwo->can_com_param.CAN_COM_POS_KD_MIN, ruiwo->can_com_param.CAN_COM_POS_KD_MAX, 12);
    uint16_t torque_int = float_to_int(torque, ruiwo->can_com_param.CAN_COM_TORQUE_MIN, ruiwo->can_com_param.CAN_COM_TORQUE_MAX, 12);

    msg.payload[0] = (pos_int >> 8) & 0xFF;
    msg.payload[1] = pos_int & 0xFF;
    msg.payload[2] = (vel_int >> 4) & 0xFF;
    msg.payload[3] = ((vel_int & 0x0F) << 4) | ((pos_kp_int >> 8) & 0x0F);
    msg.payload[4] = pos_kp_int & 0xFF;
    msg.payload[5] = (pos_kd_int >> 4) & 0xFF;
    msg.payload[6] = ((pos_kd_int & 0x0F) << 4) | ((torque_int >> 8) & 0x0F);
    msg.payload[7] = torque_int & 0xFF;

    BM_StatusTypeDef error = BM_WriteCanMessage(ruiwo->channel, &msg, 0, ruiwo->dev_info.timeout, NULL);
    if (error != BM_ERROR_OK) {
        return -1;
    }

    BM_CanMessageTypeDef rx_msg;
    uint32_t port;
    uint32_t timestamp;
    // Wait for notifications with a timeout
    if (BM_WaitForNotifications(&ruiwo->notification, 1, TEST_MSG_RX_TIMEOUT) < 0) {
#if PRINT_RECEIVE_DATA
        printf("Receive timeout.\n");
#endif
        return -2; // Timeout error code
    }
    error = BM_ReadCanMessage(ruiwo->channel, &rx_msg, &port, &timestamp);
    if (error != BM_ERROR_OK) {
#if PRINT_RECEIVE_DATA
        printf("Failed to receive message, error code: %d\n", error);
#endif
        return -2;
    }

#if PRINT_RECEIVE_DATA
    printf("Received message from device ID %u:\n", rx_msg.id.SID);
    printf("Payload received:\n");
    for (int i = 0; i < 8; i++) {
        printf("payload[%d]: %02X\n", i, rx_msg.payload[i]);
    }
#endif

    if (rx_msg.id.SID == dev_id) {
        uint8_t errcode; // unused
        return_motor_state(&rx_msg, state_list, &errcode, &ruiwo->can_com_param);
        return 0; // Success
    } else {
#if PRINT_RECEIVE_DATA
        printf("Device ID mismatch, expected %u, got %u\n", dev_id, rx_msg.id.SID);
#endif
        return -3; // ID mismatch
    }
}

int run_vel_mode(RUIWOTools* ruiwo, uint32_t dev_id, float vel, float vel_kp, float vel_kd, float vel_ki, float* state_list) {
    BM_CanMessageTypeDef msg;
    memset(&msg, 0, sizeof(msg));
    msg.id.SID = dev_id;
    msg.ctrl.tx.DLC = 8;

    uint16_t vel_int = float_to_int(vel, ruiwo->can_com_param.CAN_COM_VELOCITY_MIN, ruiwo->can_com_param.CAN_COM_VELOCITY_MAX, 16);
    uint16_t vel_kp_int = float_to_int(vel_kp, ruiwo->can_com_param.CAN_COM_VEL_KP_MIN, ruiwo->can_com_param.CAN_COM_VEL_KP_MAX, 12);
    uint16_t vel_kd_int = float_to_int(vel_kd, ruiwo->can_com_param.CAN_COM_VEL_KD_MIN, ruiwo->can_com_param.CAN_COM_VEL_KD_MAX, 12);
    uint16_t vel_ki_int = float_to_int(vel_ki, ruiwo->can_com_param.CAN_COM_VEL_KI_MIN, ruiwo->can_com_param.CAN_COM_VEL_KI_MAX, 12);

    msg.payload[0] = (vel_int >> 8) & 0xFF;
    msg.payload[1] = vel_int & 0xFF;
    msg.payload[2] = (vel_kp_int >> 4) & 0xFF;
    msg.payload[3] = ((vel_kp_int & 0x0F) << 4) | ((vel_kd_int >> 8) & 0x0F);
    msg.payload[4] = vel_kd_int & 0xFF;
    msg.payload[5] = (vel_ki_int >> 4) & 0xFF;
    msg.payload[6] = (vel_ki_int & 0x0F) << 4;
    msg.payload[7] = 0xAC;

    BM_StatusTypeDef error = BM_WriteCanMessage(ruiwo->channel, &msg, 0, ruiwo->dev_info.timeout, NULL);
    if (error != BM_ERROR_OK) {
        return -1;
    }

    BM_CanMessageTypeDef rx_msg;
    uint32_t port;
    uint32_t timestamp;
    // Wait for notifications with a timeout
    if (BM_WaitForNotifications(&ruiwo->notification, 1, TEST_MSG_RX_TIMEOUT) < 0) {
#if PRINT_RECEIVE_DATA
        printf("Receive timeout.\n");
#endif
        return -2; // Timeout error code
    }
    error = BM_ReadCanMessage(ruiwo->channel, &rx_msg, &port, &timestamp);
    if (error != BM_ERROR_OK) {
#if PRINT_RECEIVE_DATA
        printf("Failed to receive message, error code: %d\n", error);
#endif
        return -2;
    }

#if PRINT_RECEIVE_DATA
    printf("Received message from device ID %u:\n", rx_msg.id.SID);
    printf("Payload received:\n");
    for (int i = 0; i < 8; i++) {
        printf("payload[%d]: %02X\n", i, rx_msg.payload[i]);
    }
#endif

    if (rx_msg.id.SID == dev_id) {
        uint8_t errcode; // unused
        return_motor_state(&rx_msg, state_list, &errcode, &ruiwo->can_com_param);
        return 0; // Success
    } else {
#if PRINT_RECEIVE_DATA
        printf("Device ID mismatch, expected %u, got %u\n", dev_id, rx_msg.id.SID);
#endif
        return -3; // ID mismatch
    }
}

int run_torque_mode(RUIWOTools* ruiwo, uint32_t dev_id, float torque, float* state_list) {
    BM_CanMessageTypeDef msg;
    memset(&msg, 0, sizeof(msg));
    msg.id.SID = dev_id;
    msg.ctrl.tx.DLC = 8;

    uint16_t torque_int = float_to_int(torque, ruiwo->can_com_param.CAN_COM_TORQUE_MIN, ruiwo->can_com_param.CAN_COM_TORQUE_MAX, 16);

    msg.payload[0] = (torque_int >> 8) & 0xFF;
    msg.payload[1] = torque_int & 0xFF;
    msg.payload[2] = 0x00;
    msg.payload[3] = 0x00;
    msg.payload[4] = 0x00;
    msg.payload[5] = 0x00;
    msg.payload[6] = 0x00;
    msg.payload[7] = 0xAB;

    BM_StatusTypeDef error = BM_WriteCanMessage(ruiwo->channel, &msg, 0, ruiwo->dev_info.timeout, NULL);
    if (error != BM_ERROR_OK) {
        return -1;
    }

    BM_CanMessageTypeDef rx_msg;
    uint32_t port;
    uint32_t timestamp;
    // Wait for notifications with a timeout
    if (BM_WaitForNotifications(&ruiwo->notification, 1, TEST_MSG_RX_TIMEOUT) < 0) {
#if PRINT_RECEIVE_DATA
        printf("Receive timeout.\n");
#endif
        return -2; // Timeout error code
    }
    error = BM_ReadCanMessage(ruiwo->channel, &rx_msg, &port, &timestamp);
    if (error != BM_ERROR_OK) {
#if PRINT_RECEIVE_DATA
        printf("Failed to receive message, error code: %d\n", error);
#endif
        return -2;
    }

#if PRINT_RECEIVE_DATA
    printf("Received message from device ID %u:\n", rx_msg.id.SID);
    printf("Payload received:\n");
    for (int i = 0; i < 8; i++) {
        printf("payload[%d]: %02X\n", i, rx_msg.payload[i]);
    }
#endif

    if (rx_msg.id.SID == dev_id) {
        uint8_t errcode; // unused
        return_motor_state(&rx_msg, state_list, &errcode, &ruiwo->can_com_param);
        return 0; // Success
    } else {
#if PRINT_RECEIVE_DATA
        printf("Device ID mismatch, expected %u, got %u\n", dev_id, rx_msg.id.SID);
#endif
        return -3; // ID mismatch
    }
}

void initialize_ruiwoSDK(RUIWOTools *ruiwo, bool is_claw) {
    strcpy(ruiwo->dev_info.dev_type, "type");
    ruiwo->dev_info.dev_channel = 0;
    ruiwo->dev_info.bit_rate = 1000;
    ruiwo->dev_info.data_rate = 2000;
    ruiwo->dev_info.terminal_res = 1;
    ruiwo->dev_info.timeout = 1000;

    if (is_claw) {
        ruiwo->can_com_param.CAN_COM_THETA_MIN = -100.0;
        ruiwo->can_com_param.CAN_COM_THETA_MAX = 100.0;
    } else {
        ruiwo->can_com_param.CAN_COM_THETA_MIN = -12.5;
        ruiwo->can_com_param.CAN_COM_THETA_MAX = 12.5;
    }
    ruiwo->can_com_param.CAN_COM_VELOCITY_MIN = -10.0;
    ruiwo->can_com_param.CAN_COM_VELOCITY_MAX = 10.0;
    ruiwo->can_com_param.CAN_COM_TORQUE_MIN = -50.0;
    ruiwo->can_com_param.CAN_COM_TORQUE_MAX = 50.0;
    ruiwo->can_com_param.CAN_COM_POS_KP_MIN = 0.0;
    ruiwo->can_com_param.CAN_COM_POS_KP_MAX = 250.0;
    ruiwo->can_com_param.CAN_COM_POS_KD_MIN = 0.0;
    ruiwo->can_com_param.CAN_COM_POS_KD_MAX = 50.0;
    ruiwo->can_com_param.CAN_COM_VEL_KP_MIN = 0.0;
    ruiwo->can_com_param.CAN_COM_VEL_KP_MAX = 250.0;
    ruiwo->can_com_param.CAN_COM_VEL_KD_MIN = 0.0;
    ruiwo->can_com_param.CAN_COM_VEL_KD_MAX = 50.0;
    ruiwo->can_com_param.CAN_COM_VEL_KI_MIN = 0.0;
    ruiwo->can_com_param.CAN_COM_VEL_KI_MAX = 0.05;
}

#ifdef __cplusplus
}
#endif