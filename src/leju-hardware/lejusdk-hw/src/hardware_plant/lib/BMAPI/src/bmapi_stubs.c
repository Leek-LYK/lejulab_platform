/**
 * @file  bmapi_stubs.c
 * @brief Stub implementations of BMAPI functions for platforms where the
 *        Busmust USB-CAN adapter library is not available (e.g. aarch64).
 *        All functions return error / no-op so the code compiles and links,
 *        but the BMAPI-based CAN path is non-functional at runtime.
 */
#include "bmapi.h"
#include <string.h>

BM_StatusTypeDef BM_Init(void) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_UnInit(void) {
    return BM_ERROR_OK;
}

BM_StatusTypeDef BM_Enumerate(BM_ChannelInfoTypeDef channelinfos[], int* nchannels) {
    if (nchannels) *nchannels = 0;
    return BM_ERROR_OK;
}

BM_StatusTypeDef BM_Autoset(
    BM_ChannelInfoTypeDef* channelinfo,
    BM_BitrateTypeDef* bitrate,
    BM_TerminalResistorTypeDef* tres,
    BM_AutosetCallbackHandle callback,
    uintptr_t userarg) {
    return BM_ERROR_INITIALIZE;
}

BM_ChannelHandle BM_OpenCan(uint16_t port) {
    return NULL;
}

BM_StatusTypeDef BM_OpenEx(
    BM_ChannelHandle* handle,
    BM_ChannelInfoTypeDef* channelinfo,
    BM_CanModeTypeDef mode,
    BM_TerminalResistorTypeDef tres,
    const BM_BitrateTypeDef* bitrate,
    const BM_RxFilterTypeDef* rxfilter,
    int nrxfilters) {
    if (handle) *handle = NULL;
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_Close(BM_ChannelHandle handle) {
    return BM_ERROR_OK;
}

BM_StatusTypeDef BM_Reset(BM_ChannelHandle handle) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_ResetDevice(BM_ChannelHandle handle) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_Activate(BM_ChannelHandle handle) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_Deactivate(BM_ChannelHandle handle) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_ClearBuffer(BM_ChannelHandle handle) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_Read(BM_ChannelHandle handle, BM_DataTypeDef* data) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_ReadMultiple(BM_ChannelHandle handle, BM_DataTypeDef data[], uint32_t* nmessages, int timeout) {
    if (nmessages) *nmessages = 0;
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_ReadIsotp(BM_ChannelHandle handle, const void* data, uint32_t* nbytes, int timeout, BM_IsotpConfigTypeDef* config) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_ReadCanMessage(BM_ChannelHandle handle, BM_CanMessageTypeDef* msg, uint32_t* channel, uint32_t* timestamp) {
    return BM_ERROR_QRCVEMPTY;
}

BM_StatusTypeDef BM_ReadMultipleCanMessage(BM_ChannelHandle handle, BM_CanMessageTypeDef msg[], uint32_t* nmessages, int timeout, uint32_t channel[], uint32_t timestamp[]) {
    if (nmessages) *nmessages = 0;
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_Write(BM_ChannelHandle handle, const BM_DataTypeDef* data, int timeout, uint32_t* timestamp) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_WriteMultiple(BM_ChannelHandle handle, const BM_DataTypeDef data[], uint32_t* nmessages, int timeout, uint32_t timestamp[]) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_WriteIsotp(BM_ChannelHandle handle, const void* data, uint32_t nbytes, int timeout, BM_IsotpConfigTypeDef* config) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_WriteCanMessage(BM_ChannelHandle handle, BM_CanMessageTypeDef* msg, uint32_t _channel, int timeout, uint32_t* timestamp) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_WriteMultipleCanMessage(BM_ChannelHandle handle, const BM_CanMessageTypeDef msg[], uint32_t* nmessages, uint32_t _channel[], int timeout, uint32_t timestamp[]) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_Control(BM_ChannelHandle handle, uint8_t command, uint16_t value, uint16_t index, void* data, int nbytes) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_GetStatus(BM_ChannelHandle handle, BM_CanStatusInfoTypedef* statusinfo) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_GetTimestamp(BM_ChannelHandle handle, uint32_t* timestamp) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_GetTxTasks(BM_ChannelHandle handle, BM_TxTaskTypeDef* txtasks, int ntxtasks) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_GetMsgRoutes(BM_ChannelHandle handle, BM_MessageRouteTypeDef* msgroutes, int nmsgroute) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_SetCanMode(BM_ChannelHandle handle, BM_CanModeTypeDef mode) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_GetCanMode(BM_ChannelHandle handle, BM_CanModeTypeDef* mode) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_SetTerminalRegister(BM_ChannelHandle handle, BM_TerminalResistorTypeDef tres) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_GetTerminalRegister(BM_ChannelHandle handle, BM_TerminalResistorTypeDef* tres) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_SetBitrate(BM_ChannelHandle handle, const BM_BitrateTypeDef* bitrate) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_GetBitrate(BM_ChannelHandle handle, BM_BitrateTypeDef* bitrate) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_SetTxTasks(BM_ChannelHandle handle, BM_TxTaskTypeDef* txtasks, int ntxtasks) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_SetMsgRoutes(BM_ChannelHandle handle, BM_MessageRouteTypeDef* msgroutes, int nmsgroute) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_SetRxFilters(BM_ChannelHandle handle, BM_RxFilterTypeDef* rxfilters, int nrxfilters) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_GetRxFilters(BM_ChannelHandle handle, BM_RxFilterTypeDef* rxfilters, int nrxfilters) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_GetNotification(BM_ChannelHandle handle, BM_NotificationHandle* notification) {
    if (notification) *notification = NULL;
    return BM_ERROR_INITIALIZE;
}

int BM_WaitForNotifications(BM_NotificationHandle handles[], int nhandles, int ntimeoutms) {
    return -1;
}

void BM_GetErrorText(BM_StatusTypeDef errorcode, char* buffer, int nbytes, uint16_t language) {
    if (buffer && nbytes > 0) {
        strncpy(buffer, "BMAPI not available on this platform", nbytes - 1);
        buffer[nbytes - 1] = '\0';
    }
}

void BM_GetDataText(BM_DataTypeDef* data, char* buffer, int nbytes, uint16_t language) {
    if (buffer && nbytes > 0) buffer[0] = '\0';
}

BM_LogLevelTypeDef BM_GetLogLevel(void) {
    return BM_LOG_NONE;
}

void BM_SetLogLevel(BM_LogLevelTypeDef level) {
}

uint32_t BM_GetVersion(void) {
    return 0;
}

BM_StatusTypeDef BM_LoadConfig(BM_ChannelHandle handle, int configmask) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_SaveConfig(BM_ChannelHandle handle, int configmask) {
    return BM_ERROR_INITIALIZE;
}

BM_StatusTypeDef BM_ClearConfig(BM_ChannelHandle handle, int configmask) {
    return BM_ERROR_INITIALIZE;
}
