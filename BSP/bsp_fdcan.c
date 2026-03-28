#include "bsp_fdcan.h"
#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_dwt.h"

typedef struct
{
    FDCAN_HandleTypeDef *handle;
    uint8_t started;
    uint8_t next_std_filter_idx;
} FDCAN_Bus_Context_s;

#define FDCAN_STD_ID_MAX 0x7FFU

// FDCAN实例数组,用于中断回调时查找对应的实例
static FDCAN_Instance *fdcan_instances[FDCAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局FDCAN实例索引,每次有新的模块注册会自增

static FDCAN_Instance *fdcan1_std_registry[FDCAN_STD_ID_MAX + 1U] = {NULL};
static FDCAN_Instance *fdcan2_std_registry[FDCAN_STD_ID_MAX + 1U] = {NULL};

static FDCAN_Bus_Context_s fdcan_bus_contexts[] = {
    {.handle = &hfdcan1, .started = 0, .next_std_filter_idx = 0},
    {.handle = &hfdcan2, .started = 0, .next_std_filter_idx = 0},
};

volatile FDCAN_Debug_Bus_s g_fdcan1_debug = {0};
volatile FDCAN_Debug_Bus_s g_fdcan2_debug = {0};

/* ----------------------------------- 以下为私有函数 ----------------------------------------------- */

/**
 * @brief 字节数转换为FDCAN DLC值
 *        CAN FD的DLC编码: 0-8直接对应, 9->12, 10->16, 11->20, 12->24, 13->32, 14->48, 15->64
 *
 * @param bytes 字节数
 * @return uint32_t FDCAN DLC值
 */
static uint32_t BytesToDLC(uint8_t bytes)
{
    if (bytes <= 8)
        return ((uint32_t)bytes) << 16; // DLC 0-8
    else if (bytes <= 12)
        return FDCAN_DLC_BYTES_12;
    else if (bytes <= 16)
        return FDCAN_DLC_BYTES_16;
    else if (bytes <= 20)
        return FDCAN_DLC_BYTES_20;
    else if (bytes <= 24)
        return FDCAN_DLC_BYTES_24;
    else if (bytes <= 32)
        return FDCAN_DLC_BYTES_32;
    else if (bytes <= 48)
        return FDCAN_DLC_BYTES_48;
    else
        return FDCAN_DLC_BYTES_64;
}

/**
 * @brief FDCAN DLC值转换为字节数
 *
 * @param dlc FDCAN DLC值
 * @return uint8_t 实际字节数
 */
static uint8_t DLCToBytes(uint32_t dlc)
{
    static const uint8_t dlc_to_bytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    uint8_t dlc_value = (uint8_t)((dlc >> 16) & 0x0F);
    return dlc_to_bytes[dlc_value];
}

/**
 * @brief 将原始DLC nibble转换为实际字节数
 */
static uint8_t DLCNibbleToBytes(uint32_t dlc_nibble)
{
    static const uint8_t dlc_to_bytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

    if (dlc_nibble > 15U)
        return 0U;

    return dlc_to_bytes[dlc_nibble];
}

/**
 * @brief 根据句柄获取总线上下文
 */
static FDCAN_Bus_Context_s *FDCANGetBusContext(FDCAN_HandleTypeDef *handle)
{
    size_t i;

    for (i = 0; i < (sizeof(fdcan_bus_contexts) / sizeof(fdcan_bus_contexts[0])); ++i) {
        if (fdcan_bus_contexts[i].handle == handle) {
            return &fdcan_bus_contexts[i];
        }
    }

    return NULL;
}

/**
 * @brief 根据句柄获取调试观测结构
 */
static volatile FDCAN_Debug_Bus_s *FDCANGetDebugBus(FDCAN_HandleTypeDef *handle)
{
    if (handle == &hfdcan1)
        return &g_fdcan1_debug;
    if (handle == &hfdcan2)
        return &g_fdcan2_debug;
    return NULL;
}

/**
 * @brief 根据句柄获取标准ID直达映射表
 */
static FDCAN_Instance **FDCANGetStdRegistry(FDCAN_HandleTypeDef *handle)
{
    if (handle == &hfdcan1)
        return fdcan1_std_registry;
    if (handle == &hfdcan2)
        return fdcan2_std_registry;
    return NULL;
}

/**
 * @brief 拷贝最多8字节到调试缓冲区,方便在调试器里直接观察
 */
static void FDCANDebugCopy8(volatile uint8_t *dst, const uint8_t *src, uint8_t len)
{
    uint8_t i;
    uint8_t copy_len = len > 8U ? 8U : len;

    for (i = 0; i < copy_len; ++i) {
        dst[i] = src[i];
    }
    for (; i < 8U; ++i) {
        dst[i] = 0U;
    }
}

/**
 * @brief 使用实际发送头判断当前实例是否工作在CAN FD模式
 */
static uint8_t FDCANInstanceIsCanFd(FDCAN_Instance *instance)
{
    return instance->txconf.FDFormat == FDCAN_FD_CAN;
}

/**
 * @brief 解析接收报文长度
 *
 * @note  H7 FDCAN与旧工程迁移时,DataLength字段可能表现为:
 *        1. ST宏编码值(位于16~19位)
 *        2. 原始DLC nibble(0~15)
 *        3. 少数场景下的实际字节数
 *        这里统一兼容,并对经典CAN做8字节兜底。
 */
static uint8_t FDCANResolveRxLength(FDCAN_Instance *instance, uint32_t raw_dlc)
{
    uint32_t dlc_nibble;
    uint8_t resolved_len;

    if ((raw_dlc & 0xFFFF0000U) != 0U) {
        resolved_len = DLCToBytes(raw_dlc);
        if (resolved_len != 0U || raw_dlc == 0U)
            return resolved_len;
    }

    if (raw_dlc <= 8U)
        return (uint8_t)raw_dlc;

    if (instance != NULL && !FDCANInstanceIsCanFd(instance))
        return 8U;

    dlc_nibble = raw_dlc & 0xFU;
    resolved_len = DLCNibbleToBytes(dlc_nibble);
    if (resolved_len != 0U)
        return resolved_len;

    switch (raw_dlc) {
        case 12U:
        case 16U:
        case 20U:
        case 24U:
        case 32U:
        case 48U:
        case 64U:
            return (uint8_t)raw_dlc;
        default:
            return 0U;
    }
}

/**
 * @brief 根据接收头快速找到已注册实例
 */
static FDCAN_Instance *FDCANFindRxInstance(FDCAN_HandleTypeDef *handle, const FDCAN_RxHeaderTypeDef *rxconf)
{
    FDCAN_Instance **std_registry;
    size_t i;

    if (handle == NULL || rxconf == NULL)
        return NULL;

    if (rxconf->IdType == FDCAN_STANDARD_ID && rxconf->Identifier <= FDCAN_STD_ID_MAX) {
        std_registry = FDCANGetStdRegistry(handle);
        if (std_registry != NULL && std_registry[rxconf->Identifier] != NULL)
            return std_registry[rxconf->Identifier];
    }

    for (i = 0; i < idx; ++i) {
        if (fdcan_instances[i] == NULL)
            continue;

        if (handle == fdcan_instances[i]->fdcan_handle &&
            rxconf->Identifier == fdcan_instances[i]->rx_id) {
            return fdcan_instances[i];
        }
    }

    return NULL;
}

/**
 * @brief 同步实例的工作模式镜像字段,避免调试时看到影子状态与真实发送头不一致
 */
static void FDCANSyncInstanceMode(FDCAN_Instance *instance)
{
    instance->use_canfd = FDCANInstanceIsCanFd(instance);

    if (!instance->use_canfd) {
        instance->txconf.BitRateSwitch = FDCAN_BRS_OFF;
        instance->txconf.FDFormat = FDCAN_CLASSIC_CAN;
    }

    {
        volatile FDCAN_Debug_Bus_s *debug_bus = FDCANGetDebugBus(instance->fdcan_handle);
        if (debug_bus != NULL) {
            debug_bus->last_is_canfd = instance->use_canfd;
            debug_bus->last_tx_fdformat = instance->txconf.FDFormat;
        }
    }
}

/**
 * @brief 按需启动某一条FDCAN总线
 *
 * @note  启动前统一配置全局过滤器:
 *        - 非匹配标准/扩展帧全部拒绝
 *        - 遥控帧全部拒绝
 *        这样总线只会接收显式注册过的ID
 */
static HAL_StatusTypeDef FDCANEnsureBusStarted(FDCAN_Bus_Context_s *bus_ctx)
{
    volatile FDCAN_Debug_Bus_s *debug_bus = FDCANGetDebugBus(bus_ctx == NULL ? NULL : bus_ctx->handle);

    if (bus_ctx == NULL)
        return HAL_ERROR;

    if (bus_ctx->started)
        return HAL_OK;

    if (HAL_FDCAN_ConfigGlobalFilter(bus_ctx->handle,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK) {
        if (debug_bus != NULL) {
            debug_bus->start_fail_count++;
            debug_bus->last_hal_status = HAL_ERROR;
            debug_bus->last_error_code = bus_ctx->handle->ErrorCode;
        }
        return HAL_ERROR;
    }

    if (HAL_FDCAN_Start(bus_ctx->handle) != HAL_OK) {
        if (debug_bus != NULL) {
            debug_bus->start_fail_count++;
            debug_bus->last_hal_status = HAL_ERROR;
            debug_bus->last_error_code = bus_ctx->handle->ErrorCode;
        }
        return HAL_ERROR;
    }

    if (HAL_FDCAN_ActivateNotification(bus_ctx->handle,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
                                       0) != HAL_OK) {
        (void)HAL_FDCAN_Stop(bus_ctx->handle);
        if (debug_bus != NULL) {
            debug_bus->start_fail_count++;
            debug_bus->last_hal_status = HAL_ERROR;
            debug_bus->last_error_code = bus_ctx->handle->ErrorCode;
        }
        return HAL_ERROR;
    }

    bus_ctx->started = 1;
    if (debug_bus != NULL) {
        debug_bus->started = 1U;
        debug_bus->next_std_filter_idx = bus_ctx->next_std_filter_idx;
        debug_bus->start_ok_count++;
        debug_bus->last_hal_status = HAL_OK;
        debug_bus->last_error_code = bus_ctx->handle->ErrorCode;
    }
    return HAL_OK;
}

/**
 * @brief 添加过滤器以实现对特定id的报文的接收
 *
 * @note  仅为真正需要接收的实例配置过滤器。
 *        发送专用实例(rx_id=0)不会占用过滤器。
 */
static HAL_StatusTypeDef FDCANAddFilter(FDCAN_Instance *instance, FDCAN_Bus_Context_s *bus_ctx)
{
    FDCAN_FilterTypeDef filter_config = {0};
    volatile FDCAN_Debug_Bus_s *debug_bus = FDCANGetDebugBus(instance == NULL ? NULL : instance->fdcan_handle);

    if (instance == NULL || bus_ctx == NULL)
        return HAL_ERROR;

    if (instance->rx_id == 0)
        return HAL_OK;

    if (bus_ctx->next_std_filter_idx >= instance->fdcan_handle->Init.StdFiltersNbr)
        return HAL_ERROR;

    filter_config.IdType = FDCAN_STANDARD_ID;
    filter_config.FilterIndex = bus_ctx->next_std_filter_idx;
    filter_config.FilterType = FDCAN_FILTER_MASK;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter_config.FilterID1 = instance->rx_id;
    filter_config.FilterID2 = 0x7FF; // 11位全掩码,精确匹配单个标准ID

    if (HAL_FDCAN_ConfigFilter(instance->fdcan_handle, &filter_config) != HAL_OK) {
        if (debug_bus != NULL) {
            debug_bus->filter_fail_count++;
            debug_bus->last_hal_status = HAL_ERROR;
            debug_bus->last_error_code = instance->fdcan_handle->ErrorCode;
            debug_bus->next_std_filter_idx = bus_ctx->next_std_filter_idx;
        }
        return HAL_ERROR;
    }

    bus_ctx->next_std_filter_idx++;
    if (debug_bus != NULL) {
        debug_bus->filter_ok_count++;
        debug_bus->last_hal_status = HAL_OK;
        debug_bus->last_error_code = instance->fdcan_handle->ErrorCode;
        debug_bus->next_std_filter_idx = bus_ctx->next_std_filter_idx;
    }
    return HAL_OK;
}

/* -------------------- 以下为公有函数 ---------------------- */

/**
 * @brief 注册(初始化)一个FDCAN实例
 *
 * @param config 初始化配置指针
 * @return FDCAN_Instance* FDCAN实例指针,失败返回NULL
 */
FDCAN_Instance *FDCANRegister(FDCAN_Init_Config_s *config)
{
    FDCAN_Bus_Context_s *bus_ctx;
    FDCAN_Instance *fdcan;
    FDCAN_Instance **std_registry;
    uint8_t i;

    if (config == NULL || config->fdcan_handle == NULL)
        return NULL;

    bus_ctx = FDCANGetBusContext(config->fdcan_handle);
    if (bus_ctx == NULL)
        return NULL;

    std_registry = FDCANGetStdRegistry(config->fdcan_handle);

    if (idx >= FDCAN_MX_REGISTER_CNT) {
        return NULL;
    }

    if (config->rx_id != 0U &&
        config->rx_id <= FDCAN_STD_ID_MAX &&
        std_registry != NULL &&
        std_registry[config->rx_id] != NULL) {
        return NULL;
    }

    // 检查是否重复注册（兼容非标准ID或无直达表场景）
    for (i = 0; i < idx; ++i) {
        if (config->rx_id != 0U &&
            fdcan_instances[i] != NULL &&
            fdcan_instances[i]->fdcan_handle == config->fdcan_handle &&
            fdcan_instances[i]->rx_id == config->rx_id) {
            return NULL;
        }
    }

    fdcan = (FDCAN_Instance *)malloc(sizeof(FDCAN_Instance));
    if (fdcan == NULL)
        return NULL;

    memset(fdcan, 0, sizeof(FDCAN_Instance));

    // 配置发送报文头
    fdcan->txconf.Identifier = config->tx_id;
    fdcan->txconf.IdType = FDCAN_STANDARD_ID;
    fdcan->txconf.TxFrameType = FDCAN_DATA_FRAME;
    fdcan->txconf.DataLength = FDCAN_DLC_BYTES_8;
    fdcan->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    fdcan->txconf.BitRateSwitch = config->use_canfd ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    fdcan->txconf.FDFormat = config->use_canfd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    fdcan->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    fdcan->txconf.MessageMarker = 0;

    // 设置实例参数
    fdcan->fdcan_handle = config->fdcan_handle;
    fdcan->tx_id = config->tx_id;
    fdcan->rx_id = config->rx_id;
    fdcan->use_canfd = config->use_canfd ? 1U : 0U;
    fdcan->can_module_callback = config->can_module_callback;
    fdcan->id = config->id;
    fdcan->tx_data_length = 8;

    FDCANSyncInstanceMode(fdcan);

    if (FDCANAddFilter(fdcan, bus_ctx) != HAL_OK) {
        free(fdcan);
        return NULL;
    }

    if (FDCANEnsureBusStarted(bus_ctx) != HAL_OK) {
        free(fdcan);
        return NULL;
    }

    if (fdcan->rx_id != 0U &&
        fdcan->rx_id <= FDCAN_STD_ID_MAX &&
        std_registry != NULL) {
        std_registry[fdcan->rx_id] = fdcan;
    }

    fdcan_instances[idx++] = fdcan;
    return fdcan;
}

/**
 * @brief 通过FDCAN实例发送消息
 *
 * @param _instance FDCAN实例指针
 * @param timeout 超时时间,单位ms
 * @return uint8_t 1=发送成功, 0=发送失败
 */
uint8_t FDCANTransmit(FDCAN_Instance *_instance, float timeout)
{
    float dwt_start;
    volatile FDCAN_Debug_Bus_s *debug_bus;

    if (_instance == NULL || _instance->fdcan_handle == NULL)
        return 0;

    FDCANSyncInstanceMode(_instance);
    debug_bus = FDCANGetDebugBus(_instance->fdcan_handle);

    // 自动转换数据长度到DLC
    _instance->txconf.DataLength = BytesToDLC(_instance->tx_data_length);

    dwt_start = DWT_GetTimeline_ms();

    // 等待发送FIFO/Queue有空闲位置
    while (HAL_FDCAN_GetTxFifoFreeLevel(_instance->fdcan_handle) == 0U) {
        if ((DWT_GetTimeline_ms() - dwt_start) > timeout) {
            if (debug_bus != NULL) {
                debug_bus->tx_timeout_count++;
                debug_bus->tx_fail_count++;
                debug_bus->last_tx_id = _instance->txconf.Identifier;
                debug_bus->last_tx_dlc = _instance->txconf.DataLength;
                debug_bus->last_tx_fdformat = _instance->txconf.FDFormat;
                debug_bus->last_tx_tick_ms = (uint32_t)DWT_GetTimeline_ms();
                debug_bus->last_hal_status = HAL_TIMEOUT;
                debug_bus->last_error_code = _instance->fdcan_handle->ErrorCode;
                FDCANDebugCopy8(debug_bus->last_tx_data, _instance->tx_buff, _instance->tx_data_length);
            }
            return 0;
        }
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->fdcan_handle, &_instance->txconf, _instance->tx_buff) != HAL_OK) {
        if (debug_bus != NULL) {
            debug_bus->tx_fail_count++;
            debug_bus->last_tx_id = _instance->txconf.Identifier;
            debug_bus->last_tx_dlc = _instance->txconf.DataLength;
            debug_bus->last_tx_fdformat = _instance->txconf.FDFormat;
            debug_bus->last_tx_tick_ms = (uint32_t)DWT_GetTimeline_ms();
            debug_bus->last_hal_status = HAL_ERROR;
            debug_bus->last_error_code = _instance->fdcan_handle->ErrorCode;
            FDCANDebugCopy8(debug_bus->last_tx_data, _instance->tx_buff, _instance->tx_data_length);
        }
        return 0;
    }

    if (debug_bus != NULL) {
        debug_bus->tx_ok_count++;
        debug_bus->last_tx_id = _instance->txconf.Identifier;
        debug_bus->last_tx_dlc = _instance->txconf.DataLength;
        debug_bus->last_tx_fdformat = _instance->txconf.FDFormat;
        debug_bus->last_tx_tick_ms = (uint32_t)DWT_GetTimeline_ms();
        debug_bus->last_hal_status = HAL_OK;
        debug_bus->last_error_code = _instance->fdcan_handle->ErrorCode;
        FDCANDebugCopy8(debug_bus->last_tx_data, _instance->tx_buff, _instance->tx_data_length);
    }

    return 1;
}

/**
 * @brief 设置FDCAN发送数据长度
 *
 * @param _instance FDCAN实例指针
 * @param length 数据长度(字节数)
 */
void FDCANSetDataLength(FDCAN_Instance *_instance, uint8_t length)
{
    uint8_t is_canfd;

    if (_instance == NULL)
        return;

    if (length == 0 || length > FDCAN_MAX_DATA_LEN)
        return;

    FDCANSyncInstanceMode(_instance);
    is_canfd = FDCANInstanceIsCanFd(_instance);

    // 经典CAN模式: 强制限制为8字节
    if (!is_canfd && length > 8U) {
        length = 8U;
    }

    // CAN FD模式: 规范化长度到CAN FD支持的值
    if (is_canfd) {
        if (length > 48U)
            length = 64U;
        else if (length > 32U)
            length = 48U;
        else if (length > 24U)
            length = 32U;
        else if (length > 20U)
            length = 24U;
        else if (length > 16U)
            length = 20U;
        else if (length > 12U)
            length = 16U;
        else if (length > 8U)
            length = 12U;
    }

    _instance->tx_data_length = length;
}

/**
 * @brief 快速发送函数,自动处理数据拷贝和长度设置
 *
 * @param _instance FDCAN实例指针
 * @param data 要发送的数据指针
 * @param length 数据长度(字节数)
 * @param timeout 超时时间,单位ms
 * @return uint8_t 1=发送成功, 0=发送失败
 */
uint8_t FDCANTransmitEx(FDCAN_Instance *_instance, uint8_t *data, uint8_t length, float timeout)
{
    if (_instance == NULL || data == NULL)
        return 0;

    if (length == 0 || length > FDCAN_MAX_DATA_LEN)
        return 0;

    FDCANSetDataLength(_instance, length);
    memcpy(_instance->tx_buff, data, _instance->tx_data_length);

    return FDCANTransmit(_instance, timeout);
}

/* ----------------------- 回调函数定义 --------------------------*/

/**
 * @brief 处理FDCAN接收FIFO中断的通用函数
 *
 * @param _hfdcan FDCAN句柄
 * @param fifox FIFO编号 (FDCAN_RX_FIFO0 或 FDCAN_RX_FIFO1)
 */
static void FDCANFIFOxCallback(FDCAN_HandleTypeDef *_hfdcan, uint32_t fifox)
{
    FDCAN_RxHeaderTypeDef rxconf;
    uint8_t rx_data[FDCAN_MAX_DATA_LEN];
    FDCAN_Instance *matched_instance;
    uint8_t data_len;
    uint8_t dispatched;
    volatile FDCAN_Debug_Bus_s *debug_bus = FDCANGetDebugBus(_hfdcan);

    while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox) > 0U) {
        if (HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, rx_data) != HAL_OK) {
            if (debug_bus != NULL) {
                debug_bus->last_hal_status = HAL_ERROR;
                debug_bus->last_error_code = _hfdcan->ErrorCode;
            }
            break;
        }

        matched_instance = FDCANFindRxInstance(_hfdcan, &rxconf);
        data_len = FDCANResolveRxLength(matched_instance, rxconf.DataLength);
        dispatched = 0U;

        if (matched_instance != NULL) {
            if (data_len == 0U && !FDCANInstanceIsCanFd(matched_instance)) {
                data_len = 8U;
            }

            if (data_len > FDCAN_MAX_DATA_LEN) {
                data_len = FDCAN_MAX_DATA_LEN;
            }

            matched_instance->rx_len = data_len;
            if (data_len > 0U) {
                memcpy(matched_instance->rx_buff, rx_data, data_len);
            }

            if (matched_instance->can_module_callback != NULL) {
                matched_instance->can_module_callback(matched_instance);
                dispatched = 1U;
            }
        }

        if (debug_bus != NULL) {
            debug_bus->last_rx_id = rxconf.Identifier;
            debug_bus->last_rx_dlc = rxconf.DataLength;
            debug_bus->last_rx_tick_ms = (uint32_t)DWT_GetTimeline_ms();
            debug_bus->last_hal_status = HAL_OK;
            debug_bus->last_error_code = _hfdcan->ErrorCode;
            FDCANDebugCopy8(debug_bus->last_rx_data, rx_data, data_len);
            if (dispatched) {
                debug_bus->rx_match_count++;
            } else {
                debug_bus->rx_unmatched_count++;
            }
        }
    }
}

/**
 * @brief FDCAN接收FIFO0回调函数
 *        HAL库中的弱定义回调函数,这里进行重载
 *
 * @param hfdcan FDCAN句柄
 * @param RxFifo0ITs 中断标志
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO0);
    }
}

/**
 * @brief FDCAN接收FIFO1回调函数
 *        HAL库中的弱定义回调函数,这里进行重载
 *
 * @param hfdcan FDCAN句柄
 * @param RxFifo1ITs 中断标志
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U) {
        FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1);
    }
}
