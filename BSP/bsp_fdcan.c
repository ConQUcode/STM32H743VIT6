#include "bsp_fdcan.h"
#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_dwt.h"

// FDCAN实例数组,用于中断回调时查找对应的实例
static FDCAN_Instance *fdcan_instances[FDCAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局FDCAN实例索引,每次有新的模块注册会自增

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
        return bytes << 16; // DLC 0-8
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
    uint8_t dlc_value = (dlc >> 16) & 0x0F;
    return dlc_to_bytes[dlc_value];
}

/**
 * @brief 在第一个FDCAN实例初始化的时候会自动调用此函数,启动FDCAN服务
 *
 * @note  此函数会启动FDCAN1和FDCAN2,开启接收FIFO0和FIFO1的新消息中断
 *
 */
static void FDCANServiceInit(void)
{
    // 启动FDCAN1
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
    
    // 启动FDCAN2
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);
}

/**
 * @brief 添加过滤器以实现对特定id的报文的接收
 *        FDCAN使用更灵活的过滤器策略,支持64个标准ID过滤器
 *
 * @note  使用Range模式配置过滤器,每个实例占用一个过滤器
 *        前一半实例分配到FIFO0,后一半分配到FIFO1,实现负载均衡
 *
 * @param _instance FDCAN instance owned by specific module
 */
static void FDCANAddFilter(FDCAN_Instance *_instance)
{
    FDCAN_FilterTypeDef filter_config;
    static uint8_t fdcan1_filter_idx = 0;
    static uint8_t fdcan2_filter_idx = 0;
    
    uint8_t *filter_idx = (_instance->fdcan_handle == &hfdcan1) ? &fdcan1_filter_idx : &fdcan2_filter_idx;
    
    filter_config.IdType       = FDCAN_STANDARD_ID;          // 标准ID
    filter_config.FilterIndex  = (*filter_idx)++;            // 动态分配过滤器索引
    filter_config.FilterType   = FDCAN_FILTER_RANGE;         // 范围过滤模式
    filter_config.FilterConfig = (*filter_idx <= FDCAN_MX_REGISTER_CNT / 2) ? 
                                 FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1; // 负载均衡
    filter_config.FilterID1    = _instance->rx_id;           // 起始ID
    filter_config.FilterID2    = _instance->rx_id;           // 结束ID (单个ID)
    
    HAL_FDCAN_ConfigFilter(_instance->fdcan_handle, &filter_config);
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
    if (!idx) {
        FDCANServiceInit(); // 初始化FDCAN总线
    }
    if (idx >= FDCAN_MX_REGISTER_CNT) {
        // 超出最大注册数量
        return NULL;
    }
    
    // 检查是否重复注册
    for (uint8_t i = 0; i < idx; ++i) {
        if (fdcan_instances[i]->fdcan_handle == config->fdcan_handle && 
            fdcan_instances[i]->rx_id == config->rx_id) {
            // 重复注册
            return NULL;
        }
    }

    // 初始化实例
    FDCAN_Instance *fdcan = (FDCAN_Instance *)malloc(sizeof(FDCAN_Instance));
    memset(fdcan, 0, sizeof(FDCAN_Instance));

    // 配置发送报文头
    fdcan->txconf.Identifier          = config->tx_id;                                      // 标准发送ID
    fdcan->txconf.IdType              = FDCAN_STANDARD_ID;                                  // 标准ID
    fdcan->txconf.TxFrameType         = FDCAN_DATA_FRAME;                                   // 数据帧
    fdcan->txconf.DataLength          = FDCAN_DLC_BYTES_8;                                  // 默认8字节
    fdcan->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;                                   // 错误状态指示
    fdcan->txconf.BitRateSwitch       = config->use_canfd ? FDCAN_BRS_ON : FDCAN_BRS_OFF;  // CAN FD位速率切换
    fdcan->txconf.FDFormat            = config->use_canfd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN; // CAN FD格式
    fdcan->txconf.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;                                 // 不存储发送事件
    fdcan->txconf.MessageMarker       = 0;                                                  // 消息标记

    // 设置实例参数
    fdcan->fdcan_handle          = config->fdcan_handle;
    fdcan->tx_id                 = config->tx_id;
    fdcan->rx_id                 = config->rx_id;
    fdcan->use_canfd             = config->use_canfd; // 保存工作模式
    fdcan->can_module_callback   = config->can_module_callback;
    fdcan->id                    = config->id;
    fdcan->tx_data_length        = 8; // 默认8字节

    FDCANAddFilter(fdcan); // 添加FDCAN过滤器

    fdcan_instances[idx++] = fdcan; // 添加到实例数组

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
    static uint32_t busy_count;
    static volatile float wait_time __attribute__((unused));
    float dwt_start = DWT_GetTimeline_ms();

    // 自动转换数据长度到DLC
    _instance->txconf.DataLength = BytesToDLC(_instance->tx_data_length);

    // 等待发送FIFO/Queue有空闲位置
    while (HAL_FDCAN_GetTxFifoFreeLevel(_instance->fdcan_handle) == 0) {
        if (DWT_GetTimeline_ms() - dwt_start > timeout) {
            busy_count++;
            return 0; // 超时
        }
    }

    wait_time = DWT_GetTimeline_ms() - dwt_start;

    // 添加消息到发送FIFO/Queue
    if (HAL_FDCAN_AddMessageToTxFifoQ(_instance->fdcan_handle, &_instance->txconf, _instance->tx_buff) != HAL_OK) {
        busy_count++;
        return 0;
    }

    return 1; // 发送成功
}

/**
 * @brief 设置FDCAN发送数据长度
 * 
 * @param _instance FDCAN实例指针
 * @param length 数据长度(字节数)
 */
void FDCANSetDataLength(FDCAN_Instance *_instance, uint8_t length)
{
    // 安全检查: 长度必须在有效范围内
    if (length == 0 || length > FDCAN_MAX_DATA_LEN)
        return;
    
    // 经典CAN模式: 强制限制为8字节(兼容大疆3508等传统CAN设备)
    if (!_instance->use_canfd && length > 8) {
        length = 8;
    }
    
    // CAN FD模式: 规范化长度到CAN FD支持的值
    if (_instance->use_canfd) {
        if (length > 48)
            length = 64;
        else if (length > 32)
            length = 48;
        else if (length > 24)
            length = 32;
        else if (length > 20)
            length = 24;
        else if (length > 16)
            length = 20;
        else if (length > 12)
            length = 16;
        else if (length > 8)
            length = 12;
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
    if (length == 0 || length > FDCAN_MAX_DATA_LEN)
        return 0;
    
    // 经典CAN模式长度检查
    if (!_instance->use_canfd && length > 8)
        return 0; // 经典CAN不支持超过8字节
    
    // 拷贝数据到发送缓冲区
    memcpy(_instance->tx_buff, data, length);
    
    // 设置数据长度
    FDCANSetDataLength(_instance, length);
    
    // 发送
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

    // 处理FIFO中的所有消息
    while (HAL_FDCAN_GetRxFifoFillLevel(_hfdcan, fifox) > 0) {
        // 从FIFO获取消息
        if (HAL_FDCAN_GetRxMessage(_hfdcan, fifox, &rxconf, rx_data) != HAL_OK) {
            break;
        }

        // 转换DLC到实际字节数
        uint8_t data_len = DLCToBytes(rxconf.DataLength);

        // 遍历所有实例,找到匹配的实例
        for (size_t i = 0; i < idx; ++i) {
            if (_hfdcan == fdcan_instances[i]->fdcan_handle && 
                rxconf.Identifier == fdcan_instances[i]->rx_id) {
                
                if (fdcan_instances[i]->can_module_callback != NULL) {
                    fdcan_instances[i]->rx_len = data_len;
                    memcpy(fdcan_instances[i]->rx_buff, rx_data, data_len);
                    fdcan_instances[i]->can_module_callback(fdcan_instances[i]);
                }
                break;
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
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
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
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) {
        FDCANFIFOxCallback(hfdcan, FDCAN_RX_FIFO1);
    }
}
