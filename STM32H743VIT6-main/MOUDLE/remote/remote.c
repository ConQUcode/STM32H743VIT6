#include "remote.h"
#include <string.h>
#include "bsp_usart.h"
#include "usart.h"     // 引入 huart6
#include "cmsis_os.h"  // 引入 RTOS 接口 API

// 全局唯一的遥控器实例分配，为了能安全做 D-Cache 刷新，内存按32字节对齐
#if defined(__CC_ARM) || (defined(__ARMCC_VERSION) && __ARMCC_VERSION >= 6000000)
    __attribute__((aligned(32))) static Remote_Instance remote_instance_1;
#else
    static Remote_Instance remote_instance_1 __attribute__((aligned(32)));
#endif

Remote_Instance *remote_dev = NULL; 
Remote_Data_s *remote_data = NULL;
Remote_Crsf_Debug_s remote_crsf_debug;
Remote_Boxer_s remote_boxer;

#define CRSF_ADDRESS_FLIGHT_CONTROLLER 0xC8U
#define CRSF_ADDRESS_CRSF_TRANSMITTER  0xEEU
#define CRSF_ADDRESS_RADIO_TRANSMITTER 0xEAU
#define CRSF_ADDRESS_CRSF_RECEIVER     0xECU
#define CRSF_FRAME_TYPE_RC_CHANNELS    0x16U
#define CRSF_RC_CHANNEL_PAYLOAD_LEN    22U
#define CRSF_MAX_FRAME_SIZE            64U
#define CRSF_MIN_LENGTH_FIELD          2U
#define CRSF_CHANNEL_BITS              11U
#define CRSF_CHANNEL_CENTER            992U
#define CRSF_SWITCH_LOW_THRESHOLD      700U
#define CRSF_SWITCH_HIGH_THRESHOLD     1300U
#define CRSF_CRC_POLY                  0xD5U

// 引用外部在 freertos.c 中由 CubeMX 生成的任务句柄
extern osThreadId_t Remot_TaskHandle; 

/**
 * @brief 判断一个字节是否可能是CRSF地址字节
 */
static uint8_t Remote_IsCrsfAddress(uint8_t byte)
{
    return (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER) ||
           (byte == CRSF_ADDRESS_CRSF_TRANSMITTER) ||
           (byte == CRSF_ADDRESS_RADIO_TRANSMITTER) ||
           (byte == CRSF_ADDRESS_CRSF_RECEIVER);
}

/**
 * @brief CRSF CRC8, polynomial 0xD5, 覆盖 type + payload
 */
static uint8_t Remote_CrsfCrc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0U;

    while (len-- > 0U) {
        crc ^= *data++;
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1U) ^ CRSF_CRC_POLY);
            } else {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }

    return crc;
}

static int16_t Remote_CrsfCentered(uint16_t raw)
{
    return (int16_t)((int32_t)raw - (int32_t)CRSF_CHANNEL_CENTER);
}

static uint8_t Remote_CrsfToThreePosSwitch(uint16_t raw)
{
    if (raw < CRSF_SWITCH_LOW_THRESHOLD) {
        return 1U;
    }
    if (raw < CRSF_SWITCH_HIGH_THRESHOLD) {
        return 2U;
    }
    return 3U;
}

static uint8_t Remote_CrsfToTwoPosSwitch(uint16_t raw)
{
    return (raw < CRSF_CHANNEL_CENTER) ? 1U : 2U;
}

static uint8_t Remote_CrsfToKey(uint16_t raw)
{
    return (raw > CRSF_SWITCH_HIGH_THRESHOLD) ? 0U : 1U;
}

static uint16_t Remote_ChannelRaw(const uint16_t *channels, uint8_t index)
{
    if (index >= REMOTE_CRSF_CHANNEL_COUNT) {
        return CRSF_CHANNEL_CENTER;
    }
    return channels[index];
}

static int16_t Remote_ChannelCentered(const uint16_t *channels, uint8_t index)
{
    return Remote_CrsfCentered(Remote_ChannelRaw(channels, index));
}

static void Remote_UpdateBoxerData(const uint16_t *channels)
{
    for (uint8_t i = 0U; i < REMOTE_CRSF_CHANNEL_COUNT; i++) {
        remote_boxer.raw[i] = channels[i];
    }

    remote_boxer.right_x = Remote_ChannelCentered(channels, REMOTE_BOXER_CH_RIGHT_X);
    remote_boxer.right_y = Remote_ChannelCentered(channels, REMOTE_BOXER_CH_RIGHT_Y);
    remote_boxer.left_y  = Remote_ChannelCentered(channels, REMOTE_BOXER_CH_LEFT_Y);
    remote_boxer.left_x  = Remote_ChannelCentered(channels, REMOTE_BOXER_CH_LEFT_X);
    remote_boxer.s1      = Remote_ChannelCentered(channels, REMOTE_BOXER_CH_S1);
    remote_boxer.s2      = Remote_ChannelCentered(channels, REMOTE_BOXER_CH_S2);

    remote_boxer.sa = Remote_CrsfToTwoPosSwitch(Remote_ChannelRaw(channels, REMOTE_BOXER_CH_SA));
    remote_boxer.sb = Remote_CrsfToThreePosSwitch(Remote_ChannelRaw(channels, REMOTE_BOXER_CH_SB));
    remote_boxer.sc = Remote_CrsfToThreePosSwitch(Remote_ChannelRaw(channels, REMOTE_BOXER_CH_SC));
    remote_boxer.sd = Remote_CrsfToTwoPosSwitch(Remote_ChannelRaw(channels, REMOTE_BOXER_CH_SD));
    remote_boxer.se = Remote_CrsfToTwoPosSwitch(Remote_ChannelRaw(channels, REMOTE_BOXER_CH_SE));
    remote_boxer.sf = Remote_CrsfToTwoPosSwitch(Remote_ChannelRaw(channels, REMOTE_BOXER_CH_SF));
}

/**
 * @brief 解包CRSF 0x16 RC Channels Packed帧
 */
static void Remote_ParseCrsfChannels(Remote_Instance *instance, const uint8_t *payload)
{
    uint16_t channels[REMOTE_CRSF_CHANNEL_COUNT] = {0U};

    for (uint8_t ch = 0U; ch < REMOTE_CRSF_CHANNEL_COUNT; ch++) {
        uint16_t bit_offset = (uint16_t)ch * CRSF_CHANNEL_BITS;
        uint8_t byte_index = (uint8_t)(bit_offset >> 3U);
        uint8_t bit_shift = (uint8_t)(bit_offset & 0x07U);
        uint32_t word = payload[byte_index];

        if ((byte_index + 1U) < CRSF_RC_CHANNEL_PAYLOAD_LEN) {
            word |= ((uint32_t)payload[byte_index + 1U] << 8U);
        }
        if ((byte_index + 2U) < CRSF_RC_CHANNEL_PAYLOAD_LEN) {
            word |= ((uint32_t)payload[byte_index + 2U] << 16U);
        }

        channels[ch] = (uint16_t)((word >> bit_shift) & 0x07FFU);
        remote_crsf_debug.raw_channels[ch] = channels[ch];
        remote_crsf_debug.centered_channels[ch] = Remote_CrsfCentered(channels[ch]);
    }

    Remote_UpdateBoxerData(channels);

    // 兼容旧应用层字段，后续应用层可直接切到remote_boxer。
    instance->data.rocker_l_ = remote_boxer.left_x;
    instance->data.rocker_l1 = remote_boxer.left_y;
    instance->data.rocker_r_ = remote_boxer.right_x;
    instance->data.rocker_r1 = remote_boxer.right_y;
    instance->data.dial      = remote_boxer.s1;

    instance->data.switch_left  = remote_boxer.sb;
    instance->data.switch_right = remote_boxer.sc;
    instance->data.KEY[0] = Remote_CrsfToKey(Remote_ChannelRaw(channels, REMOTE_BOXER_CH_SA));
    instance->data.KEY[1] = Remote_CrsfToKey(Remote_ChannelRaw(channels, REMOTE_BOXER_CH_SD));
    instance->data.KEY[2] = Remote_CrsfToKey(Remote_ChannelRaw(channels, REMOTE_BOXER_CH_SE));
    instance->data.KEY[3] = Remote_CrsfToKey(Remote_ChannelRaw(channels, REMOTE_BOXER_CH_SF));

    instance->last_update_time = HAL_GetTick();
    remote_crsf_debug.rc_frame_count++;
}

static uint16_t Remote_RingUnreadLen(uint16_t read_idx, uint16_t write_idx)
{
    if (write_idx >= read_idx) {
        return (uint16_t)(write_idx - read_idx);
    }
    return (uint16_t)(REMOTE_RX_BUF_SIZE - read_idx + write_idx);
}

/**
 * @brief RTOS任务层的解包过程 (脱离中断执行)
 * @param instance 遥控器实例指针
 */
static void Remote_Parse(Remote_Instance *instance) {
    uint8_t *data = instance->rx_buf;
    
    // 如果启用了D-Cache，需要让Cache失效以读取DMA修改后的物理内存真实数据
    // 我们已经在头文件中通过 __attribute__((aligned(32))) 将 rx_buf 强制分离，
    // 其大小为128字节（4条Cache Line），可以直接安全失效，不会误伤结构体中的 read_idx 等解析状态变量
    SCB_InvalidateDCache_by_Addr((uint32_t *)instance->rx_buf, REMOTE_RX_BUF_SIZE);

    // 获取 DMA 当前写指针位置（硬件环形寄存器指针）
    uint16_t write_idx = REMOTE_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(instance->huart->hdmarx);
    if (write_idx >= REMOTE_RX_BUF_SIZE) {
        write_idx = 0U;
    }

    remote_crsf_debug.parse_events++;
    remote_crsf_debug.last_write_idx = write_idx;

    // 当读写指针不对齐时，说明硬件 DMA 写了新数据
    while (instance->read_idx != write_idx) {
        uint16_t unread_len = Remote_RingUnreadLen(instance->read_idx, write_idx);
        if (unread_len < 2U) {
            break;
        }

        uint8_t address = data[instance->read_idx];
        if (Remote_IsCrsfAddress(address) == 0U) {
            instance->read_idx = (instance->read_idx + 1U) % REMOTE_RX_BUF_SIZE;
            remote_crsf_debug.sync_lost_count++;
            continue;
        }

        uint8_t length = data[(instance->read_idx + 1U) % REMOTE_RX_BUF_SIZE];
        uint16_t total_len = (uint16_t)length + 2U;

        remote_crsf_debug.last_address = address;
        remote_crsf_debug.last_frame_len = length;

        if ((length < CRSF_MIN_LENGTH_FIELD) || (total_len > CRSF_MAX_FRAME_SIZE)) {
            instance->read_idx = (instance->read_idx + 1U) % REMOTE_RX_BUF_SIZE;
            remote_crsf_debug.length_error_count++;
            continue;
        }

        if (unread_len < total_len) {
            break;
        }

        uint8_t frame[CRSF_MAX_FRAME_SIZE];
        for (uint16_t i = 0U; i < total_len; i++) {
            frame[i] = data[(instance->read_idx + i) % REMOTE_RX_BUF_SIZE];
        }

        uint8_t crc_received = frame[total_len - 1U];
        uint8_t crc_calculated = Remote_CrsfCrc8(&frame[2], (uint8_t)(length - 1U));
        remote_crsf_debug.last_frame_type = frame[2];
        remote_crsf_debug.last_crc_received = crc_received;
        remote_crsf_debug.last_crc_calculated = crc_calculated;

        if (crc_calculated != crc_received) {
            instance->read_idx = (instance->read_idx + 1U) % REMOTE_RX_BUF_SIZE;
            remote_crsf_debug.crc_error_count++;
            continue;
        }

        remote_crsf_debug.valid_frame_count++;
        remote_crsf_debug.last_frame_time = HAL_GetTick();

        if ((frame[2] == CRSF_FRAME_TYPE_RC_CHANNELS) &&
            (length == (CRSF_RC_CHANNEL_PAYLOAD_LEN + 2U))) {
            Remote_ParseCrsfChannels(instance, &frame[3]);
        }

        instance->read_idx = (instance->read_idx + total_len) % REMOTE_RX_BUF_SIZE;
    }
}

/**
 * @brief 遥控器快速初始化
 *        定死了使用USART6以及专用的RTOS唤醒任务
 */
void RemoteControlInit(void) {
    remote_dev = &remote_instance_1;
    memset(remote_dev, 0, sizeof(Remote_Instance));
    memset(&remote_crsf_debug, 0, sizeof(remote_crsf_debug));
    memset(&remote_boxer, 0, sizeof(remote_boxer));
    remote_data = &remote_instance_1.data;
    
    remote_dev->huart = &huart6;
    remote_dev->task_handle = Remot_TaskHandle; // 绑定目标挂起任务的句柄
    
    // 强制将 CubeMX 默认配置的 DMA Normal 模式改为 Circular 环形模式
    if (remote_dev->huart->hdmarx != NULL) {
        remote_dev->huart->hdmarx->Init.Mode = DMA_CIRCULAR;
        HAL_DMA_Init(remote_dev->huart->hdmarx);
    }
    
    // 首次启动空闲中断和环形DMA接收。它将在后台永不停息地搬运数据，不仅在满时发中断，空闲时也会发中断
    HAL_UARTEx_ReceiveToIdle_DMA(remote_dev->huart, remote_dev->rx_buf, sizeof(remote_dev->rx_buf));
}

/**
 * @brief 遥控器处理任务逻辑封装
 *        放入 StartRemote 任务的 for(;;) 循环内即可
 */
void RemoteControlTask(void) {
    if (remote_dev == NULL) return;
    
    // 无限期挂起，等待空闲中断发送过来的通知掩码 (Flags = 0x01)
    // 该函数在无数据到来时彻底释放CPU，不产生开销
    uint32_t flags = osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
    
    if ((flags & 0x01) == 0x01) {
        // 被唤醒，从缓存提取结构数据
        Remote_Parse(remote_dev);
    }
}

/**
 * @brief 中断接收分发接口
 *        已被包含在弱定义的 HAL_UARTEx_RxEventCallback 中
 */
void Remote_RxCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (remote_dev != NULL && remote_dev->huart == huart) {
        remote_crsf_debug.isr_rx_events++;
        remote_crsf_debug.last_rx_size = Size;

        // 由于咱们配置了 CIRCULAR 环形 DMA 模式，这里绝不要调用 Abort 也绝不要再重新 Receive！！！
        // 硬件 DMA 会在后台源源不断将数据覆盖写入 rx_buf，我们只需要提取即可。
        
        // 动态绑定句柄防被过早调用初始化导致NULL
        if (remote_dev->task_handle == NULL) {
            remote_dev->task_handle = Remot_TaskHandle;
        }

        // 在中断里，直接向目标线程发送标记旗 0x01 (完全非阻塞)
        if (remote_dev->task_handle != NULL) {
            osThreadFlagsSet(remote_dev->task_handle, 0x01);
        }
    }
}

/**
 * @brief 串口硬件错误回调函数 (主要处理 ORE 溢出错误)
 */
void Remote_ErrorCallback(UART_HandleTypeDef *huart) {
	if (remote_dev != NULL && remote_dev->huart == huart) {
        // 发生错误(如打断点引起的 ORE 溢出)后，底层库会自动中止接收并关闭中断。
        // 此时我们必须手动彻底重置状态，并重新下发环形接收指令。
        HAL_UART_AbortReceive(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_dev->rx_buf, sizeof(remote_dev->rx_buf));
    }
}
