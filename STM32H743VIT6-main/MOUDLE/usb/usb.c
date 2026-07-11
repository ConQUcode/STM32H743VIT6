#include "usb.h"
#include "string.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "protocol.h"

/* ------------------ H7 性能/缓存安全配置区 ------------------ */
/* 根据不同编译器语法，将 USB 底层收发数组映射至你申请的 MPU Non-Cacheable 安全区（0x30040000） */

#if defined ( __CC_ARM ) /* MDK ARM Compiler 5 */
    __attribute__((at(0x30040000))) uint8_t usb_rx_buffer[USB_RX_BUFFER_SIZE];
    __attribute__((at(0x30040800))) uint8_t usb_tx_buffer[USB_TX_BUFFER_SIZE];
#elif defined ( __ARMCC_VERSION ) && (__ARMCC_VERSION >= 6010050) /* MDK ARM Compiler 6 (AC6) */
    uint8_t usb_rx_buffer[USB_RX_BUFFER_SIZE] __attribute__((section(".ARM.__at_0x30040000")));
    uint8_t usb_tx_buffer[USB_TX_BUFFER_SIZE] __attribute__((section(".ARM.__at_0x30040800")));
#elif defined ( __GNUC__ ) /* GCC */
    uint8_t usb_rx_buffer[USB_RX_BUFFER_SIZE] __attribute__((section(".RamD2_SRAM2"))); /* 取决于 ld 链接脚本设置 */
    uint8_t usb_tx_buffer[USB_TX_BUFFER_SIZE] __attribute__((section(".RamD2_SRAM2")));
#else
    uint8_t usb_rx_buffer[USB_RX_BUFFER_SIZE];
    uint8_t usb_tx_buffer[USB_TX_BUFFER_SIZE];
#endif


/* 全局变量和标志 */
static usb_rx_callback_t rx_callback = NULL;
static uint8_t usb_initialized = 0;

/* 环形缓冲区定义定义(存放接收到的用户数据) */
static uint8_t ring_buffer[RING_BUFFER_SIZE];
static volatile uint32_t rb_head = 0; // 写入位置
static volatile uint32_t rb_tail = 0; // 读取位置

/* 用于保存协议数据的全局变量 */
USB_Chassis_Cmd_s usb_chassis_cmd;
uint32_t usb_last_recv_time = 0;
USB_ScreenLink_s usb_screen_link;

#define USB_SCREEN_SOF0                 0xA5U
#define USB_SCREEN_SOF1                 0x5AU
#define USB_SCREEN_VERSION              0x01U
#define USB_SCREEN_TYPE_NEXT            0x01U
#define USB_SCREEN_TYPE_ACK             0x81U
#define USB_SCREEN_MAX_PAYLOAD          64U
#define USB_SCREEN_ACK_TIMEOUT_MS       150U
#define USB_SCREEN_NEW_CMD_INTERVAL_MS  50U
#define USB_SCREEN_MAX_RETRY            3U

typedef enum {
    USB_SCREEN_RX_WAIT_SOF0 = 0,
    USB_SCREEN_RX_WAIT_SOF1,
    USB_SCREEN_RX_WAIT_VERSION,
    USB_SCREEN_RX_WAIT_TYPE,
    USB_SCREEN_RX_WAIT_SEQ,
    USB_SCREEN_RX_WAIT_LEN,
    USB_SCREEN_RX_WAIT_PAYLOAD,
    USB_SCREEN_RX_WAIT_CRC_LO,
    USB_SCREEN_RX_WAIT_CRC_HI,
} USB_ScreenRxState_e;

static struct {
    uint8_t state;
    uint8_t version;
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t idx;
    uint8_t payload[USB_SCREEN_MAX_PAYLOAD];
    uint8_t crc_lo;
} usb_screen_rx;

/**
 * @brief USB模块初始化
 */
void USB_Init(void)
{
    if (!usb_initialized)
    {
        // 清理安全缓冲和环形缓冲
        memset(usb_rx_buffer, 0, USB_RX_BUFFER_SIZE);
        memset(usb_tx_buffer, 0, USB_TX_BUFFER_SIZE);
        memset(ring_buffer, 0, RING_BUFFER_SIZE);
        
        rb_head = 0;
        rb_tail = 0;
        rx_callback = NULL;
        usb_initialized = 1;
        
        memset(&usb_chassis_cmd, 0, sizeof(usb_chassis_cmd));
        usb_last_recv_time = 0;
        memset(&usb_screen_link, 0, sizeof(usb_screen_link));
        memset(&usb_screen_rx, 0, sizeof(usb_screen_rx));
    }
}

/**
 * @brief 注册USB接收数据回调函数
 */
void USB_RegisterRxCallback(usb_rx_callback_t callback)
{
    rx_callback = callback;
}

/**
 * @brief USB底层数据接收处理（由 usbd_cdc_if.c 的 CDC_Receive_FS 调用）
 * @note 处于中断/USB接收线程上下文中，不要进行阻塞延时操作
 */
void USB_RxHandler(uint8_t *buf, uint32_t len)
{
    if (len > 0)
    {
        // 因为 USB_RxHandler 运行在中断 (OTG_FS_IRQ) 上下文
        // 此处不再调用 taskENTER_CRITICAL()，而在取数函数中进行保护即可
        
        for (uint32_t i = 0; i < len; i++)
        {
            uint32_t next_head = (rb_head + 1) % RING_BUFFER_SIZE;
            if (next_head != rb_tail) // 环形缓冲区未满
            {
                ring_buffer[rb_head] = buf[i];
                rb_head = next_head;
            }
            else
            {
               // 缓冲区溢出，此处可增加一些统计或断言
                break;
            }
        }
        
        // 通知应用层取数据
        if (rx_callback != NULL)
        {
            // 回调：可以在外头发送信号量给其它任务，而非直接在这里解包（因为容易爆栈/超时）
            rx_callback(buf, len);
        }
    }
}

/**
 * @brief 通过USB发送数据(带有 H7 D-Cache 防呆刷新)
 */
uint8_t USB_Transmit(uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || len > USB_TX_BUFFER_SIZE)
        return USBD_FAIL;
    
    // 如果系统自带状态忙，则返回 BUSY
    extern USBD_HandleTypeDef hUsbDeviceFS;
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc->TxState != 0)
    {
        return USBD_BUSY;
    }

    // 1. 将应用层传递的数据，统一拷贝进映射到了 0x30040000 的 MPU 安全区
    memcpy(usb_tx_buffer, data, len);

    // 补充：为了双保险，即便用了 MPU 也可以做个 flush Cache（防止误配置）。如果完全确认 MPU 生效这里其实可去。
    SCB_CleanDCache_by_Addr((uint32_t *)usb_tx_buffer, (len + 31) & ~31);

    // 2. 调用底层发送，此时底层读的始终是真正的 RAM，而不是 CPU 脏数据
    return CDC_Transmit_FS(usb_tx_buffer, len);
}

/**
 * @brief 通过USB发送字符串
 */
uint8_t USB_TransmitString(const char *str)
{
    if (str == NULL)
        return USBD_FAIL;
    
    uint16_t len = strlen(str);
    return USB_Transmit((uint8_t*)str, len);
}

/**
 * @brief 供应用程序主动从环形缓冲区取定长数据
 * @return 实际取出的长度
 */
uint32_t USB_ReadRingBuffer(uint8_t *data, uint32_t len)
{
    uint32_t read_cnt = 0;
    
    taskENTER_CRITICAL();
    while (read_cnt < len && rb_head != rb_tail)
    {
        data[read_cnt++] = ring_buffer[rb_tail];
        rb_tail = (rb_tail + 1) % RING_BUFFER_SIZE;
    }
    taskEXIT_CRITICAL();
    
    return read_cnt;
}

/* ========================================================================= */
/*                      协议栈回调函数实现 (集成 protocol.h)                   */
/* ========================================================================= */

// 由于是由外部工具生成，手动在此声明下解析函数
extern void protocol_fsm_feed(uint8_t byte);

static uint16_t USB_ScreenCrc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    while (len-- > 0U) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8U; i++) crc = (crc & 1U) ? (crc >> 1U) ^ 0xA001U : (crc >> 1U);
    }
    return crc;
}

static void USB_ScreenResetRx(void) { memset(&usb_screen_rx, 0, sizeof(usb_screen_rx)); }

static uint8_t USB_ScreenSendFrame(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    uint8_t frame[72]; uint16_t crc; uint8_t idx = 0;
    if (len > USB_SCREEN_MAX_PAYLOAD) return USBD_FAIL;
    frame[idx++] = USB_SCREEN_SOF0; frame[idx++] = USB_SCREEN_SOF1; frame[idx++] = USB_SCREEN_VERSION;
    frame[idx++] = type; frame[idx++] = seq; frame[idx++] = len;
    if ((payload != NULL) && (len > 0U)) { memcpy(&frame[idx], payload, len); idx = (uint8_t)(idx + len); }
    crc = USB_ScreenCrc16(&frame[2], (uint16_t)(4U + len)); frame[idx++] = (uint8_t)(crc & 0xFFU); frame[idx++] = (uint8_t)(crc >> 8U);
    return USB_Transmit(frame, idx);
}

static void USB_ScreenHandleFrame(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len)
{
    if ((type == USB_SCREEN_TYPE_ACK) && (len == 1U) && (payload[0] == USB_SCREEN_TYPE_NEXT)) {
        usb_screen_link.last_ack_seq = seq; usb_screen_link.last_ack_type = payload[0]; usb_screen_link.ack_flag = 1U;
        if ((usb_screen_link.state == USB_SCREEN_WAIT_ACK) && (seq == usb_screen_link.pending_seq)) usb_screen_link.state = USB_SCREEN_IDLE;
    }
}

static void USB_ScreenFeedByte(uint8_t byte)
{
    switch (usb_screen_rx.state) {
        case USB_SCREEN_RX_WAIT_SOF0: usb_screen_rx.state = (byte == USB_SCREEN_SOF0) ? USB_SCREEN_RX_WAIT_SOF1 : USB_SCREEN_RX_WAIT_SOF0; break;
        case USB_SCREEN_RX_WAIT_SOF1: usb_screen_rx.state = (byte == USB_SCREEN_SOF1) ? USB_SCREEN_RX_WAIT_VERSION : USB_SCREEN_RX_WAIT_SOF0; break;
        case USB_SCREEN_RX_WAIT_VERSION: usb_screen_rx.version = byte; usb_screen_rx.state = (byte == USB_SCREEN_VERSION) ? USB_SCREEN_RX_WAIT_TYPE : USB_SCREEN_RX_WAIT_SOF0; break;
        case USB_SCREEN_RX_WAIT_TYPE: usb_screen_rx.type = byte; usb_screen_rx.state = USB_SCREEN_RX_WAIT_SEQ; break;
        case USB_SCREEN_RX_WAIT_SEQ: usb_screen_rx.seq = byte; usb_screen_rx.state = USB_SCREEN_RX_WAIT_LEN; break;
        case USB_SCREEN_RX_WAIT_LEN: usb_screen_rx.len = byte; usb_screen_rx.idx = 0U; usb_screen_rx.state = (byte > USB_SCREEN_MAX_PAYLOAD) ? USB_SCREEN_RX_WAIT_SOF0 : ((byte == 0U) ? USB_SCREEN_RX_WAIT_CRC_LO : USB_SCREEN_RX_WAIT_PAYLOAD); break;
        case USB_SCREEN_RX_WAIT_PAYLOAD: usb_screen_rx.payload[usb_screen_rx.idx++] = byte; if (usb_screen_rx.idx >= usb_screen_rx.len) usb_screen_rx.state = USB_SCREEN_RX_WAIT_CRC_LO; break;
        case USB_SCREEN_RX_WAIT_CRC_LO: usb_screen_rx.crc_lo = byte; usb_screen_rx.state = USB_SCREEN_RX_WAIT_CRC_HI; break;
        case USB_SCREEN_RX_WAIT_CRC_HI: { uint8_t frame[68]; uint16_t crc; frame[0] = usb_screen_rx.version; frame[1] = usb_screen_rx.type; frame[2] = usb_screen_rx.seq; frame[3] = usb_screen_rx.len; if (usb_screen_rx.len > 0U) memcpy(&frame[4], usb_screen_rx.payload, usb_screen_rx.len); crc = USB_ScreenCrc16(frame, (uint16_t)(4U + usb_screen_rx.len)); if ((usb_screen_rx.crc_lo == (uint8_t)(crc & 0xFFU)) && (byte == (uint8_t)(crc >> 8U))) USB_ScreenHandleFrame(usb_screen_rx.type, usb_screen_rx.seq, usb_screen_rx.payload, usb_screen_rx.len); USB_ScreenResetRx(); } break;
        default: USB_ScreenResetRx(); break;
    }
}

uint8_t USB_ScreenSendNext(void)
{
    uint32_t now = HAL_GetTick(); uint8_t seq = usb_screen_link.next_seq;
    if (usb_screen_link.state == USB_SCREEN_WAIT_ACK) return USBD_BUSY;
    if ((uint32_t)(now - usb_screen_link.last_send_tick) < USB_SCREEN_NEW_CMD_INTERVAL_MS) return USBD_BUSY;
    if (USB_ScreenSendFrame(USB_SCREEN_TYPE_NEXT, seq, NULL, 0U) != USBD_OK) return USBD_BUSY;
    usb_screen_link.pending_seq = seq; usb_screen_link.next_seq = (uint8_t)(seq + 1U); usb_screen_link.retry_count = 0U;
    usb_screen_link.state = USB_SCREEN_WAIT_ACK; usb_screen_link.wait_ack_start_tick = now; usb_screen_link.last_send_tick = now;
    usb_screen_link.ack_flag = 0U; usb_screen_link.last_error = 0U; return USBD_OK;
}

void USB_ScreenTask(void)
{
    uint32_t now = HAL_GetTick();
    if (usb_screen_link.state != USB_SCREEN_WAIT_ACK) return;
    if ((uint32_t)(now - usb_screen_link.wait_ack_start_tick) < USB_SCREEN_ACK_TIMEOUT_MS) return;
    if (usb_screen_link.retry_count >= USB_SCREEN_MAX_RETRY) { usb_screen_link.state = USB_SCREEN_ERROR; usb_screen_link.last_error = 1U; return; }
    if (USB_ScreenSendFrame(USB_SCREEN_TYPE_NEXT, usb_screen_link.pending_seq, NULL, 0U) == USBD_OK) {
        usb_screen_link.retry_count++; usb_screen_link.wait_ack_start_tick = now; usb_screen_link.last_send_tick = now;
    }
}

uint8_t USB_ScreenIsBusy(void) { return (usb_screen_link.state == USB_SCREEN_WAIT_ACK) ? 1U : 0U; }
void USB_ScreenClearAckFlag(void) { usb_screen_link.ack_flag = 0U; }

/**
 * @brief USB数据解析任务
 * @note 请在外部主循环或RTOS任务中周期性调用（如每包调用一次或使用死循环和osDelay）
 */
void USB_ProcessTask(void)
{
    uint8_t rx_byte;
    while (USB_ReadRingBuffer(&rx_byte, 1) > 0)
    {
        protocol_fsm_feed(rx_byte);
        USB_ScreenFeedByte(rx_byte);
    }
    USB_ScreenTask();
}

void serial_write_byte(uint8_t byte)
{
    USB_Transmit(&byte, 1);
}

void serial_write(const uint8_t* data, uint16_t len)
{
    if (data == NULL || len == 0)
    {
        return;
    }
    USB_Transmit((uint8_t*)data, len);
}

void on_receive_Handshake(const Packet_Handshake* pkt)
{
    // 收到握手包处理
    
    // 【新增测试代码】直接回显：通过 send_* 函数原路发回去给电脑
    send_Handshake(pkt);
}

void on_receive_Heartbeat(const Packet_Heartbeat* pkt)
{
    // 收到心跳包，更新时间戳
    usb_last_recv_time = HAL_GetTick();
    
    // 【新增测试代码】直接回显：通过 send_* 函数原路发回去给电脑
    send_Heartbeat(pkt);
}

void on_receive_CmdVel(const Packet_CmdVel* pkt)
{
    // 收到速度控制包
    usb_chassis_cmd.linear_x = pkt->linear_x;
    usb_chassis_cmd.linear_y = pkt->linear_y;
    usb_chassis_cmd.angular_z = pkt->angular_z;
    usb_last_recv_time = HAL_GetTick();

    // 【新增测试代码】收到什么速度也原路发回
    send_CmdVel(pkt);
}

