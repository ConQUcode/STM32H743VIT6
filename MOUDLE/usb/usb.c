#include "usb.h"
#include "string.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "main.h"

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

/**
 * @brief USB数据解析任务
 * @note 请在外部主循环或RTOS任务中周期性调用（如每包调用一次或使用死循环和osDelay）
 */
void USB_ProcessTask(void)
{
    uint8_t rx_byte;
    // 将环形缓冲区内所有未处理的数据取空，送入协议状态机
    while (USB_ReadRingBuffer(&rx_byte, 1) > 0)
    {
        protocol_fsm_feed(rx_byte);
    }
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
}

void on_receive_Heartbeat(const Packet_Heartbeat* pkt)
{
    // 收到心跳包，更新时间戳
    usb_last_recv_time = HAL_GetTick();
}

void on_receive_CmdVel(const Packet_CmdVel* pkt)
{
    // 收到速度控制包
    usb_chassis_cmd.linear_x = pkt->linear_x;
    usb_chassis_cmd.linear_y = pkt->linear_y;
    usb_chassis_cmd.angular_z = pkt->angular_z;
    usb_last_recv_time = HAL_GetTick();
}

