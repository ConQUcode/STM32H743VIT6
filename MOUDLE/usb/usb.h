#ifndef USB_H
#define USB_H

#include "stdint.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"  // 包含 FreeRTOS 支持

/* 接收缓冲区大小 */
#define USB_RX_BUFFER_SIZE 2048
/* 数据发送缓冲区大小 */
#define USB_TX_BUFFER_SIZE 2048

/* 环形缓冲区大小 */
#define RING_BUFFER_SIZE 2048

/* USB接收数据回调函数类型 */
typedef void (*usb_rx_callback_t)(uint8_t *data, uint32_t len);

/* 对外暴露给 CubeMX (usbd_cdc_if.c) 的 MPU Non-Cacheable 安全数组 */
extern uint8_t usb_rx_buffer[USB_RX_BUFFER_SIZE];
extern uint8_t usb_tx_buffer[USB_TX_BUFFER_SIZE];

/**
 * @brief USB模块初始化
 */
void USB_Init(void);

/**
 * @brief 注册USB接收数据回调函数
 * @param callback 回调函数指针
 */
void USB_RegisterRxCallback(usb_rx_callback_t callback);

/**
 * @brief 通过USB发送数据
 * @param data 要发送的数据指针
 * @param len 数据长度
 * @return uint8_t 发送状态 (USBD_OK, USBD_BUSY, USBD_FAIL)
 */
uint8_t USB_Transmit(uint8_t *data, uint16_t len);

/**
 * @brief 通过USB发送字符串
 * @param str 字符串指针
 * @return uint8_t 发送状态
 */
uint8_t USB_TransmitString(const char *str);

/**
 * @brief USB接收数据处理(内部函数,由usbd_cdc_if.c调用)
 * @param buf 接收到的数据缓冲区
 * @param len 数据长度
 */
void USB_RxHandler(uint8_t *buf, uint32_t len);

/**
 * @brief 从环形缓冲区读取指定数量的数据
 * @param data 输出目标缓冲区
 * @param len 需要读取的长度
 * @return 实际读取到的长度
 */
uint32_t USB_ReadRingBuffer(uint8_t *data, uint32_t len);

/**
 * @brief USB数据解析任务(建议在RTOS的USB任务循环中调用)
 */
void USB_ProcessTask(void);

/* 底盘控制指令结构体 (需与通信层匹配) */
#pragma pack(1)
typedef struct {
    float linear_x;   // 线速度 m/s
    float linear_y;   // 横移速度 m/s
    float angular_z;  // 角速度 rad/s
} USB_Chassis_Cmd_s;
#pragma pack()

/* 外部变量声明 (记录了解析后的协议指令) */
extern USB_Chassis_Cmd_s usb_chassis_cmd;
extern uint32_t usb_last_recv_time;

#endif // USB_H
