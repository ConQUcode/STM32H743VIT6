/**
 * @file bsp_fdcan.h
 * @author Adapted for STM32H7 FDCAN
 * @brief  FDCAN的bsp层封装,支持CAN FD模式
 * @version 1.0
 * @date 2026-01-30
 *
 * @copyright Copyright (c) 2026
 *
 */
#ifndef BSP_FDCAN_H
#define BSP_FDCAN_H

#include <stdint.h>
#include "fdcan.h"

// 最多能够支持的FDCAN设备数
#define FDCAN_MX_REGISTER_CNT 16
#define FDCAN_MAX_DATA_LEN    64 // CAN FD最大数据长度

// FDCAN数据长度选项（字节）
#define FDCAN_LEN_8   8
#define FDCAN_LEN_12  12
#define FDCAN_LEN_16  16
#define FDCAN_LEN_20  20
#define FDCAN_LEN_24  24
#define FDCAN_LEN_32  32
#define FDCAN_LEN_48  48
#define FDCAN_LEN_64  64

#pragma pack(1) // 1字节对齐

/* FDCAN的实例结构体 */
typedef struct fdcan_instance {
    FDCAN_HandleTypeDef *fdcan_handle;  // FDCAN句柄
    FDCAN_TxHeaderTypeDef txconf;       // FDCAN报文发送配置
    uint32_t tx_id;                     // FDCAN报文发送ID
    uint8_t tx_buff[FDCAN_MAX_DATA_LEN]; // 发送缓存,最大64字节
    uint8_t tx_data_length;             // 实际发送数据长度(字节数)
    
    uint8_t rx_buff[FDCAN_MAX_DATA_LEN]; // 接收缓存,最大64字节
    uint32_t rx_id;                      // 接收id
    uint8_t rx_len;                      // 接收长度(实际字节数)
    
    uint8_t use_canfd;                   // 工作模式: 0=经典CAN(兼容3508等), 1=CAN FD

    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct fdcan_instance *);
    void *id; // 使用FDCAN外设的模块指针

} FDCAN_Instance;
#pragma pack()

/* FDCAN初始化实例结构体 */
typedef struct
{
    FDCAN_HandleTypeDef *fdcan_handle;       // FDCAN句柄
    uint32_t tx_id;                          // 发送id
    uint32_t rx_id;                          // 接收id
    uint8_t use_canfd;                       // 是否启用CAN FD模式: 1=CAN FD, 0=经典CAN
    void (*can_module_callback)(FDCAN_Instance *); // 处理接收数据的回调函数
    void *id;                                // 拥有FDCAN实例的模块地址
} FDCAN_Init_Config_s;

/**
 * @brief 注册(初始化)一个FDCAN实例
 * 
 * @param config 初始化配置指针
 * @return FDCAN_Instance* FDCAN实例指针,失败返回NULL
 */
FDCAN_Instance *FDCANRegister(FDCAN_Init_Config_s *config);

/**
 * @brief 通过FDCAN实例发送消息
 *        发送前需要向FDCAN实例的tx_buff写入发送数据,并设置tx_data_length
 *
 * @param _instance FDCAN实例指针
 * @param timeout 超时时间,单位ms
 * @return uint8_t 1=发送成功, 0=发送失败
 */
uint8_t FDCANTransmit(FDCAN_Instance *_instance, float timeout);

/**
 * @brief 设置FDCAN发送数据长度
 *        支持: 8, 12, 16, 20, 24, 32, 48, 64字节
 *        
 * @param _instance FDCAN实例指针
 * @param length 数据长度(字节数)
 */
void FDCANSetDataLength(FDCAN_Instance *_instance, uint8_t length);

/**
 * @brief 快速发送函数,自动处理数据拷贝和长度设置
 * 
 * @param _instance FDCAN实例指针
 * @param data 要发送的数据指针
 * @param length 数据长度(字节数)
 * @param timeout 超时时间,单位ms
 * @return uint8_t 1=发送成功, 0=发送失败
 */
uint8_t FDCANTransmitEx(FDCAN_Instance *_instance, uint8_t *data, uint8_t length, float timeout);

#endif // BSP_FDCAN_H
