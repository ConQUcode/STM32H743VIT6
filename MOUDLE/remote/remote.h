#ifndef _REMOTE_H_
#define _REMOTE_H_

#include <stdint.h>
#include "main.h"

// 遥控器帧协议常数
#define REMOTE_FRAME_LEN 18
#define REMOTE_HEADER    0xAA
#define REMOTE_FOOTER    0x55

/**
 * @brief 遥控器解析后的有效数据结构
 */
typedef struct {
    uint8_t KEY[4];        // 4字节的按键状态位势
    int16_t rocker_l_;     // 左摇杆 X 轴 
    int16_t rocker_l1;     // 左摇杆 Y 轴 
    int16_t rocker_r_;     // 右摇杆 X 轴 
    int16_t rocker_r1;     // 右摇杆 Y 轴 
    int16_t dial;          // 拨轮数据
    uint8_t switch_left;   // 左侧拨动开关状态
    uint8_t switch_right;  // 右侧拨动开关状态
} Remote_Data_s;

/**
 * @brief 遥控器物理对象实例结构体
 */
typedef struct {
    UART_HandleTypeDef *huart;                    // 绑定的串口句柄
    uint8_t rx_buf[REMOTE_FRAME_LEN * 2];         // 独立分配的DMA接收缓存
    Remote_Data_s data;                           // 遥控器各种按钮/摇杆的数据
    uint32_t last_update_time;                    // 记录最后一次成功接收帧的时间戳 (用于掉线检测)
    
    void *task_handle;                            // 绑定的RTOS任务句柄指针(void* 规避头文件循环包含)
} Remote_Instance;

extern Remote_Instance *remote_dev; // 对外暴露的唯实例指针

// ================== 应用层封装接口 ==================

/**
 * @brief 遥控器快速初始化
 *        用户直接在初始化代码(例如 ChassisInit) 中调用一次即可
 */
void RemoteControlInit(void);

/**
 * @brief 遥控器处理任务逻辑
 *        用户直接在 freertos.c 的 StartRemote 任务的 for(;;) 循环里调用即可
 */
void RemoteControlTask(void);

/**
 * @brief 空闲中断响应逻辑
 *        已被包含在弱定义的 HAL_UARTEx_RxEventCallback 中，一般无需手动调用
 */
void Remote_RxCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif /* _REMOTE_H_ */