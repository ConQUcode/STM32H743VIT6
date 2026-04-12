#ifndef _REMOTE_H_
#define _REMOTE_H_

#include <stdint.h>
#include "main.h"

// 遥控器帧协议常数
#define REMOTE_FRAME_LEN 18
#define REMOTE_HEADER    0xAA
#define REMOTE_FOOTER    0x55
#define REMOTE_RX_BUF_SIZE 128 // 环形缓冲区尺寸，设为32的整数倍完美适应 STM32H7 D-Cache

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
    
    // 为了防止 D-Cache 刷新时误伤同结构体内的其它变量，这里强制 rx_buf 本身32字节对齐
#if defined(__CC_ARM) || (defined(__ARMCC_VERSION) && __ARMCC_VERSION >= 6000000)
    __attribute__((aligned(32))) uint8_t rx_buf[REMOTE_RX_BUF_SIZE];
#else
    uint8_t rx_buf[REMOTE_RX_BUF_SIZE] __attribute__((aligned(32)));
#endif

    uint16_t read_idx;                            // 软件解包读指针
    Remote_Data_s data;                           // 遥控器各种按钮/摇杆的数据
    uint32_t last_update_time;                    // 记录最后一次成功接收帧的时间戳 (用于掉线检测)
    
    void *task_handle;                            // 绑定的RTOS任务句柄指针(void* 规避头文件循环包含)
} Remote_Instance;

extern Remote_Instance *remote_dev; // 对外暴露的唯实例指针
extern Remote_Data_s *remote_data; // 对外暴露的数据实例

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