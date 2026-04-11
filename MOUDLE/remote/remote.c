#include "remote.h"
#include <string.h>
#include "usart.h"     // 引入 huart6
#include "cmsis_os.h"  // 引入 RTOS 接口 API

// 全局唯一的遥控器实例分配
static Remote_Instance remote_instance_1;
Remote_Instance *remote_dev = NULL; 

// 引用外部在 freertos.c 中由 CubeMX 生成的任务句柄
extern osThreadId_t Remot_TaskHandle; 

/**
 * @brief 大端序转小端序内联函数 (适用STM32解析)
 */
static inline int16_t be16_to_i16(const uint8_t *p) {
    return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

/**
 * @brief RTOS任务层的解包过程 (脱离中断执行)
 * @param instance 遥控器实例指针
 */
static void Remote_Parse(Remote_Instance *instance) {
    uint8_t *data = instance->rx_buf;
    uint16_t len = sizeof(instance->rx_buf);
    
    // 从缓存查找帧头和帧尾
    for (uint16_t i = 0; i <= len - REMOTE_FRAME_LEN; i++) {
        if (data[i] == REMOTE_HEADER && data[i + REMOTE_FRAME_LEN - 1] == REMOTE_FOOTER) {
            const uint8_t *frame_start = &data[i];
            
            instance->data.KEY[0] = frame_start[1];
            instance->data.KEY[1] = frame_start[2];
            instance->data.KEY[2] = frame_start[3];
            instance->data.KEY[3] = frame_start[4];
            
            instance->data.rocker_l_ = be16_to_i16(&frame_start[5]);
            instance->data.rocker_l1 = be16_to_i16(&frame_start[7]);
            instance->data.rocker_r_ = be16_to_i16(&frame_start[9]);
            instance->data.rocker_r1 = be16_to_i16(&frame_start[11]);
            instance->data.dial      = be16_to_i16(&frame_start[13]);
            
            instance->data.switch_left  = frame_start[15];
            instance->data.switch_right = frame_start[16];
            
            instance->last_update_time = HAL_GetTick();
            return;
        }
    }
}

/**
 * @brief 遥控器快速初始化
 *        定死了使用USART6以及专用的RTOS唤醒任务
 */
void RemoteControlInit(void) {
    remote_dev = &remote_instance_1;
    memset(remote_dev, 0, sizeof(Remote_Instance));
    
    remote_dev->huart = &huart6;
    remote_dev->task_handle = Remot_TaskHandle; // 绑定目标挂起任务的句柄
    
    // 首次启动空闲中断和DMA接收
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
        // 在中断里，直接向目标线程发送标记旗 0x01 (完全非阻塞)
        osThreadFlagsSet(remote_dev->task_handle, 0x01);
        
        // 瞬间重启 DMA 接收下一帧
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_dev->rx_buf, sizeof(remote_dev->rx_buf));
    }
}

/**
 * @brief 重写 HAL 库的接收回调。
 * 如果你在其他文件没有这个函数，这里会自动生效；如果有，可以将这里剪切过去。
 */
__weak void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    Remote_RxCallback(huart, Size);
}