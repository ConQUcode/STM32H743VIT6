#include "remote.h"
#include <string.h>
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

// 引用外部在 freertos.c 中由 CubeMX 生成的任务句柄
extern osThreadId_t Remot_TaskHandle; 

/**
 * @brief 大端序转小端序内联函数 
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
    
    // 如果启用了D-Cache，需要让Cache失效以读取DMA修改后的物理内存真实数据
    // 我们已经在头文件中通过 __attribute__((aligned(32))) 将 rx_buf 强制分离，
    // 其大小为128字节（4条Cache Line），可以直接安全失效，不会误伤结构体中的 read_idx 等解析状态变量
    SCB_InvalidateDCache_by_Addr((uint32_t *)instance->rx_buf, REMOTE_RX_BUF_SIZE);

    // 获取 DMA 当前写指针位置（硬件环形寄存器指针）
    uint16_t write_idx = REMOTE_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(instance->huart->hdmarx);
    
    // 当读写指针不对齐时，说明硬件 DMA 写了新数据
    while (instance->read_idx != write_idx) {
        // 计算环形区内的待处理未读总字节数
        int16_t unread_len = write_idx - instance->read_idx;
        if (unread_len < 0) {
            unread_len += REMOTE_RX_BUF_SIZE;
        }
        
        // 如果攒着的未读内容连一个完整帧的长度(18字节)都不够，提前退出，等下一次中断唤醒
        if (unread_len < REMOTE_FRAME_LEN) {
            break;
        }
        
        // 我们利用一个临时数组把这18个待检测的字节，拼接整理成线性连续的便于检测
        // 这自动解决了“环形末尾跨界”到“数组头部”的数据割裂问题
        uint8_t frame[REMOTE_FRAME_LEN];
        for (int i = 0; i < REMOTE_FRAME_LEN; i++) {
            frame[i] = data[(instance->read_idx + i) % REMOTE_RX_BUF_SIZE];
        }
        
        // 判断拼凑出来的线性连续这18字节，是否恰好是头AA尾55
        if (frame[0] == REMOTE_HEADER && frame[REMOTE_FRAME_LEN - 1] == REMOTE_FOOTER) {
            // 解析出合法帧数据
            instance->data.KEY[0] = frame[1];
            instance->data.KEY[1] = frame[2];
            instance->data.KEY[2] = frame[3];
            instance->data.KEY[3] = frame[4];
            
            instance->data.rocker_l_ = be16_to_i16(&frame[5]);
            instance->data.rocker_l1 = -be16_to_i16(&frame[7]);
            instance->data.rocker_r_ = be16_to_i16(&frame[9]);
            instance->data.rocker_r1 = be16_to_i16(&frame[11]);
            instance->data.dial      = be16_to_i16(&frame[13]);
            
            instance->data.switch_left  = frame[15];
            instance->data.switch_right = frame[16];
            
            instance->last_update_time = HAL_GetTick();
            
            // 匹配成功，直接向后消化掉一整帧
            instance->read_idx = (instance->read_idx + REMOTE_FRAME_LEN) % REMOTE_RX_BUF_SIZE;
        } else {
            // 没有匹配上(可能ESP32乱发了数据或错位了)，那么读指针试探性地向后滑动找新的 0xAA 帧头
            instance->read_idx = (instance->read_idx + 1) % REMOTE_RX_BUF_SIZE;
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
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (remote_dev != NULL && remote_dev->huart == huart) {
        // 发生错误(如打断点引起的 ORE 溢出)后，底层库会自动中止接收并关闭中断。
        // 此时我们必须手动彻底重置状态，并重新下发环形接收指令。
        HAL_UART_AbortReceive(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, remote_dev->rx_buf, sizeof(remote_dev->rx_buf));
    }
}