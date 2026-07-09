#ifndef _REMOTE_H_
#define _REMOTE_H_

#include <stdint.h>
#include "main.h"
#include "usart.h"

// 遥控器帧协议常数
#define REMOTE_FRAME_LEN 18
#define REMOTE_HEADER    0xAA
#define REMOTE_FOOTER    0x55
#define REMOTE_RX_BUF_SIZE 128 // 环形缓冲区尺寸，设为32的整数倍完美适应 STM32H7 D-Cache

// 当前接收机协议: RadioMaster Boxer + ELRS NANO 推荐使用 CRSF UART
#define REMOTE_PROTOCOL_CRSF 1
#define REMOTE_CRSF_CHANNEL_COUNT 16

/*
 * Boxer通过EdgeTX Mixer输出CH1~CH16，CRSF只传通道号，不传物理控件名。
 * 下面是默认AETR + AUX映射；如果Watch发现某个开关在其他通道变化，只改这里即可。
 */
#define REMOTE_BOXER_CH_RIGHT_X 0U  // CH1 Aileron
#define REMOTE_BOXER_CH_RIGHT_Y 1U  // CH2 Elevator
#define REMOTE_BOXER_CH_LEFT_Y  2U  // CH3 Throttle
#define REMOTE_BOXER_CH_LEFT_X  3U  // CH4 Rudder
#define REMOTE_BOXER_CH_SA      4U  // CH5
#define REMOTE_BOXER_CH_SB      5U  // CH6
#define REMOTE_BOXER_CH_SC      6U  // CH7
#define REMOTE_BOXER_CH_SD      7U  // CH8
#define REMOTE_BOXER_CH_SE      8U  // CH9
#define REMOTE_BOXER_CH_SF      9U  // CH10
#define REMOTE_BOXER_CH_S1      10U // CH11
#define REMOTE_BOXER_CH_S2      11U // CH12
#define REMOTE_BOXER_CH_6POS    10U // CH11, measured six-position switch

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
 * @brief CRSF接收调试数据，便于先在Watch窗口确认链路是否打通
 */
typedef struct {
    uint32_t isr_rx_events;       // USART6 RxEvent回调触发次数
    uint32_t parse_events;        // RemoteControlTask实际进入解析次数
    uint32_t valid_frame_count;   // CRC通过的CRSF帧数量
    uint32_t rc_frame_count;      // 成功解析RC通道帧数量
    uint32_t crc_error_count;     // CRC错误帧数量
    uint32_t length_error_count;  // 长度异常帧数量
    uint32_t sync_lost_count;     // 滑动丢弃的非CRSF起始字节数量
    uint32_t last_frame_time;     // 最近一次CRC通过帧的HAL tick
    uint16_t last_rx_size;        // HAL回调传入的Size
    uint16_t last_write_idx;      // 当前DMA写指针位置
    uint8_t last_address;         // 最近一次CRSF地址字节
    uint8_t last_frame_len;       // 最近一次CRSF length字段
    uint8_t last_frame_type;      // 最近一次CRSF type字段
    uint8_t last_crc_received;    // 最近一次收到的CRC
    uint8_t last_crc_calculated;  // 最近一次计算出的CRC
    uint16_t raw_channels[REMOTE_CRSF_CHANNEL_COUNT];      // 16路CRSF原始通道值
    int16_t centered_channels[REMOTE_CRSF_CHANNEL_COUNT];  // 以中位值为0的通道值
} Remote_Crsf_Debug_s;

/**
 * @brief RadioMaster Boxer语义化数据，先用于调试确认通道和控件对应关系
 */
typedef struct {
    int16_t left_x;       // 左摇杆横向
    int16_t left_y;       // 左摇杆纵向
    int16_t right_x;      // 右摇杆横向
    int16_t right_y;      // 右摇杆纵向
    int16_t s1;           // S1旋钮
    int16_t s2;           // S2旋钮
    uint8_t sa;           // SA两段开关: 1/2
    uint8_t sb;           // SB三段开关: 1/2/3
    uint8_t sc;           // SC三段开关: 1/2/3
    uint8_t sd;           // SD两段开关: 1/2
    uint8_t se;           // SE自锁开关: 1/2
    uint8_t sf;           // SF回弹开关: 1/2
    uint8_t six_pos;      // 六段开关: 1/2/3/4/5/6
    uint16_t raw[REMOTE_CRSF_CHANNEL_COUNT]; // 保留一份语义层看到的原始通道值
} Remote_Boxer_s;

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
extern Remote_Crsf_Debug_s remote_crsf_debug; // CRSF接收调试数据
extern Remote_Boxer_s remote_boxer; // RadioMaster Boxer语义化调试数据

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

/**
 * @brief 串口错误处理逻辑
 */
void Remote_ErrorCallback(UART_HandleTypeDef *huart);

#endif /* _REMOTE_H_ */
