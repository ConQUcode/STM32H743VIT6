/**
 * @file damiao_motor.h
 * @brief  达妙(Damiao)智能关节电机驱动 —— 适配 STM32H7 FDCAN
 * @version 1.0
 * @date    2026-05-03
 *
 * @note
 * ==================================  达妙电机 CAN 协议说明 ==================================
 *
 * 【通信模型】一发一收：电机只有收到控制帧后才会上报反馈帧，因此必须周期性发送控制帧。
 *
 * 【CAN ID 分配】
 *   每个电机有两个 CAN ID：
 *   - SlaveID  (ESC_ID)：电机接收控制帧的 ID，范围 0x01 ~ 0x7F，由达妙上位机设置。
 *   - MasterID (MST_ID)：电机发送反馈帧的 ID，通常 = SlaveID + 0x10，由达妙上位机设置。
 *   注意：MasterID 不能设为 0x00（广播地址），会导致通信异常。
 *
 * 【CAN 参数】
 *   - 波特率：1 Mbps，经典 CAN 2.0B
 *   - 字节序：大端 (Big-Endian)
 *   - 数据域：8 字节
 *
 * 【控制模式】
 *   模式通过达妙 Windows 上位机预先设置并保存到电机 Flash，代码中不能动态切换模式类型。
 *   代码中的 DM_Control_Mode_e 仅用于**告知本驱动按照哪种协议格式打包控制帧**，
 *   必须与电机内部设置的模式保持一致。
 *
 *   MIT 模式 (mode 1)：CAN ID = SlaveID（无偏移）
 *     电机内部执行 τ = KP × (p_des - p) + KD × (v_des - v) + t_ff
 *     利用 MIT 的 KP/KD/前馈力矩三个参数可衍生出三种控制方式：
 *       - 位置控制：KP ≠ 0，KD ≠ 0，t_ff = 0
 *       - 速度控制：KP = 0，KD ≠ 0，t_ff = 0
 *       - 力矩控制：KP = 0，KD = 0，t_ff = 设定值
 *
 *   位置速度模式 (mode 2)：CAN ID = SlaveID + 0x100
 *     电机内部三环串级控制 (位置环→速度环→电流环)
 *
 *   速度模式 (mode 3)：CAN ID = SlaveID + 0x200
 *     电机内部纯速度环控制
 *
 * 【控制帧格式 - MIT 模式】(紧凑打包, 共8字节)
 *   Byte0:   位置 [15:8]        (uint16, 大端, 16bit)
 *   Byte1:   位置 [7:0]         映射: float_to_uint(pos, -p_max, +p_max, 16)
 *   Byte2:   速度 [11:4]        (12bit)
 *   Byte3:   速度[3:0] | KP[11:8]  映射: float_to_uint(vel, -v_max, +v_max, 12)
 *   Byte4:   KP [7:0]           (12bit)  映射: float_to_uint(kp, 0, 500, 12)
 *   Byte5:   KD [11:4]          (12bit)
 *   Byte6:   KD[3:0] | 力矩[11:8]  映射: float_to_uint(kd, 0, 5, 12)
 *   Byte7:   前馈力矩 [7:0]     (12bit)  映射: float_to_uint(tor, -t_max, +t_max, 12)
 *
 *   注意: 使能/失能通过独立命令帧控制,不在 MIT 帧中携带
 *
 * 【控制帧格式 - 位置速度模式】
 *   Byte0-1: 位置期望值 (uint16, 大端)
 *   Byte2-3: 速度期望值 (uint16, 大端)
 *   Byte4-6: 保留 (填 0x00)
 *   Byte7:   0xFC 使能 / 0xFD 失能
 *
 * 【控制帧格式 - 速度模式】
 *   Byte0-1: 速度期望值 (uint16, 大端)
 *   Byte2-6: 保留 (填 0x00)
 *   Byte7:   0xFC 使能 / 0xFD 失能
 *
 * 【反馈帧格式 - 所有模式通用】(CAN ID = MasterID)
 *   Byte0:   电机 ID [3:0] + 状态 [7:4]
 *   Byte1-2: 当前位置 (16bit, 大端), 映射到 -p_max ~ +p_max
 *   Byte3-4: 当前速度 (12bit): (rx[3]<<4) | (rx[4]>>4), 映射到 -v_max ~ +v_max
 *   Byte4-5: 当前力矩 (12bit): ((rx[4]&0x0F)<<8) | rx[5], 映射到 -t_max ~ +t_max
 *   Byte6:   MOS 温度 (℃)
 *   Byte7:   转子温度 (℃)
 *
 * 【多圈累计】
 *   编码器为 16 位 (0 ~ 65535)，半区间阈值 32768。
 *   通过比较相邻两次位置差值的跳变方向和幅度来判断是否跨越了编码器零点，
 *   实现连续多圈累计，原理与大疆电机一致。
 *
 * 【滤波】
 *   速度采用一阶低通滤波 (DM_SPEED_SMOOTH_COEF = 0.35)
 *   力矩采用一阶低通滤波 (DM_CURRENT_SMOOTH_COEF = 0.90)
 *   系数为 1.0f 时关闭滤波。
 *
 * 【与 DJI 电机驱动的接口对齐】
 *   本驱动的公有 API 命名和功能完全对标 dji_motor.h：
 *     DJIMotorInit  → DMMotorInit
 *     DJIMotorControl → DMMotorControl
 *     DJIMotorSetRef → DMMotorSetRef
 *     等等...
 *   应用层可以用完全相同的调用方式控制两种电机。
 *
 * 【与 DJI 电机驱动的关键差异】
 *   1. CAN 拓扑：达妙是"一对一"(每个电机独立 CAN ID)，DJI 是"四合一"(4 电机共享控制帧)
 *   2. 控制输出：达妙发位置/速度/力矩目标，DJI 发 int16 电流值
 *   3. 编码器：达妙 16 位 (0~65535)，DJI 12 位 (0~8191)
 *   4. 使能方式：达妙需先发使能帧 {0xFF*7, 0xFC} 再发模式配置帧到 0x7FF，DJI 发电流即生效
 *   5. 反馈触发：达妙只有收到控制帧后才回复，DJI 电机主动上报
 *
 * 【CAN 总线共存】
 *   达妙和 DJI 电机可以挂载在同一路 FDCAN 总线上，互不干扰：
 *   - 两种电机使用不同的 CAN ID 范围
 *   - 都使用经典 CAN 模式 (1Mbps)
 *   - 各自独立注册 FDCAN_Instance
 *   - 各自有独立的 Control() 函数和 RTOS 任务
 *
 * 【使用步骤】
 *   1. 在达妙上位机中设置好电机型号、SlaveID、MasterID、控制模式(推荐MIT)、CAN 波特率(1M)
 *   2. 在 application 层构造 Motor_Init_Config_s，填入对应的 CAN ID 和 PID 参数
 *   3. 调用 DMMotorInit() 注册电机
 *   4. 调用 DMMotorSetControlMode() 告知驱动当前电机的工作模式
 *   5. 调用 DMMotorSetMITParams() 设置 MIT 模式的 KP/KD（仅 MIT 模式需要）
 *   6. 周期性调用 DMMotorControl()（1kHz，需自行创建 RTOS 任务）
 *   7. 应用层通过 DMMotorSetRef() 设定目标值
 * ========================================================================================
 */

#ifndef DAMIAO_MOTOR_H
#define DAMIAO_MOTOR_H

#include "bsp_fdcan.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"

/* ================================== 宏定义 ================================== */

/** @brief 最大支持的达妙电机数量
 *  @note  每个达妙电机占用一个 FDCAN_Instance (同时收发)，
 *         不像 DJI 需要 6 个公共发送实例。所以数量设 8 足够使用。 */
#define DM_MOTOR_CNT 8

/* ---------- 滤波系数 ---------- */
/**
 * @brief 速度滤波系数 (一阶低通)
 * @note  值越小滤波越强，1.0f 时关闭滤波。
 *        达妙电机原始速度反馈噪声比 DJI 稍大，0.35 是比较均衡的选择。
 */
#define DM_SPEED_SMOOTH_COEF    0.35f

/**
 * @brief 力矩/电流滤波系数 (一阶低通)
 * @note  力矩反馈噪声较大，必须用强滤波。建议 ≥ 0.85。
 */
#define DM_CURRENT_SMOOTH_COEF  0.90f

/* ---------- 编码器参数 ---------- */
/**
 * @brief 达妙编码器总刻度数 (16 位)
 * @note  0 ~ 65535 对应 -p_max ~ +p_max 的物理角度范围
 */
#define DM_ECD_RESOLUTION       65536

/**
 * @brief 半区间阈值，用于多圈累计时判断编码器是否溢出翻转
 * @note  相邻两次编码器差值超过此阈值即认为发生了跨零点翻转
 */
#define DM_ECD_HALF_RANGE       (DM_ECD_RESOLUTION / 2)

/* ---------- 各型号电机默认参数 ---------- */
/**
 * @brief 以下宏定义了各型号达妙电机的默认物理量范围
 * @note  这些值必须与达妙上位机中设置的参数一致，否则反馈值会换算错误。
 *        如果在上位机中修改了这些参数，需要同步更新对应的宏或通过 DMMotorSetMITParams() 修改。
 *
 *  P_MAX：位置最大值 (弧度)，电机一圈 = 2π rad ≈ 6.283 rad。
 *         12.5 rad ≈ 2 圈范围，是比较常用的设置。
 *  V_MAX：速度最大值 (rad/s)，45 rad/s ≈ 430 rpm (DM4310/DM4340 常用值)。
 *  T_MAX：力矩最大值 (Nm)，取决于电机固件设定，默认 10 Nm 是个保守参考值。
 */

/* DM4310: 额定 0.25Nm，峰值 0.75Nm */
#define DM4310_P_MAX 12.5f   /**< DM4310 位置范围 ±2圈 */
#define DM4310_V_MAX 30.0f   /**< DM4310 速度范围 ±30 rad/s */
#define DM4310_T_MAX 10.0f   /**< DM4310 力矩范围 ±10 Nm */

/* DM4340: 与 DM4310 同系列,参数基本一致 */
#define DM4340_P_MAX 12.5f
#define DM4340_V_MAX 30.0f
#define DM4340_T_MAX 10.0f

/* DM6006: 额定 4Nm，峰值 11Nm，双编码器，速度较低 */
#define DM6006_P_MAX 12.5f
#define DM6006_V_MAX 30.0f   /**< DM6006 速度范围 ±30 rad/s */
#define DM6006_T_MAX 10.0f

/* DM8009: 大扭矩型号 */
#define DM8009_P_MAX 12.5f
#define DM8009_V_MAX 20.0f   /**< DM8009 速度范围 ±20 rad/s */
#define DM8009_T_MAX 10.0f

/* ---------- MIT 模式协议约束 ---------- */
/**
 * @brief MIT 帧中各字段的位宽和物理量范围
 * @note  位置=16bit, 速度/KP/KD/力矩=12bit (紧凑打包)
 *        这些值必须与达妙上位机中设置的参数一致
 */
#define DM_MIT_POS_BITS  16     /**< 位置位宽 */
#define DM_MIT_VEL_BITS  12     /**< 速度位宽 */
#define DM_MIT_KP_BITS   12     /**< KP 位宽 */
#define DM_MIT_KD_BITS   12     /**< KD 位宽 */
#define DM_MIT_TOR_BITS  12     /**< 前馈力矩位宽 */

#define DM_MIT_KP_MAX    500.0f /**< KP 映射上限 */
#define DM_MIT_KD_MAX    500.0f /**< KD 映射上限 (与 KP 一致, 0~500) */

/* ---------- MIT 模式默认参数 ---------- */
#define DM_MIT_DEFAULT_KP 1.0f  /**< 默认 KP (位置刚度) */
#define DM_MIT_DEFAULT_KD 0.2f  /**< 默认 KD (速度阻尼), 不可为零 */


/* ================================== 类型定义 ================================== */

/**
 * @brief 达妙电机控制模式枚举
 * @note  此枚举用于告知驱动按哪种协议格式打包控制帧，
 *        必须与电机在达妙上位机中设置的模式一致。
 */
typedef enum {
    DM_MODE_MIT     = 0,  /**< MIT 混合控制模式 (位置+速度+力矩并联,最常用) */
    DM_MODE_POS_VEL = 1,  /**< 位置速度串级模式 (电机内部三环串级) */
    DM_MODE_VEL     = 2,  /**< 纯速度模式 (电机内部速度环) */
} DM_Control_Mode_e;

/**
 * @brief MIT 模式参数
 * @note  这些参数在两种情况下使用：
 *        1. 打包 MIT 控制帧时，kp/kd 需要转换为 uint16 填入帧中
 *        2. 解析反馈帧时，p_max/v_max/t_max 用于将原始 uint16 换算为物理量
 */
typedef struct {
    float kp;       /**< 位置刚度 KP，用于 MIT 控制帧 */
    float kd;       /**< 速度阻尼 KD，用于 MIT 控制帧 */
    float p_max;    /**< 位置最大值 (弧度)，用于反馈换算和控制帧打包 */
    float v_max;    /**< 速度最大值 (rad/s)，用于反馈换算和控制帧打包 */
    float t_max;    /**< 力矩最大值 (Nm 或 A)，用于反馈换算 */
} DM_MIT_Params_s;

/**
 * @brief MIT 运动配置 (用于外部位置环的梯形速度限制)
 * @note  单位均为弧度制:
 *        - max_vel_rad_s: 最大角速度 (rad/s)
 *        - max_acc_rad_s2: 最大角加速度 (rad/s^2)
 */
typedef struct {
    float max_vel_rad_s;
    float max_acc_rad_s2;
    float pos_rad;       /**< 当前位置指令 (rad) */
    float vel_rad_s;     /**< 当前速度指令 (rad/s), 用于加速度限制 */
    uint8_t initialized; /**< 是否已用反馈速度初始化 */
} DM_MIT_Profile_s;

/**
 * @brief 达妙电机反馈测量值
 * @note  结构与 DJI_Motor_Measure_s 对齐，包含原始值和物理量两种形式。
 *        同时支持单圈角度和多圈累计角度，方便位置闭环。
 */
typedef struct {
    /* ---- 原始 CAN 反馈值 ---- */
    uint16_t position_raw;  /**< 原始位置 (0~65535, 来自反馈帧 Byte1-2) */
    int16_t  velocity_raw;  /**< 原始速度 (有符号, 来自反馈帧 Byte3-4) */
    int16_t  torque_raw;    /**< 原始力矩 (有符号, 来自反馈帧 Byte5-6) */
    uint8_t  temperature;   /**< MOS 温度 (来自反馈帧 Byte6) */
    uint8_t  rotor_temp;    /**< 转子温度 (来自反馈帧 Byte7) */
    uint8_t  motor_id;      /**< 电机 SlaveID (来自反馈帧 Byte0 低4位) */
    uint8_t  state;         /**< 电机状态 (来自反馈帧 Byte0 高4位) */

    /* ---- 物理量 (由原始值换算) ---- */
    float position_rad;     /**< 当前位置 (弧度) */
    float last_position_rad;/**< 上一次位置 (弧度), 用于多圈累计 */
    float velocity_rad_s;   /**< 当前速度 (rad/s) */
    float torque_nm;        /**< 当前力矩 (Nm) + 一阶低通滤波 */

    /* ---- 角度制 (与 DJI 对齐, 方便 PID 调参) ---- */
    float angle_single_round; /**< 单圈角度 (度), 范围 0~360 */
    float speed_aps;          /**< 滤波后角速度 (度/秒, 用于速度环反馈) */

    /* ---- 多圈累计 ---- */
    int32_t  total_round;     /**< 累计圈数 (带符号) */
    float    total_angle;     /**< 累计总角度 (度) */
    float    zero_offset;     /**< 零点偏移量 (度), DMMotorReset 时更新 */
} DM_Motor_Measure_s;

/**
 * @brief 达妙电机实例
 * @note  结构与 DJIMotor_Instance 对齐。
 *        每个达妙电机独立持有一个 FDCAN_Instance，同时处理收发。
 *        不采用 DJI 的分组发送(sender_assignment)方式。
 */
typedef struct {
    DM_Motor_Measure_s measure;                    /**< 反馈测量值 (CAN 中断中更新) */
    Motor_Control_Setting_s motor_settings;        /**< 电机控制设置 (闭环类型/反转/反馈来源等) */
    Motor_Controller_s motor_controller;           /**< 电机控制器 (三环 PID + 前馈指针) */

    FDCAN_Instance *motor_fdcan_instance;          /**< FDCAN 实例指针 (同时处理收发) */

    Motor_Type_e motor_type;                       /**< 电机型号 (DM4310/DM4340/DM6006/DM8009) */
    Motor_Working_Type_e stop_flag;                /**< 启停标志 (MOTOR_STOP / MOTOR_ENABLED) */
    DM_Control_Mode_e control_mode;                /**< 当前控制模式 (决定打包哪种帧格式) */
    DM_MIT_Params_s mit_params;                    /**< MIT 模式参数 (KP/KD/P_MAX/V_MAX/T_MAX) */
    DM_MIT_Profile_s mit_profile;                  /**< MIT 模式运动配置 (梯形速度限制) */

    Daemon_Instance *daemon;                       /**< 守护进程实例 (检测电机离线) */
    uint32_t feed_cnt;                             /**< 喂狗计数器 (DWT 时间戳) */
    float    dt;                                   /**< 距上次反馈的时间间隔 (秒) */
    int16_t  last_set;                             /**< 上一次的控制输出值 (调试用) */
    uint8_t  feedback_initialized;                 /**< 反馈初始化完成标志 */
    uint8_t  feedback_updated;                     /**< 本次控制周期是否收到新反馈 */
} DMMotor_Instance;

/**
 * @brief 达妙电机调试观测结构
 * @note  全局单例 g_dm_motor_debug，可在调试器中直接查看。
 *        记录最后一次操作的详细信息，帮助排查通信和控制问题。
 */
typedef struct {
    /* ---- 统计计数 ---- */
    uint32_t init_ok_count;       /**< 初始化成功计数 */
    uint32_t init_fail_count;     /**< 初始化失败计数 */
    uint32_t decode_count;        /**< 反馈解码次数 (CAN 中断中递增) */
    uint32_t control_count;       /**< 控制循环次数 (DMMotorControl 中递增) */

    /* ---- 最后一次初始化信息 ---- */
    uint8_t  last_init_stage;     /**< 初始化失败时的阶段号:
                                      0=成功, 1=config为空, 2=超出数量,
                                      3=malloc失败, 4=FDCAN注册中,
                                      5=FDCAN注册失败, 6=Daemon注册失败 */
    uint8_t  last_control_mode;   /**< 最后一次设置的控制模式 */
    uint16_t last_rx_id;          /**< 最后一次注册的 RX CAN ID (MasterID) */
    uint16_t last_tx_id;          /**< 最后一次注册的 TX CAN ID (SlaveID) */

    /* ---- 最后一次反馈数据 ---- */
    uint16_t last_position_raw;   /**< 最后一次原始位置 */
    int16_t  last_velocity_raw;   /**< 最后一次原始速度 */
    int16_t  last_torque_raw;     /**< 最后一次原始力矩 */
    uint8_t  last_temperature;    /**< 最后一次温度 */
    uint8_t  reserved;            /**< 对齐保留 */

    /* ---- 最后一次物理量 ---- */
    float last_position_rad;      /**< 最后一次位置 (弧度) */
    float last_velocity_rad_s;    /**< 最后一次速度 (rad/s) */
    float last_total_angle;       /**< 最后一次累计角度 (度) */
    float last_control_ref;       /**< 最后一次控制参考值 (pid_ref) */
    int16_t last_control_set;     /**< 最后一次控制输出值 */

    /* ---- CAN 发送调试 ---- */
    uint16_t last_tx_std_id;      /**< 最后一次发送的 CAN ID */
    uint8_t  last_tx_ok;          /**< 最后一次发送是否成功 (1=成功, 0=失败) */
    uint32_t tx_ok_count;         /**< 发送成功计数 */
    uint32_t tx_fail_count;       /**< 发送失败计数 */

    /* ---- 最后一次收发原始数据 (8字节) ---- */
    uint8_t last_tx_data[8];      /**< 最后一次发送的控制帧原始数据 */
    uint8_t last_rx_data[8];      /**< 最后一次接收的反馈帧原始数据 */
} DMMotor_Debug_s;

/** @brief 全局调试实例 (在 damiao_motor.c 中定义) */
extern volatile DMMotor_Debug_s g_dm_motor_debug;


/* ================================== 公有 API 声明 ================================== */

/**
 * @brief   初始化并注册一个达妙智能电机
 *
 * @param   config  电机初始化结构体指针,包含:
 *                  - motor_type:             电机型号 (DM4310/DM4340/DM6006/DM8009)
 *                  - can_init_config:        CAN 配置 (fdcan_handle, tx_id=SlaveID, rx_id=MasterID, use_canfd=0)
 *                  - controller_param_init_config: 三环 PID 参数 (angle/speed/current)
 *                  - controller_setting_init_config: 控制设置 (闭环类型/反转/反馈来源/角度模式等)
 *
 * @return  成功返回电机实例指针, 失败返回 NULL (可通过 g_dm_motor_debug.last_init_stage 查看失败原因)
 *
 * @note    初始化流程:
 *          1. 参数校验 → 2. malloc 实例 → 3. 拷贝控制设置 → 4. 初始化三环 PID
 *          5. 设置默认 MIT 参数 → 6. 注册 FDCAN 实例 → 7. 注册守护进程 → 8. 使能电机 → 9. 存入全局数组
 *
 *          传参推荐使用 C99 指定初始化器风格:
 *          Motor_Init_Config_s cfg = {
 *              .motor_type = DM4340,
 *              .can_init_config = { .fdcan_handle = &hfdcan1, .tx_id = 0x01, .rx_id = 0x11 },
 *              .controller_param_init_config = { ... },
 *              .controller_setting_init_config = { ... },
 *          };
 *
 * @warning can_init_config 中的 tx_id 即为电机的 SlaveID, rx_id 即为 MasterID。
 *          确保这些 ID 与达妙上位机中设置的一致。
 *          确保同一路 CAN 上没有两个电机使用相同的 RX ID。
 */
DMMotor_Instance *DMMotorInit(Motor_Init_Config_s *config);

/**
 * @brief   使能电机 (允许控制输出)
 * @param   motor  电机实例指针
 * @note    调用后电机 stop_flag 置为 MOTOR_ENABLED,
 *          DMMotorControl 中会正常发送控制帧 (Byte7 = 0xFC)
 */
void DMMotorEnable(DMMotor_Instance *motor);

/**
 * @brief   停止电机 (关闭输出)
 * @param   motor  电机实例指针
 * @note    调用后电机 stop_flag 置为 MOTOR_STOP,
 *          DMMotorControl 中会发送失能帧 (Byte7 = 0xFD, 电机亮红灯)
 *          这会让电机彻底失能，而非仅发零力矩。
 */
void DMMotorStop(DMMotor_Instance *motor);

/**
 * @brief   切换反馈数据来源 (小陀螺模式常用)
 * @param   motor  电机实例指针
 * @param   loop   要切换反馈来源的控制闭环 (ANGLE_LOOP 或 SPEED_LOOP)
 * @param   type   目标反馈模式 (MOTOR_FEED 使用电机自身反馈, OTHER_FEED 使用外部数据)
 * @param   ptr    外部反馈数据的浮点指针 (type == OTHER_FEED 时有效)
 * @note    例如将角度反馈来源从电机编码器切换为 IMU 数据:
 *          DMMotorChangeFeed(motor, ANGLE_LOOP, OTHER_FEED, &imu_angle);
 */
void DMMotorChangeFeed(DMMotor_Instance *motor, Closeloop_Type_e loop,
                        Feedback_Source_e type, float *ptr);

/**
 * @brief   切换最外层闭环类型
 * @param   motor       电机实例指针
 * @param   outer_loop  新的外层闭环类型 (ANGLE_LOOP / SPEED_LOOP / CURRENT_LOOP)
 * @note    调用后 DMMotorControl 中的 PID 串级目标会相应改变,
 *          但不会改变 close_loop_type (哪些环参与计算)。
 *          例如从角度环切换到速度环:
 *          DMMotorOuterLoop(motor, SPEED_LOOP);
 *          之后 DMMotorSetRef(motor, 100) 就表示目标速度 100 度/秒。
 */
void DMMotorOuterLoop(DMMotor_Instance *motor, Closeloop_Type_e outer_loop);

/**
 * @brief   设定电机控制参考值
 * @param   motor  电机实例指针
 * @param   ref    参考值,含义取决于当前外层闭环类型:
 *                - ANGLE_LOOP:   目标角度 (度)
 *                - SPEED_LOOP:   目标角速度 (度/秒)
 *                - CURRENT_LOOP: 目标力矩 (由 t_max 定义的单位)
 * @note    应用层可以忽略底层闭环细节,将此函数视为"设定目标值"即可。
 *          本函数仅写入 pid_ref,实际的 PID 计算在 DMMotorControl() 中完成。
 */
void DMMotorSetRef(DMMotor_Instance *motor, float ref);

/**
 * @brief   重置电机角度零点
 * @param   motor  电机实例指针
 * @note    将当前角度锁定为新的零点:
 *          - total_position_offset 更新为当前 total_position_raw
 *          - total_angle 归零
 *          - total_angle_raw 归零
 *          - total_round 归零
 *          调用后 measure.total_angle 将从 0 开始累计。
 */
void DMMotorReset(DMMotor_Instance *motor);

/**
 * @brief   设置电机的控制模式
 * @param   motor  电机实例指针
 * @param   mode   控制模式 (DM_MODE_MIT / DM_MODE_POS_VEL / DM_MODE_VEL)
 * @note    此函数修改 FDCAN_Instance 的 tx_id 以匹配模式偏移:
 *          - 从 MIT 模式切换到 Pos-Vel 模式: tx_id 增加 0x100
 *          - 从 MIT 模式切换到 Vel 模式:     tx_id 增加 0x200
 *          必须在电机失能状态下通过达妙上位机先切换好电机内部模式,
 *          然后再调用此函数告知驱动使用对应的协议格式。
 *
 * @warning 此函数不切换电机内部的模式！电机模式的切换必须在达妙上位机中完成！
 */
void DMMotorSetControlMode(DMMotor_Instance *motor, DM_Control_Mode_e mode);

/**
 * @brief   设置 MIT 模式的 KP/KD 参数
 * @param   motor  电机实例指针
 * @param   kp     位置刚度 KP (0.0 ~ 500.0, 典型值 0.5 ~ 10.0)
 * @param   kd     速度阻尼 KD (0.0 ~ 100.0, 典型值 0.1 ~ 1.0)
 * @note    仅 MIT 模式有效。运行时调用可动态调节控制刚度。
 *          KP=0 且 KD=0 时电机无阻力 (力矩控制模式)。
 */
void DMMotorSetMITParams(DMMotor_Instance *motor, float kp, float kd);

/**
 * @brief 设置 MIT 模式的速度/加速度限制 (用于梯形速度控制)
 * @param motor          电机实例指针
 * @param max_vel_rad_s  最大角速度 (rad/s)
 * @param max_acc_rad_s2 最大角加速度 (rad/s^2)
 */
void DMMotorSetMITProfile(DMMotor_Instance *motor, float max_vel_rad_s, float max_acc_rad_s2);

/**
 * @brief   发送模式配置命令到电机 (通过 CAN ID 0x7FF)
 * @param   motor  电机实例指针
 * @note    电机使能后必须调用此函数,告知电机当前工作的控制模式。
 *          发送到 CAN ID 0x7FF 的配置帧: [ID_L][ID_H][0x55][0x0A][mode][0][0][0]
 *          mode: 1=MIT, 2=位置速度, 3=速度, 4=力矩
 */
void DMSendModeConfig(DMMotor_Instance *motor);

/**
 * @brief   达妙电机总控制函数
 * @note    遍历所有已注册的达妙电机实例,执行:
 *          1. 检查是否有新反馈 (feedback_updated)
 *          2. 运行三环 PID 串级计算 (ANGLE → SPEED → CURRENT/TORQUE)
 *          3. 根据 control_mode 打包控制帧:
 *             - MIT:     pos_des + vel_des + KP + KD + 使能
 *             - Pos-Vel: pos_des + vel_des + 使能
 *             - Vel:     vel_des + 使能
 *          4. 通过 FDCAN 直接发送控制帧
 *          5. 清零 feedback_updated,等待下次反馈触发
 *
 *          此函数需要在 RTOS 任务中周期性调用,推荐频率 1kHz (1ms 周期)。
 *          参考 DJI 的做法:
 *          void DMMotor_TaskFunc(void *argument) {
 *              for(;;) { DMMotorControl(); osDelay(1); }
 *          }
 *
 * @warning 此函数必须在 FreeRTOS 任务中调用,不能在中断上下文中调用!
 *          因为内部使用了 PIDCalculate (依赖 DWT 计时) 和 HAL CAN 发送。
 *
 * @note    关于 PID 输出如何映射到控制帧:
 *
 *          MIT 模式下:
 *            ANGLE_LOOP 的 PID 输出 → position_des  (uint16, 映射到 ±p_max)
 *            SPEED_LOOP 的 PID 输出 → velocity_des  (uint16, 映射到 ±v_max)
 *            KP/KD 使用 mit_params 中的设定值 (固定参数,不从 PID 产生)
 *            电机的 CURRENT_LOOP 输出目前未直接映射到 MIT 帧的 t_ff 字段
 *
 *          Pos-Vel 模式下:
 *            ANGLE_LOOP 的 PID 输出 → position_des (uint16)
 *            SPEED_LOOP 的 PID 输出 → velocity_des (uint16)
 *            (电机内部完成电流环)
 *
 *          Vel 模式下:
 *            SPEED_LOOP 的 PID 输出 → velocity_des (uint16)
 *            (电机内部完成速度环和电流环)
 */
void DMMotorControl(void);

#endif /* DAMIAO_MOTOR_H */
