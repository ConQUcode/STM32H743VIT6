/**
 * @file damiao_motor.c
 * @brief  达妙(Damiao)智能关节电机驱动实现
 * @version 1.0
 * @date    2026-05-03
 *
 * @note
 * ==== 设计总览 ====
 *
 * 本驱动完全对标 dji_motor.c 的架构和接口风格,主要差异在于 CAN 通信拓扑:
 *   - DJI:  6 个公共发送实例 (sender_assignment) + N 个接收实例,4 电机共享 1 帧
 *   - 达妙: N 个独立 FDCAN_Instance (每个同时收发),1 电机 1 帧
 *
 * 整体数据流:
 *   CAN RX 中断
 *     → FDCANFIFOxCallback()
 *       → can_module_callback = DecodeDMMotor()  // 解析反馈帧
 *         → 更新 measure (位置/速度/力矩/温度)
 *         → 多圈累计
 *         → 低通滤波
 *         → DaemonReload()  // 喂狗
 *         → feedback_updated = 1
 *
 *   RTOS 任务 (1kHz)
 *     → DMMotorControl()
 *       → 检查 feedback_updated
 *       → 三环 PID 串级计算 (ANGLE → SPEED → CURRENT/TORQUE)
 *       → 根据 control_mode 打包控制帧
 *       → DMSendFrame() → HAL_FDCAN_AddMessageToTxFifoQ()
 *       → feedback_updated = 0
 *
 * 参考:
 *   - 达妙电机 CAN 协议 (详见 damiao_motor.h 头部注释)
 *   - DJI 电机驱动 (dji_motor.c) 的架构模式
 */

#include "dm_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "string.h"
#include "stdlib.h"
#include <math.h>

/* =========================== 私有(static)函数声明 ============================= */

/**
 * @brief FDCAN 接收回调 —— 解析达妙电机反馈帧
 * @param fdcan_instance  触发回调的 FDCAN 实例
 * @note  在 CAN RX 中断上下文中被调用,需要快速执行完毕。
 *        主要工作: 解析 8 字节反馈 → 换算物理量 → 滤波 → 多圈累计 → 喂狗。
 */
static void DecodeDMMotor(FDCAN_Instance *fdcan_instance);

/**
 * @brief 守护进程离线回调 —— 电机断连时自动停止
 * @param motor_ptr  DMMotor_Instance 指针 (通过 void* 传递)
 * @note  当 daemon 的 temp_count 递减到 0 时,说明电机在 reload_count 个周期内无反馈,
 *        此时自动将 stop_flag 设为 MOTOR_STOP,防止失控。
 */
static void DMMotorLostCallback(void *motor_ptr);

/**
 * @brief 通过 HAL 库直接发送一帧 CAN 报文
 * @param instance  FDCAN 实例指针 (tx_buff 中已填充好 8 字节数据)
 * @return 1=发送成功, 0=发送失败
 * @note  参考 DJISendFrameDirect() 的实现,直接构造 FDCAN_TxHeaderTypeDef 后调用
 *        HAL_FDCAN_AddMessageToTxFifoQ(),不经过 FDCANTransmit() 的超时等待逻辑。
 */
static uint8_t DMSendFrame(FDCAN_Instance *instance);

/**
 * @brief 打包 MIT 模式控制帧 (8 字节)
 * @param motor       电机实例
 * @param angle_out   角度环 PID 输出 (度),映射为 position_des
 * @param speed_out   速度环 PID 输出 (度/秒),映射为 velocity_des
 * @note  帧格式: [posH][posL][velH][velL][KPH][KPL][KDH][KDL=0xFC]
 *        KP/KD 取自 motor->mit_params (固定参数,非 PID 输出)
 */
static void DMPackMITFrame(DMMotor_Instance *motor, float pos, float vel, float torque);
static void DMPackMITFrameWithGains(DMMotor_Instance *motor, float pos, float vel, float torque,
                                    float kp, float kd);

/**
 * @brief 打包位置速度模式控制帧 (8 字节)
 * @param motor       电机实例
 * @param angle_out   角度环 PID 输出 (度),映射为 position_des
 * @param speed_out   速度环 PID 输出 (度/秒),映射为 velocity_des
 * @note  帧格式: [posH][posL][velH][velL][0][0][0][0xFC]
 *        位置和速度均为 uint16,不包含 KP/KD。
 */
static void DMPackPosVelFrame(DMMotor_Instance *motor, float angle_out, float speed_out);

/**
 * @brief 打包自定义位置速度模式控制帧 (8 字节)
 * @param motor   电机实例
 * @param pos     目标位置 (rad)
 * @param vel     速度限制 (rad/s)
 * @note  帧格式: [p_des (4字节 float, 小端)][v_des (4字节 float, 小端)]
 */
static void DMPackPosVelCustomFrame(DMMotor_Instance *motor, float pos, float vel);

static void DMPackPosVelCustomFrame(DMMotor_Instance *motor, float pos, float vel)
{
    FDCAN_Instance *fdcan = motor->motor_fdcan_instance;
    
    /* 采用 memcpy 以保证浮点数到字节数组的正确转换 (小端) */
    memcpy(&fdcan->tx_buff[0], &pos, 4);
    memcpy(&fdcan->tx_buff[4], &vel, 4);
}

/**
 * @brief 打包速度模式控制帧 (8 字节)
 * @param motor       电机实例
 * @param speed_out   速度环 PID 输出 (度/秒),映射为 velocity_des
 * @note  帧格式: [velH][velL][0][0][0][0][0][0xFC]
 *        只有速度字段,其余保留。
 */
static void DMPackVelFrame(DMMotor_Instance *motor, float speed_out);

/**
 * @brief 判断当前控制配置是否需要等待新反馈
 * @param setting  电机控制设置
 * @return 1=需要新反馈, 0=开环或不需要
 * @note  开环模式不应被 feedback_updated 绑死,否则改了参考值也无法发新指令。
 *        逻辑与 DJIMotorControlNeedsFreshFeedback() 一致。
 */
static uint8_t DMControlNeedsFreshFeedback(const Motor_Control_Setting_s *setting);

/**
 * @brief 根据电机型号设置默认 MIT 参数 (p_max, v_max, t_max, kp, kd)
 * @param motor  电机实例
 * @note  在 DMMotorInit 中调用。KP/KD 使用默认值,后续可通过 DMMotorSetMITParams 修改。
 */
static void DMSetDefaultMITParams(DMMotor_Instance *motor);
static void DMInitMITProfile(DMMotor_Instance *motor);

/**
 * @brief 将原始 uint 映射为物理浮点值
 * @param raw  原始值 (0 ~ 2^bits-1)
 * @param min  物理量最小值
 * @param max  物理量最大值
 * @param bits 位宽 (16=位置, 12=速度/KP/KD/力矩)
 */
static float DMUintToFloat(uint16_t raw, float min, float max, uint8_t bits);

/**
 * @brief 将物理浮点值映射为 uint
 * @param val  物理值
 * @param min  物理量最小值
 * @param max  物理量最大值
 * @param bits 位宽
 */
static uint16_t DMFloatToUint(float val, float min, float max, uint8_t bits);

/**
 * @brief 将原始位置值 (0~65535) 转换为单圈角度 (度)
 * @param pos_raw  原始位置 (uint16 → float)
 * @return 角度 (度), 范围 0.0 ~ 360.0
 * @note  角度 = pos_raw × (360 / 65536)
 */
static float DMRawPosToAngleDeg(float pos_raw);

/**
 * @brief 将原始速度值转换为角速度 (度/秒)
 * @param vel_raw  原始速度 (int16)
 * @param v_max    速度最大值 (rad/s)
 * @return 角速度 (度/秒)
 * @note  角速度 = (vel_raw / 65535) × v_max × (180/π)
 */
static float DMRawVelToAps(int16_t vel_raw, float v_max);

/**
 * @brief 将角度归一化到 [0, 360) 范围
 * @param angle_deg  任意角度值 (度)
 * @return 归一化后的角度 (度)
 */
static float DMNormalizeAngleDeg360(float angle_deg);

/**
 * @brief 计算距离当前总角度最近的等效单圈目标
 * @param current_total  当前总角度 (度, 含多圈累计)
 * @param target         目标单圈角度 (度, 用户输入, 如 90°)
 * @return 调整后的目标总角度 (度)
 * @note  用于单圈最短路径角度控制 (MOTOR_ANGLE_MODE_SINGLE_TURN)。
 *        例如当前 350°,目标 10°,返回 370° (顺时针转 20°) 而非 10° (逆时针转 340°)。
 *        逻辑与 DJICalcNearestSingleTurnTargetDeg() 一致。
 */
static float DMCalcNearestSingleTurnTargetDeg(float current_total, float target);

/**
 * @brief MIT 模式控制入口 (位置外环 + 速度梯形限制)
 */
static void DMControlMIT(DMMotor_Instance *motor);


/* ============================= 全局变量 ============================= */

/** @brief 下一次注册的数组索引 (静态,文件内全局) */
static uint8_t dm_idx = 0;

/** @brief 达妙电机实例指针数组 (最多 DM_MOTOR_CNT 个) */
static DMMotor_Instance *dm_motor_instances[DM_MOTOR_CNT] = {NULL};

/**
 * @brief 全局调试观测实例
 * @note  可在调试器 (Keil/J-Link) 中直接查看,实时了解:
 *        - 初始化成功/失败次数和失败阶段
 *        - 最近一次反馈的原始值和物理量
 *        - CAN 发送成功/失败计数
 *        - 最近一次收发的原始 CAN 帧数据 (8 字节)
 */
volatile DMMotor_Debug_s g_dm_motor_debug = {0};


/* ========================== 私有(static)函数实现 =========================== */

/* ---------- 工具函数 ---------- */

static float DMUintToFloat(uint16_t raw, float min, float max, uint8_t bits)
{
    /* 线性映射: raw ∈ [0, 2^bits-1] → float ∈ [min, max] */
    float span   = max - min;
    float offset = min;
    return ((float)raw) * span / ((float)((1 << bits) - 1)) + offset;
}

static uint16_t DMFloatToUint(float val, float min, float max, uint8_t bits)
{
    /* 线性映射: float ∈ [min, max] → uint16 ∈ [0, 2^bits-1] */
    float span   = max - min;
    float offset = min;

    if (val < min)
        val = min;
    if (val > max)
        val = max;

    return (uint16_t)((val - offset) * ((float)((1 << bits) - 1)) / span);
}

static float DMRawPosToAngleDeg(float pos_raw)
{
    /* 16位编码器 → 角度: raw/65536 × 360° */
    return pos_raw * (360.0f / 65536.0f);
}

static float DMRawVelToAps(int16_t vel_raw, float v_max)
{
    /* 原始速度 → rad/s → 度/秒 (× 180/π ≈ 57.29578) */
    return (float)vel_raw / 65535.0f * v_max * RAD_2_DEGREE;
}

static float DMNormalizeAngleDeg360(float angle_deg)
{
    /* 用 fmodf 取余数,负数结果 +360 使其落在 [0, 360) */
    float a = fmodf(angle_deg, 360.0f);
    if (a < 0.0f)
        a += 360.0f;
    return a;
}

static float DMCalcNearestSingleTurnTargetDeg(float current_total, float target)
{
    float cur = DMNormalizeAngleDeg360(current_total);  /* 当前单圈角度 */
    float tar = DMNormalizeAngleDeg360(target);          /* 目标单圈角度 */
    float delta = tar - cur;                             /* 差值 */

    /* 选择最短旋转方向: >180° 则减一圈, <-180° 则加一圈 */
    if (delta > 180.0f)
        delta -= 360.0f;
    else if (delta < -180.0f)
        delta += 360.0f;

    /* 返回: 当前总角度 + 最短增量 */
    return current_total + delta;
}

static void DMControlMIT(DMMotor_Instance *motor)
{
    Motor_Control_Setting_s *setting;
    Motor_Controller_s *ctrl;
    DM_Motor_Measure_s *measure;
    float target_deg;
    float target_rad;
    float current_deg;
    float pos_cmd;
    float vel_cmd;
    float error;
    float abs_error;
    float dir;
    float max_vel;
    float max_acc;
    float brake_dist;
    float dt;

    if (motor == NULL)
        return;

    setting = &motor->motor_settings;
    ctrl = &motor->motor_controller;
    measure = &motor->measure;

    if (motor->feedback_initialized == 0U) {
        /* 未收到反馈前: KP=0 避免位置跳变, KD 使用实际值提供速度阻尼,
         * 非零增益确保电机回复反馈帧, 打破鸡生蛋死锁 */
        DMPackMITFrameWithGains(motor, 0.0f, 0.0f, 0.0f,
                                0.0f, motor->mit_params.kd);
        return;
    }

    /* 目标角度输入使用度制,保持与应用层一致 */
    target_deg = ctrl->pid_ref;
    if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
        target_deg = -target_deg;

    /* 目标转弧度并归一化到 [0, 2π), 对齐位置字段的单圈映射 */
    target_rad = target_deg * DEGREE_2_RAD;
    target_rad = fmodf(target_rad, 2.0f * PI);
    if (target_rad < 0.0f) target_rad += 2.0f * PI;
    target_deg = target_rad * RAD_2_DEGREE;

    if (setting->angle_mode == MOTOR_ANGLE_MODE_SINGLE_TURN)
    {
        current_deg = measure->position_rad * RAD_2_DEGREE;
        if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            current_deg = -current_deg;
        target_deg = DMCalcNearestSingleTurnTargetDeg(current_deg, target_deg);
        target_rad = target_deg * DEGREE_2_RAD;
        LIMIT_MIN_MAX(target_rad, -motor->mit_params.p_max, motor->mit_params.p_max);
    }

    dt = motor->dt;
    if (dt <= 0.0f || dt > 0.1f)
        dt = 0.001f;

    if (motor->mit_profile.initialized == 0U) {
        motor->mit_profile.pos_rad = measure->position_rad;
        motor->mit_profile.vel_rad_s = measure->speed_aps * DEGREE_2_RAD;
        motor->mit_profile.initialized = 1U;
    }

    pos_cmd = motor->mit_profile.pos_rad;
    vel_cmd = motor->mit_profile.vel_rad_s;
    max_vel = motor->mit_profile.max_vel_rad_s;
    max_acc = motor->mit_profile.max_acc_rad_s2;

    if (max_vel < 0.0f)
        max_vel = -max_vel;
    if (max_acc < 0.0f)
        max_acc = -max_acc;
    if (max_acc < 1e-6f)
        max_acc = 1e-6f;

    error = target_rad - pos_cmd;
    dir = (error >= 0.0f) ? 1.0f : -1.0f;
    abs_error = (error >= 0.0f) ? error : -error;

    brake_dist = (vel_cmd * vel_cmd) / (2.0f * max_acc);
    if (abs_error <= brake_dist)
        vel_cmd -= dir * max_acc * dt;
    else
        vel_cmd += dir * max_acc * dt;

    if (vel_cmd > max_vel)
        vel_cmd = max_vel;
    else if (vel_cmd < -max_vel)
        vel_cmd = -max_vel;

    pos_cmd += vel_cmd * dt;
    if ((dir > 0.0f && pos_cmd > target_rad) || (dir < 0.0f && pos_cmd < target_rad)) {
        pos_cmd = target_rad;
        vel_cmd = 0.0f;
    }

    motor->mit_profile.pos_rad = pos_cmd;
    motor->mit_profile.vel_rad_s = vel_cmd;

    DMPackMITFrame(motor, pos_cmd, vel_cmd, 0.0f);

    g_dm_motor_debug.last_control_ref = ctrl->pid_ref;
    g_dm_motor_debug.last_control_set = (int16_t)(vel_cmd * RAD_2_DEGREE);
}


/* ---------- 控制帧打包函数 ---------- */

static uint8_t DMControlNeedsFreshFeedback(const Motor_Control_Setting_s *setting)
{
    if (setting == NULL)
        return 0U;

    /* 位置环 (外层) 需要位置反馈 */
    if ((setting->close_loop_type & ANGLE_LOOP) &&
        setting->outer_loop_type == ANGLE_LOOP)
        return 1U;

    /* 速度环 (外层或内层) 需要速度反馈 */
    if ((setting->close_loop_type & SPEED_LOOP) &&
        (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        return 1U;

    /* 电流环/力矩环需要力矩反馈 */
    if (setting->close_loop_type & (CURRENT_LOOP | TORQUE_LOOP))
        return 1U;

    /* 开环控制: 不需要等待反馈 */
    return 0U;
}

static void DMSetDefaultMITParams(DMMotor_Instance *motor)
{
    DM_MIT_Params_s *p = &motor->mit_params;

    /* KP/KD 使用默认值,后续可通过 DMMotorSetMITParams() 运行时修改 */
    p->kp = DM_MIT_DEFAULT_KP;
    p->kd = DM_MIT_DEFAULT_KD;

    /* 根据电机型号设置物理量范围
     * 这些值必须与达妙上位机中设置的 P_MAX/V_MAX/T_MAX 保持一致 */
    switch (motor->motor_type) {
        case DM4310:
            p->p_max = DM4310_P_MAX;
            p->v_max = DM4310_V_MAX;
            p->t_max = DM4310_T_MAX;
            break;
        case DM4340:
            p->p_max = DM4340_P_MAX;
            p->v_max = DM4340_V_MAX;
            p->t_max = DM4340_T_MAX;
            break;
        case DM6006:
            p->p_max = DM6006_P_MAX;
            p->v_max = DM6006_V_MAX;
            p->t_max = DM6006_T_MAX;
            break;
        case DM8009:
            p->p_max = DM8009_P_MAX;
            p->v_max = DM8009_V_MAX;
            p->t_max = DM8009_T_MAX;
            break;
        default:
            /* 未知型号: 使用 DM4340 的参数作为安全默认值 */
            p->p_max = DM4340_P_MAX;
            p->v_max = DM4340_V_MAX;
            p->t_max = DM4340_T_MAX;
            break;
    }
}

static void DMInitMITProfile(DMMotor_Instance *motor)
{
    if (motor == NULL)
        return;

    motor->mit_profile.max_vel_rad_s = motor->mit_params.v_max;
    motor->mit_profile.max_acc_rad_s2 = motor->mit_params.v_max * 2.0f;
    motor->mit_profile.pos_rad = 0.0f;
    motor->mit_profile.vel_rad_s = 0.0f;
    motor->mit_profile.initialized = 0U;
}

/**
 * @brief 打包 MIT 模式控制帧 (12-bit 紧凑格式)
 *
 * MIT 帧格式 (8 字节):
 *   [posH][posL][velH(8)][velL|KPH(4)][KPL(8)][KDH(8)][KDL|torH(4)][torL(8)]
 *   位宽: pos=16, vel=12, KP=12, KD=12, torque=12
 */
static void DMPackMITFrame(DMMotor_Instance *motor, float pos, float vel, float torque)
{
    DM_MIT_Params_s *p = &motor->mit_params;

    DMPackMITFrameWithGains(motor, pos, vel, torque, p->kp, p->kd);
}

static void DMPackMITFrameWithGains(DMMotor_Instance *motor, float pos, float vel, float torque,
                                    float kp, float kd)
{
    DM_MIT_Params_s *p = &motor->mit_params;
    FDCAN_Instance *fdcan = motor->motor_fdcan_instance;
    uint16_t pos_u, vel_u, kp_u, kd_u, tor_u;

    pos_u = DMFloatToUint(pos, -p->p_max, p->p_max, DM_MIT_POS_BITS);
    vel_u = DMFloatToUint(vel, -p->v_max, p->v_max, DM_MIT_VEL_BITS);
    kp_u  = DMFloatToUint(kp, 0.0f, DM_MIT_KP_MAX, DM_MIT_KP_BITS);
    kd_u  = DMFloatToUint(kd, 0.0f, DM_MIT_KD_MAX, DM_MIT_KD_BITS);
    tor_u = DMFloatToUint(torque, -p->t_max, p->t_max, DM_MIT_TOR_BITS);

    fdcan->tx_buff[0] = (uint8_t)(pos_u >> 8);
    fdcan->tx_buff[1] = (uint8_t)(pos_u & 0xFF);
    fdcan->tx_buff[2] = (uint8_t)(vel_u >> 4);
    fdcan->tx_buff[3] = (uint8_t)(((vel_u & 0x0F) << 4) | (kp_u >> 8));
    fdcan->tx_buff[4] = (uint8_t)(kp_u & 0xFF);
    fdcan->tx_buff[5] = (uint8_t)(kd_u >> 4);
    fdcan->tx_buff[6] = (uint8_t)(((kd_u & 0x0F) << 4) | (tor_u >> 8));
    fdcan->tx_buff[7] = (uint8_t)(tor_u & 0xFF);
}

/**
 * @brief 打包位置速度模式控制帧
 *
 * Pos-Vel 帧格式 (8 字节, 大端):
 *   Byte0-1: 位置期望值   uint16
 *   Byte2-3: 速度期望值   uint16
 *   Byte4-6: 保留 (0x00)
 *   Byte7:   使能标志 (0xFC)
 *
 * @note  此模式下电机内部执行三环串级,驱动只需给目标位置和目标速度。
 *        位置/速度使用相同的 uint16 映射方式 (与 MIT 一致)。
 */
/**
 * @brief 打包位置速度模式控制帧 (8 字节)
 * @param motor   电机实例
 * @param pos     目标位置 (rad)
 * @param vel     速度限制 (rad/s)
 * @note  帧格式: [p_des (4字节 float, 小端)][v_des (4字节 float, 小端)]
 */
static void DMPackPosVelFrame(DMMotor_Instance *motor, float pos, float vel)
{
    FDCAN_Instance *fdcan = motor->motor_fdcan_instance;
    
    /* 采用 memcpy 以保证浮点数到字节数组的正确转换 (小端) */
    memcpy(&fdcan->tx_buff[0], &pos, 4);
    memcpy(&fdcan->tx_buff[4], &vel, 4);
}

/**
 * @brief 打包速度模式控制帧
 *
 * Vel 帧格式 (8 字节, 大端):
 *   Byte0-1: 速度期望值   uint16
 *   Byte2-6: 保留 (0x00)
 *   Byte7:   使能标志 (0xFC)
 */
static void DMPackVelFrame(DMMotor_Instance *motor, float speed_out)
{
    DM_MIT_Params_s *p = &motor->mit_params;
    FDCAN_Instance *fdcan = motor->motor_fdcan_instance;
    uint16_t vel_des;

    vel_des = DMFloatToUint(speed_out, -p->v_max, p->v_max, 16);

    fdcan->tx_buff[0] = (uint8_t)(vel_des >> 8);
    fdcan->tx_buff[1] = (uint8_t)(vel_des & 0xFF);
    fdcan->tx_buff[2] = 0;
    fdcan->tx_buff[3] = 0;
    fdcan->tx_buff[4] = 0;
    fdcan->tx_buff[5] = 0;
    fdcan->tx_buff[6] = 0;
    fdcan->tx_buff[7] = 0xFC; /* 使能 */
}


/* ---------- CAN 发送函数 ---------- */

/**
 * @brief 直接通过 HAL 库发送 CAN 控制帧
 *
 * @note  与 DJI 的 DJISendFrameDirect() 采用相同的直发策略:
 *        跳过 FDCANTransmit() 的超时等待逻辑,直接调用 HAL_FDCAN_AddMessageToTxFifoQ()。
 *        这样做的好处是:
 *        1. 避免在 1kHz 控制循环中因 FIFO 满而阻塞
 *        2. 发送失败时仅记录错误,不阻塞后续电机的控制
 *
 *        FDCAN 配置:
 *        - 经典 CAN 模式 (FDCAN_CLASSIC_CAN)
 *        - 标准帧 ID (FDCAN_STANDARD_ID)
 *        - 8 字节数据长度 (FDCAN_DLC_BYTES_8)
 *        - 不启用比特率切换 (FDCAN_BRS_OFF)
 */
static uint8_t DMSendFrame(FDCAN_Instance *instance)
{
    FDCAN_TxHeaderTypeDef tx_header;

    if (instance == NULL || instance->fdcan_handle == NULL)
        return 0U;

    /* 构造 CAN 发送帧头 */
    tx_header.Identifier = instance->tx_id;            /* CAN ID = SlaveID + 模式偏移 */
    tx_header.IdType = FDCAN_STANDARD_ID;              /* 标准帧 (11位ID) */
    tx_header.TxFrameType = FDCAN_DATA_FRAME;          /* 数据帧 */
    tx_header.DataLength = FDCAN_DLC_BYTES_8;          /* 8 字节数据 */
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;           /* 不启用波特率切换 */
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;            /* 经典 CAN 模式 */
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS; /* 不记录发送事件 */
    tx_header.MessageMarker = 0U;

    /* 直接放入发送 FIFO,不等待,不重试 */
    if (HAL_FDCAN_AddMessageToTxFifoQ(instance->fdcan_handle, &tx_header,
                                       instance->tx_buff) != HAL_OK) {
        g_dm_motor_debug.tx_fail_count++;
        g_dm_motor_debug.last_tx_ok = 0U;
        return 0U;
    }

    /* 发送成功: 更新调试信息 */
    g_dm_motor_debug.tx_ok_count++;
    g_dm_motor_debug.last_tx_ok = 1U;
    g_dm_motor_debug.last_tx_std_id = (uint16_t)instance->tx_id;

    /* 将最后一次发送的数据存入调试区,方便在调试器中观察 */
    g_dm_motor_debug.last_tx_data[0] = instance->tx_buff[0];
    g_dm_motor_debug.last_tx_data[1] = instance->tx_buff[1];
    g_dm_motor_debug.last_tx_data[2] = instance->tx_buff[2];
    g_dm_motor_debug.last_tx_data[3] = instance->tx_buff[3];
    g_dm_motor_debug.last_tx_data[4] = instance->tx_buff[4];
    g_dm_motor_debug.last_tx_data[5] = instance->tx_buff[5];
    g_dm_motor_debug.last_tx_data[6] = instance->tx_buff[6];
    g_dm_motor_debug.last_tx_data[7] = instance->tx_buff[7];

    return 1U;
}


/* ---------- CAN 接收回调函数 (中断上下文) ---------- */

/**
 * @brief 达妙电机 FDCAN 接收回调 —— 解析反馈帧
 *
 * @note  **此函数在 CAN RX 中断中执行,需要尽可能快速完成。**
 *
 * 反馈帧格式 (各模式通用, CAN ID = MasterID):
 *   Byte0:   电机 SlaveID (用于校验)
 *   Byte1-2: 当前位置 (uint16, 大端)  → 映射到 -p_max ~ +p_max
 *   Byte3-4: 当前速度 (int16,  大端)  → 映射到 -v_max ~ +v_max
 *   Byte5-6: 当前力矩 (int16,  大端)  → 映射到 -t_max ~ +t_max
 *   Byte7:   温度/状态信息
 *
 * 处理流程:
 *   1. 校验数据长度 (经典 CAN 至少 7 字节,不足则兜底为 8)
 *   2. 通过 fdcan_instance->id 反查电机实例
 *   3. 解析 8 字节原始数据 → 物理量 (rad, rad/s, Nm)
 *   4. 一阶低通滤波 (速度、力矩)
 *   5. 多圈累计 (半区间法检测编码器溢出)
 *   6. 更新角度制和累计角度
 *   7. 喂狗 (DaemonReload)
 *   8. feedback_updated = 1 (通知控制循环有新数据)
 *
 * 多圈累计原理 (半区间法):
 *   编码器为 16 位 (0~65535),当电机连续旋转时,编码器会周期性从 65535 翻转到 0
 *   (正转) 或从 0 翻转到 65535 (反转)。
 *   通过比较相邻两次 position_raw 的差值:
 *   - 差值 > +32768 (半区间) → 实际是负向翻转 (65535→0), total_round++
 *   - 差值 < -32768 (半区间) → 实际是正向翻转 (0→65535), total_round--
 *   这与 DJI 电机的多圈累计算法完全一致,只是分辨率从 12 位变为 16 位。
 */
static void DecodeDMMotor(FDCAN_Instance *fdcan_instance)
{
    uint8_t *rxbuff;
    DMMotor_Instance *motor;
    DM_Motor_Measure_s *measure;
    DM_MIT_Params_s *p;
    uint16_t raw_pos, raw_vel, raw_tor;
    float pos_rad, vel_rad_s, tor_nm;
    float prev_torque;

    if (fdcan_instance == NULL)
        return;

    if (fdcan_instance->rx_len < 7U) {
        if (fdcan_instance->use_canfd == 0U)
            fdcan_instance->rx_len = 8U;
        else
            return;
    }

    rxbuff = fdcan_instance->rx_buff;

    motor = (DMMotor_Instance *)fdcan_instance->id;
    if (motor == NULL)
        return;

    g_dm_motor_debug.decode_count++;

    measure = &motor->measure;
    p = &motor->mit_params;

    /* 喂狗 */
    if (motor->daemon != NULL)
        DaemonReload(motor->daemon);

    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    /* ---- 解析反馈帧 (12-bit 紧凑格式) ---- */
    measure->motor_id    = rxbuff[0] & 0x0F;
    measure->state       = (rxbuff[0] >> 4) & 0x0F;

    raw_pos = ((uint16_t)rxbuff[1] << 8) | rxbuff[2];
    raw_vel = ((uint16_t)rxbuff[3] << 4) | (rxbuff[4] >> 4);
    raw_tor = ((uint16_t)(rxbuff[4] & 0x0F) << 8) | rxbuff[5];

    measure->position_raw = raw_pos;
    measure->velocity_raw = (int16_t)raw_vel;
    measure->torque_raw   = (int16_t)raw_tor;

    /* 保存上一次位置 (float), 用于多圈累计 */
    measure->last_position_rad = measure->position_rad;

    /* 原始值 → 物理量
     * 位置字段为 [0, 2π] 单圈绝对角度映射, 非 [-p_max, +p_max] */
    pos_rad  = DMUintToFloat(raw_pos, -p->p_max, p->p_max, DM_MIT_POS_BITS);
    vel_rad_s = DMUintToFloat(raw_vel, -p->v_max, p->v_max, 12);
    tor_nm   = DMUintToFloat(raw_tor, -p->t_max, p->t_max, 12);

    measure->position_rad  = pos_rad;
    measure->velocity_rad_s = vel_rad_s;

    /* 力矩低通滤波 */
    prev_torque = measure->torque_nm;
    measure->torque_nm = (1.0f - DM_CURRENT_SMOOTH_COEF) * prev_torque +
                         DM_CURRENT_SMOOTH_COEF * tor_nm;

    /* 温度 */
    measure->temperature = rxbuff[6];  /* T_Mos */
    measure->rotor_temp  = rxbuff[7];  /* T_Rotor */

    /* ---- 角度制换算 ---- */
    measure->angle_single_round = pos_rad * RAD_2_DEGREE;

    /* 角速度 rad/s → 度/秒, 一阶低通滤波 */
    {
        float raw_speed_aps = vel_rad_s * RAD_2_DEGREE;
        measure->speed_aps = (1.0f - DM_SPEED_SMOOTH_COEF) * measure->speed_aps +
                             DM_SPEED_SMOOTH_COEF * raw_speed_aps;
    }

    /* ---- 首次初始化 ---- */
    if (motor->feedback_initialized == 0U) {
        measure->last_position_rad = pos_rad;
        measure->speed_aps = vel_rad_s * RAD_2_DEGREE;
        measure->total_round = 0;
        measure->total_angle = measure->angle_single_round;
        /* 首帧将目标锁到当前位置,避免上电回零转圈 */
        motor->motor_controller.pid_ref = measure->angle_single_round;
        motor->feedback_initialized = 1U;
        motor->feedback_updated = 1U;

        g_dm_motor_debug.last_position_raw = raw_pos;
        g_dm_motor_debug.last_velocity_raw = (int16_t)raw_vel;
        g_dm_motor_debug.last_torque_raw = (int16_t)raw_tor;
        g_dm_motor_debug.last_temperature = measure->temperature;
        g_dm_motor_debug.last_position_rad = pos_rad;
        g_dm_motor_debug.last_velocity_rad_s = vel_rad_s;
        g_dm_motor_debug.last_total_angle = measure->total_angle;
        g_dm_motor_debug.last_rx_id = (uint16_t)fdcan_instance->rx_id;
        return;
    }

    /* ---- 多圈累计 (float 半区间法) ---- */
    if (pos_rad - measure->last_position_rad > PI)
        measure->total_round--;
    else if (pos_rad - measure->last_position_rad < -PI)
        measure->total_round++;

    measure->total_angle = measure->total_round * 360.0f
                         + measure->angle_single_round
                         - measure->zero_offset;

    motor->feedback_updated = 1U;

    g_dm_motor_debug.last_position_raw = raw_pos;
    g_dm_motor_debug.last_velocity_raw = (int16_t)raw_vel;
    g_dm_motor_debug.last_torque_raw = (int16_t)raw_tor;
    g_dm_motor_debug.last_temperature = measure->temperature;
    g_dm_motor_debug.last_position_rad = pos_rad;
    g_dm_motor_debug.last_velocity_rad_s = vel_rad_s;
    g_dm_motor_debug.last_total_angle = measure->total_angle;
    g_dm_motor_debug.last_rx_id = (uint16_t)fdcan_instance->rx_id;
}


/* ---------- 守护进程离线回调 ---------- */

/**
 * @brief 电机离线时的自动停止回调
 * @note  当 daemon 的 temp_count 递减到 0 时被 DaemonTask 调用。
 *        将电机 stop_flag 设为 MOTOR_STOP,
 *        下一次 DMMotorControl 会发送失能帧 (Byte7=0xFD) 安全停止电机。
 */
static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotor_Instance *motor = (DMMotor_Instance *)motor_ptr;

    if (motor == NULL || motor->motor_fdcan_instance == NULL)
        return;

    motor->stop_flag = MOTOR_STOP;
}


/* ======================== 公有函数实现 ======================== */

/**
 * @brief 初始化达妙电机
 *
 * 完整初始化流程 (共 9 步):
 *   1. 参数校验 —— config 非空
 *   2. 数量检查 —— dm_idx < DM_MOTOR_CNT
 *   3. 分配内存 —— malloc(sizeof(DMMotor_Instance)), memset 清零
 *   4. 拷贝配置 —— motor_type, motor_settings (包含闭环类型/反转等)
 *   5. 初始化 PID —— PIDInit × 3 (angle / speed / current)
 *   6. 设置默认 MIT 参数 —— 根据 motor_type 查表
 *   7. 注册 FDCAN 实例 —— 强制 use_canfd=0, 回调=DecodeDMMotor
 *   8. 注册守护进程 —— reload_count=3 (3 个周期无反馈即判离线)
 *   9. 存入全局数组 —— dm_motor_instances[dm_idx++] = motor
 *
 * @note  任何一步失败都会:
 *        - 释放已分配资源 (free + FDCAN 回调置空)
 *        - 记录失败阶段到 g_dm_motor_debug.last_init_stage
 *        - 返回 NULL
 */
DMMotor_Instance *DMMotorInit(Motor_Init_Config_s *config)
{
    FDCAN_Init_Config_s can_config;
    DMMotor_Instance *motor;
    Daemon_Init_Config_s daemon_config;

    /* Step 1: 参数校验 */
    if (config == NULL) {
        g_dm_motor_debug.init_fail_count++;
        g_dm_motor_debug.last_init_stage = 1;  /* stage=1: config 为空 */
        return NULL;
    }

    /* Step 2: 数量上限检查 */
    if (dm_idx >= DM_MOTOR_CNT) {
        g_dm_motor_debug.init_fail_count++;
        g_dm_motor_debug.last_init_stage = 2;  /* stage=2: 超出最大数量 */
        return NULL;
    }

    /* Step 3: 分配内存并清零 */
    motor = (DMMotor_Instance *)malloc(sizeof(DMMotor_Instance));
    if (motor == NULL) {
        g_dm_motor_debug.init_fail_count++;
        g_dm_motor_debug.last_init_stage = 3;  /* stage=3: malloc 失败 */
        return NULL;
    }
    memset(motor, 0, sizeof(DMMotor_Instance));

    /* Step 4: 拷贝配置 */
    motor->motor_type     = config->motor_type;
    motor->motor_settings = config->controller_setting_init_config;
    motor->control_mode   = DM_MODE_MIT;   /* 默认 MIT 模式 */
    motor->stop_flag      = MOTOR_STOP;     /* 初始为停止状态 */

    /* Step 5: 初始化三环 PID */
    PIDInit(&motor->motor_controller.current_PID,
            &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID,
            &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID,
            &config->controller_param_init_config.angle_PID);

    /* 拷贝前馈和外部反馈指针 */
    motor->motor_controller.other_angle_feedback_ptr =
        config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr =
        config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr =
        config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr =
        config->controller_param_init_config.speed_feedforward_ptr;

    /* Step 6: 根据型号设置默认 MIT 参数 */
    DMSetDefaultMITParams(motor);
    DMInitMITProfile(motor);

    /* Step 7: 注册 FDCAN 实例 */
    can_config = config->can_init_config;
    can_config.use_canfd = 0;                          /* 强制经典 CAN */
    can_config.can_module_callback = DecodeDMMotor;    /* RX 中断回调 */
    can_config.id = motor;                             /* 实例反向指针,中断中反查 */

    g_dm_motor_debug.last_init_stage = 4;  /* stage=4: 正在注册 FDCAN */

    motor->motor_fdcan_instance = FDCANRegister(&can_config);
    if (motor->motor_fdcan_instance == NULL) {
        free(motor);
        g_dm_motor_debug.init_fail_count++;
        g_dm_motor_debug.last_init_stage = 5;  /* stage=5: FDCAN 注册失败 */
        return NULL;
    }

    /* 显式绑定回调 (防御性编程,确保中断链路正确) */
    motor->motor_fdcan_instance->can_module_callback = DecodeDMMotor;
    motor->motor_fdcan_instance->id = motor;

    /* 设置发送数据长度为 8 字节 (经典 CAN) */
    FDCANSetDataLength(motor->motor_fdcan_instance, 8);

    /* Step 8: 注册守护进程 (看门狗) */
    memset(&daemon_config, 0, sizeof(daemon_config));
    daemon_config.callback     = DMMotorLostCallback;
    daemon_config.owner_id     = motor;
    daemon_config.reload_count = 3;  /* 3 个控制周期无反馈 = 离线 */

    motor->daemon = DaemonRegister(&daemon_config);
    if (motor->daemon == NULL) {
        /* 守护进程注册失败: 清理 FDCAN 实例后退出 */
        motor->motor_fdcan_instance->can_module_callback = NULL;
        motor->motor_fdcan_instance->id = NULL;
        free(motor);
        g_dm_motor_debug.init_fail_count++;
        g_dm_motor_debug.last_init_stage = 6;  /* stage=6: Daemon 注册失败 */
        return NULL;
    }

    /* Step 9: 仅注册实例，不在驱动初始化阶段自动使能/改模式。
     * 电机模式完全依赖上位机保存结果，由应用层自行设置发送 ID 和使能。 */
    dm_motor_instances[dm_idx++] = motor;

    /* 初始化成功 */
    g_dm_motor_debug.init_ok_count++;
    g_dm_motor_debug.last_init_stage = 0;  /* stage=0: 成功 */
    g_dm_motor_debug.last_tx_id = (uint16_t)can_config.tx_id;
    g_dm_motor_debug.last_rx_id = (uint16_t)can_config.rx_id;
    g_dm_motor_debug.last_control_mode = (uint8_t)motor->control_mode;

    return motor;
}

void DMMotorEnable(DMMotor_Instance *motor)
{
    FDCAN_Instance *fdcan;

    if (motor == NULL || motor->motor_fdcan_instance == NULL)
        return;

    fdcan = motor->motor_fdcan_instance;
    motor->stop_flag = MOTOR_ENALBED;

    /* 发送使能帧: {0xFF*7, 0xFC} */
    memset(fdcan->tx_buff, 0xFF, 7);
    fdcan->tx_buff[7] = 0xFC;
    DMSendFrame(fdcan);
    memset(fdcan->tx_buff, 0, 7);


}

void DMMotorStop(DMMotor_Instance *motor)
{
    FDCAN_Instance *fdcan;

    if (motor == NULL || motor->motor_fdcan_instance == NULL)
        return;

    fdcan = motor->motor_fdcan_instance;
    motor->stop_flag = MOTOR_STOP;

    /* 发送一次性失能帧: {0xFF*7, 0xFD},电机收到后亮红灯 */
    memset(fdcan->tx_buff, 0xFF, 7);
    fdcan->tx_buff[7] = 0xFD;
    DMSendFrame(fdcan);
    memset(fdcan->tx_buff, 0, 7);  /* 恢复 */
}

void DMMotorChangeFeed(DMMotor_Instance *motor, Closeloop_Type_e loop,
                        Feedback_Source_e type, float *ptr)
{
    if (motor == NULL)
        return;

    /* 根据闭环类型设置对应的反馈来源 */
    if (loop == ANGLE_LOOP) {
        motor->motor_settings.angle_feedback_source      = type;
        motor->motor_controller.other_angle_feedback_ptr = ptr;
    } else if (loop == SPEED_LOOP) {
        motor->motor_settings.speed_feedback_source      = type;
        motor->motor_controller.other_speed_feedback_ptr = ptr;
    }
}

void DMMotorOuterLoop(DMMotor_Instance *motor, Closeloop_Type_e outer_loop)
{
    if (motor == NULL)
        return;
    motor->motor_settings.outer_loop_type = outer_loop;
}

void DMMotorSetRef(DMMotor_Instance *motor, float ref)
{
    if (motor == NULL)
        return;
    motor->motor_controller.pid_ref = ref;
    
    /* 如果是位置速度模式,同时更新内部 pos_ref */
    if (motor->control_mode == DM_MODE_POS_VEL) {
        motor->pos_ref = ref * DEGREE_2_RAD;
    }
}

void DMMotorSetPosVelRef(DMMotor_Instance *motor, float pos, float vel)
{
    if (motor == NULL)
        return;
    motor->pos_ref = pos;
    motor->vel_ref = fabsf(vel); // 速度限制始终为正
}

/**
 * @brief 重置电机角度零点
 * @note  将当前累计角度锁定为零点, total_angle 归零, total_round 归零。
 */
void DMMotorReset(DMMotor_Instance *motor)
{
    if (motor == NULL)
        return;

    motor->measure.zero_offset = motor->measure.total_angle;
    motor->measure.total_angle = 0.0f;
    motor->measure.total_round = 0;
    motor->mit_profile.pos_rad = 0.0f;
    motor->mit_profile.vel_rad_s = 0.0f;
    motor->mit_profile.initialized = 0U;
}

/**
 * @brief 设置电机控制模式 (运行时切换)
 *
 * @note  **重要**: 此函数只改变驱动侧的 CAN ID 偏移和打包方式,
 *        不会切换电机内部的模式!
 *        电机内部模式的切换必须在达妙 Windows 上位机中设置并保存到 Flash。
 *
 * CAN ID 偏移规则:
 *   - MIT 模式:     tx_id = SlaveID (无偏移)
 *   - Pos-Vel 模式: tx_id = SlaveID + 0x100
 *   - Vel 模式:     tx_id = SlaveID + 0x200
 *
 * 实现方式: 从当前 tx_id 中提取 SlaveID (低 8 位),再加上新模式的偏移。
 * 这样可以支持任意模式之间的切换。
 */
void DMMotorSetControlMode(DMMotor_Instance *motor, DM_Control_Mode_e mode)
{
    FDCAN_Instance *fdcan;
    uint32_t base_id;

    if (motor == NULL || motor->motor_fdcan_instance == NULL)
        return;

    fdcan = motor->motor_fdcan_instance;
    motor->control_mode = mode;
    base_id = motor->motor_fdcan_instance->tx_id & 0xFF; /* 获取基础 SlaveID */

    switch (mode) {
        case DM_MODE_POS_VEL:
            fdcan->tx_id = base_id + 0x100;
            break;
        case DM_MODE_VEL:
            fdcan->tx_id = base_id + 0x200;
            break;
        default:
            fdcan->tx_id = base_id;
            break;
    }
 
    fdcan->txconf.Identifier = fdcan->tx_id; 
}

void DMMotorSetMITParams(DMMotor_Instance *motor, float kp, float kd)
{
    if (motor == NULL)
        return;

    motor->mit_params.kp = kp;
    motor->mit_params.kd = kd;
}

void DMMotorSetMITProfile(DMMotor_Instance *motor, float max_vel_rad_s, float max_acc_rad_s2)
{
    if (motor == NULL)
        return;

    if (max_vel_rad_s < 0.0f)
        max_vel_rad_s = -max_vel_rad_s;
    if (max_acc_rad_s2 < 0.0f)
        max_acc_rad_s2 = -max_acc_rad_s2;

    motor->mit_profile.max_vel_rad_s = max_vel_rad_s;
    motor->mit_profile.max_acc_rad_s2 = max_acc_rad_s2;
}

/**
 * @brief 发送模式配置命令到电机
 * @note  通过 CAN ID 0x7FF 发送配置帧, 告知电机当前工作模式。
 *        必须在使能后调用。命令码 0x55, 寄存器 0x0A, mode: 1=MIT
 */
void DMSendModeConfig(DMMotor_Instance *motor)
{
    FDCAN_Instance *fdcan;
    uint32_t saved_tx_id;

    if (motor == NULL || motor->motor_fdcan_instance == NULL)
        return;

    fdcan = motor->motor_fdcan_instance;
    saved_tx_id = fdcan->tx_id;

    /* 模式配置帧→固定 CAN ID 0x7FF */
    fdcan->tx_id = 0x7FF;

    fdcan->tx_buff[0] = (uint8_t)(saved_tx_id & 0xFF);
    fdcan->tx_buff[1] = (uint8_t)((saved_tx_id >> 8) & 0x07);
    fdcan->tx_buff[2] = 0x55;                       /* 写入命令码 */
    fdcan->tx_buff[3] = 0x0A;                       /* 寄存器地址 */
    fdcan->tx_buff[4] = motor->control_mode + 1;     /* MIT=1, PosVel=2, Vel=3 */
    fdcan->tx_buff[5] = 0x00;
    fdcan->tx_buff[6] = 0x00;
    fdcan->tx_buff[7] = 0x00;

    DMSendFrame(fdcan);

    /* 恢复 tx_id */
    fdcan->tx_id = saved_tx_id;
    memset(fdcan->tx_buff, 0, 8);
}

/**
 * @brief 达妙电机总控制函数 (周期性调用, 推荐 1kHz)
 *
 * 遍历所有已注册的达妙电机,对每个电机执行:
 *
 *   [1] 获取当前闭环配置 (setting/controller/measure)
 *   [2] 检查是否需要等待新反馈 (DMControlNeedsFreshFeedback)
 *   [3] MIT 模式:
 *       - 使用位置外环 (ANGLE PID) 生成速度参考
 *       - 速度参考做梯形加速度限制
 *       - 直接打包 MIT 帧 (pos_des, vel_des, torque_ff)
 *   [4] 非 MIT 模式:
 *       - 处理电机反转标志
 *       - 位置环计算: pid_ref (角度目标) → ANGLE PID → angle_output
 *       - 速度环计算: angle_output → SPEED PID → speed_output
 *       - 力矩环计算: speed_output → CURRENT PID → current_output
 *       - 根据 stop_flag 和 control_mode 打包控制帧
 *   [5] 发送 CAN 帧 (DMSendFrame)
 *   [6] 清零 feedback_updated (等待下次反馈触发)
 *
 * @note  PID 输出到控制帧的映射:
 *        - 非 MIT: angle_output → position_des, speed_output → velocity_des
 *        - MIT: pos_des = 目标位置, vel_des = 速度参考, torque_ff = 0
 *
 * @warning 此函数必须在 RTOS 任务中调用,不能在中断上下文调用!
 *          因为内部使用了 PIDCalculate (依赖 DWT 浮点计时) 和 HAL CAN 发送。
 *
 * @attention 关于 feedback_updated 的设计:
 *   feedback_updated 是驱动层的"新数据就绪"标志:
 *   - CAN 中断收到反馈 → DecodeDMMotor → feedback_updated = 1
 *   - 控制循环使用反馈 → PID 计算 → feedback_updated = 0
 *   这样保证了:
 *   1. 同一份反馈不会被重复用于多次 PID 计算
 *   2. 没有新反馈时不会用旧数据重复计算 (除非是开环模式)
 *   3. 控制频率受限于反馈频率 (实际应用中反馈频率就是控制频率)
 */
void DMMotorControl(void)
{
    size_t i;
    DMMotor_Instance *motor;

    g_dm_motor_debug.control_count++;

    /* 遍历所有已注册的达妙电机实例 */
    for (i = 0; i < dm_idx; i++) {
        motor = dm_motor_instances[i];
        if (motor == NULL || motor->motor_fdcan_instance == NULL)
            continue;

        /* ---- 启停状态处理 ---- */
        if (motor->stop_flag == MOTOR_STOP) {
            /* 停止状态: 发送失能帧 {0xFF*7, 0xFD} */
            memset(motor->motor_fdcan_instance->tx_buff, 0xFF, 7);
            motor->motor_fdcan_instance->tx_buff[7] = 0xFD;
        } else {
            /* 使能状态: 直接根据控制模式打包控制帧，不进行任何 host 端 PID 计算 */
            switch (motor->control_mode) {
                case DM_MODE_POS_VEL:
                    /* 直接使用 pos_ref 和 vel_ref */
                    DMPackPosVelFrame(motor, motor->pos_ref, motor->vel_ref);
                    break;
                case DM_MODE_VEL:
                    /* 速度模式下，直接使用 pid_ref 作为速度目标 */
                    DMPackVelFrame(motor, motor->motor_controller.pid_ref);
                    break;
                default:
                    /* MIT 模式下，直接使用 pid_ref 作为力矩目标 (t_ff)，KP/KD 设为 0 */
                    DMPackMITFrameWithGains(motor, 0.0f, 0.0f, motor->motor_controller.pid_ref, 0.0f, 0.0f);
                    break;
            }
        }

        /* 确保持续下发控制帧 */
        DMSendFrame(motor->motor_fdcan_instance);

        /* 仅在收到反馈时由中断置 1，此处手动清零不影响发送 */
        motor->feedback_updated = 0U;
    }
}
