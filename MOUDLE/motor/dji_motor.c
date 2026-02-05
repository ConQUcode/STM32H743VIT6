#include "dji_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
//#include "memory.h"
#include "stdlib.h"

/* ---------------------------------------- 私有函数声明  ------------------------------------- */
static void MotorSenderGrouping(DJIMotor_Instance *motor, FDCAN_Init_Config_s *fdcan_config);
static void DecodeDJIMotor(FDCAN_Instance *fdcan_instance);
static void DJIMotorLostCallback(void *motor_ptr);
static void InitSenderInstances(void);

/* ------------------------------------------ 变量声明  --------------------------------------- */
static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
static uint8_t sender_initialized = 0; // 发送实例初始化标志

/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJIMotor_Instance *dji_motor_instances[DJI_MOTOR_CNT] = {NULL};

/**
 * @brief 由于DJI电机发送以四个一组的形式进行,故对其进行特殊处理,用6个(2fdcan*3group)fdcan_instance专门负责发送
 *        该变量将在 DJIMotorControl() 中使用,分组在 MotorSenderGrouping()中进行
 *
 * @note  使用FDCAN实例数组,通过FDCANRegister()初始化
 *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * fdcan1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * fdcan2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
static FDCAN_Instance *sender_assignment[6] = {NULL};

/**
 * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧
 */
static uint8_t sender_enable_flag[6] = {0};

/* ---------------------------------------- 私有函数实现  ------------------------------------- */

/**
 * @brief 初始化发送实例数组,在第一个电机注册时调用
 */
static void InitSenderInstances(void)
{
    FDCAN_Init_Config_s sender_config;
    
    // FDCAN1 - 0x1FF组
    sender_config = (FDCAN_Init_Config_s){
        .fdcan_handle = &hfdcan1,
        .tx_id = 0x1FF,
        .rx_id = 0,  // 发送专用,不接收
        .use_canfd = 0,  // 经典CAN模式
        .can_module_callback = NULL,
        .id = NULL,
    };
    sender_assignment[0] = FDCANRegister(&sender_config);
    FDCANSetDataLength(sender_assignment[0], 8);
    
    // FDCAN1 - 0x200组
    sender_config.tx_id = 0x200;
    sender_assignment[1] = FDCANRegister(&sender_config);
    FDCANSetDataLength(sender_assignment[1], 8);
    
    // FDCAN1 - 0x2FF组
    sender_config.tx_id = 0x2FF;
    sender_assignment[2] = FDCANRegister(&sender_config);
    FDCANSetDataLength(sender_assignment[2], 8);
    
    // FDCAN2 - 0x1FF组
    sender_config.fdcan_handle = &hfdcan2;
    sender_config.tx_id = 0x1FF;
    sender_assignment[3] = FDCANRegister(&sender_config);
    FDCANSetDataLength(sender_assignment[3], 8);
    
    // FDCAN2 - 0x200组
    sender_config.tx_id = 0x200;
    sender_assignment[4] = FDCANRegister(&sender_config);
    FDCANSetDataLength(sender_assignment[4], 8);
    
    // FDCAN2 - 0x2FF组
    sender_config.tx_id = 0x2FF;
    sender_assignment[5] = FDCANRegister(&sender_config);
    FDCANSetDataLength(sender_assignment[5], 8);
    
    sender_initialized = 1;
}

/**
 * @brief 电机分组,因为至多4个电机可以共用一帧CAN控制报文
 *
 * @param motor 电机实例指针
 * @param fdcan_config FDCAN初始化结构体
 */
static void MotorSenderGrouping(DJIMotor_Instance *motor, FDCAN_Init_Config_s *fdcan_config)
{
    uint8_t motor_id = fdcan_config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;

    switch (motor->motor_type) {
        case M2006:
        case M3508:
            if (motor_id < 4) {
                motor_send_num = motor_id;
                motor_grouping = fdcan_config->fdcan_handle == &hfdcan1 ? 1 : 4;
            } else {
                motor_send_num = motor_id - 4;
                motor_grouping = fdcan_config->fdcan_handle == &hfdcan1 ? 0 : 3;
            }

            // 计算接收id
            fdcan_config->rx_id = 0x200 + motor_id + 1; // 把ID+1,进行分组设置
            // 设置分组发送id
            sender_enable_flag[motor_grouping] = 1;      // 设置发送标志位,防止发送空帧
            motor->message_num                 = motor_send_num; // 发送id
            motor->sender_group                = motor_grouping; // 分组

            // 检查id是否冲突
            for (uint8_t i = 0; i < idx; ++i) {
                if (dji_motor_instances[i]->motor_fdcan_instance->fdcan_handle == fdcan_config->fdcan_handle && 
                    dji_motor_instances[i]->motor_fdcan_instance->rx_id == fdcan_config->rx_id) {
                    // id冲突,进入错误处理
                    uint16_t fdcan_bus __attribute__((unused)) = fdcan_config->fdcan_handle == &hfdcan1 ? 1 : 2;
                    // 可以在这里添加错误处理代码
                }
            }
            break;

        case GM6020:
            if (motor_id < 4) {
                motor_send_num = motor_id;
                motor_grouping = fdcan_config->fdcan_handle == &hfdcan1 ? 0 : 3;
            } else {
                motor_send_num = motor_id - 4;
                motor_grouping = fdcan_config->fdcan_handle == &hfdcan1 ? 2 : 5;
            }

            fdcan_config->rx_id                = 0x204 + motor_id + 1; // 把ID+1,进行分组设置
            sender_enable_flag[motor_grouping] = 1;                    // 设置发送标志位,防止发送空帧
            motor->message_num                 = motor_send_num;       // 发送id
            motor->sender_group                = motor_grouping;       // 分组

            for (uint8_t i = 0; i < idx; ++i) {
                if (dji_motor_instances[i]->motor_fdcan_instance->fdcan_handle == fdcan_config->fdcan_handle && 
                    dji_motor_instances[i]->motor_fdcan_instance->rx_id == fdcan_config->rx_id) {
                    // id冲突,进入错误处理
                    uint16_t fdcan_bus __attribute__((unused)) = fdcan_config->fdcan_handle == &hfdcan1 ? 1 : 2;
                }
            }
            break;

        default:
            break;
    }
}

/**
 * @brief dji电机的FDCAN回调函数,用于解析电机的反馈报文,并对电机的反馈数据进行滤波
 *
 * @param fdcan_instance  电机的FDCAN实例
 */
static void DecodeDJIMotor(FDCAN_Instance *fdcan_instance)
{
    uint8_t *rxbuff              = fdcan_instance->rx_buff;
    DJIMotor_Instance *motor     = (DJIMotor_Instance *)fdcan_instance->id;
    DJI_Motor_Measure_s *measure = &motor->measure;

    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    // 解析数据并对电流和速度进行滤波
    measure->last_ecd           = measure->ecd;
    measure->ecd                = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps          = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    // 多圈角度计算
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = (measure->total_round * 360 + measure->angle_single_round) - measure->zero_offset;
}

/**
 * @brief  电机守护进程的回调函数,用于检测电机是否丢失
 */
static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotor_Instance *motor                 = (DJIMotor_Instance *)motor_ptr;
    motor->stop_flag                         = MOTOR_STOP;
    uint16_t fdcan_bus __attribute__((unused)) = motor->motor_fdcan_instance->fdcan_handle == &hfdcan1 ? 1 : 2;
}

/* ---------------------------------------- 公有函数实现  ------------------------------------- */

/**
 * @brief 调用此函数注册一个DJI智能电机
 */
DJIMotor_Instance *DJIMotorInit(Motor_Init_Config_s *config)
{
    // 第一次调用时初始化发送实例
    if (!sender_initialized) {
        InitSenderInstances();
    }
    
    if (idx >= DJI_MOTOR_CNT) {
        return NULL;
    }

    DJIMotor_Instance *motor = (DJIMotor_Instance *)malloc(sizeof(DJIMotor_Instance));
    memset(motor, 0, sizeof(DJIMotor_Instance));

    // 电机的基本设置
    motor->motor_type     = config->motor_type;
    motor->motor_settings = config->controller_setting_init_config;

    // 电机的PID初始化
    PIDInit(&motor->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.current_feedforward_ptr  = config->controller_param_init_config.current_feedforward_ptr;
    motor->motor_controller.speed_feedforward_ptr    = config->controller_param_init_config.speed_feedforward_ptr;

    MotorSenderGrouping(motor, &config->can_init_config);

    // 电机的FDCAN初始化 - 关键:设置为经典CAN模式
    config->can_init_config.use_canfd = 0;  // DJI电机必须使用经典CAN
    config->can_init_config.can_module_callback = DecodeDJIMotor;
    config->can_init_config.id                  = motor;
    motor->motor_fdcan_instance                 = FDCANRegister(&config->can_init_config);

    // 注册守护进程
    Daemon_Init_Config_s daemon_config = {
        .callback     = DJIMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 2,
    };
    motor->daemon = DaemonRegister(&daemon_config);

    DJIMotorEnable(motor);

    dji_motor_instances[idx++] = motor;
    return motor;
}

void DJIMotorEnable(DJIMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DJIMotorStop(DJIMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void DJIMotorChangeFeed(DJIMotor_Instance *motor, Closeloop_Type_e loop, Feedback_Source_e type, float *ptr)
{
    if (loop == ANGLE_LOOP) {
        motor->motor_settings.angle_feedback_source      = type;
        motor->motor_controller.other_angle_feedback_ptr = ptr;
    } else if (loop == SPEED_LOOP) {
        motor->motor_settings.speed_feedback_source      = type;
        motor->motor_controller.other_speed_feedback_ptr = ptr;
    }
}

void DJIMotorOuterLoop(DJIMotor_Instance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

void DJIMotorSetRef(DJIMotor_Instance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void DJIMotorReset(DJIMotor_Instance *motor)
{
    motor->measure.zero_offset = motor->measure.total_angle + motor->measure.zero_offset;
}

/**
 * @brief 为所有电机实例计算三环PID,发送控制报文
 */
void DJIMotorControl(void)
{
    uint8_t group, num;
    int16_t set;
    DJIMotor_Instance *motor;
    Motor_Control_Setting_s *motor_setting;
    Motor_Controller_s *motor_controller;
    DJI_Motor_Measure_s *measure;
    float pid_measure, pid_ref;

    // 遍历所有电机实例
    for (size_t i = 0; i < idx; i++) {
        motor            = dji_motor_instances[i];
        motor_setting    = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure          = &motor->measure;
        pid_ref          = motor_controller->pid_ref;

        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
            pid_ref = -pid_ref;
        }

        // 计算位置环
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP) {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle;
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))) {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD) {
                pid_ref += *motor_controller->speed_feedforward_ptr;
            }
            if (motor_setting->speed_feedback_source == OTHER_FEED) {
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            } else {
                pid_measure = measure->speed_aps;
            }
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // 计算电流环
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD) {
            pid_ref += *motor_controller->current_feedforward_ptr;
        }
        if (motor_setting->close_loop_type & CURRENT_LOOP) {
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);
        }

        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) {
            pid_ref *= -1;
        }

        // 获取最终输出
        set = (int16_t)pid_ref;

        // 分组填入发送数据
        group                                         = motor->sender_group;
        num                                           = motor->message_num;
        sender_assignment[group]->tx_buff[2 * num]     = (uint8_t)(set >> 8);
        sender_assignment[group]->tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff);

        // 若该电机处于停止状态,直接将buff置零
        if (motor->stop_flag == MOTOR_STOP) {
            memset(sender_assignment[group]->tx_buff + 2 * num, 0, 2);
        }
    }

    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < 6; ++i) {
        if (sender_enable_flag[i]) {
            FDCANTransmit(sender_assignment[i], 1);
        }
    }
}
