#include "dji_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
//#include "memory.h"
#include "string.h"
#include "stdlib.h"

/* ---------------------------------------- 私有函数声明  ------------------------------------- */
static uint8_t MotorSenderGrouping(DJIMotor_Instance *motor, FDCAN_Init_Config_s *fdcan_config);
static void DecodeDJIMotor(FDCAN_Instance *fdcan_instance);
static void DJIMotorLostCallback(void *motor_ptr);
static uint8_t InitSenderInstances(void);
static FDCAN_Instance *RegisterSenderInstance(FDCAN_HandleTypeDef *fdcan_handle, uint32_t tx_id);
static void ClearSenderFrames(void);
static DJIMotor_Instance *FindMotorByFdcanInstance(FDCAN_Instance *fdcan_instance);
static void RefreshMotorFdcanBinding(DJIMotor_Instance *motor);
static uint8_t DJIMotorControlNeedsFreshFeedback(const Motor_Control_Setting_s *motor_setting);
static float DJIRawSpeedRpmToAps(int16_t speed_rpm);
static float DJIEcdToAngleDeg(float ecd_value);
static float DJINormalizeAngleDeg360(float angle_deg);
static float DJICalcNearestSingleTurnTargetDeg(float current_total_angle_deg, float target_single_turn_ecd);
static volatile FDCAN_Debug_Bus_s *DJIGetBusDebug(FDCAN_HandleTypeDef *handle);
static void DJICopy8(volatile uint8_t *dst, const uint8_t *src);
static uint8_t DJISendFrameDirect(FDCAN_Instance *sender_instance);

/* ------------------------------------------ 变量声明  --------------------------------------- */
static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
static uint8_t sender_initialized = 0; // 发送实例初始化标志

/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJIMotor_Instance *dji_motor_instances[DJI_MOTOR_CNT] = {NULL};
volatile DJIMotor_Debug_s g_dji_motor_debug = {0};

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
 * @brief 注册单个发送实例
 */
static FDCAN_Instance *RegisterSenderInstance(FDCAN_HandleTypeDef *fdcan_handle, uint32_t tx_id)
{
    FDCAN_Init_Config_s sender_config = {
        .fdcan_handle = fdcan_handle,
        .tx_id = tx_id,
        .rx_id = 0,  // 发送专用,不接收
        .use_canfd = 0,  // 经典CAN模式
        .can_module_callback = NULL,
        .id = NULL,
    };

    FDCAN_Instance *instance = FDCANRegister(&sender_config);
    if (instance != NULL) {
        FDCANSetDataLength(instance, 8);
    }

    return instance;
}

/**
 * @brief 初始化发送实例数组,在第一个电机注册时调用
 */
static uint8_t InitSenderInstances(void)
{
    static const struct
    {
        FDCAN_HandleTypeDef *fdcan_handle;
        uint32_t tx_id;
    } sender_map[6] = {
        {&hfdcan1, 0x1FF},
        {&hfdcan1, 0x200},
        {&hfdcan1, 0x2FF},
        {&hfdcan2, 0x1FF},
        {&hfdcan2, 0x200},
        {&hfdcan2, 0x2FF},
    };
    size_t i;

    for (i = 0; i < 6; ++i) {
        if (sender_assignment[i] == NULL) {
            sender_assignment[i] = RegisterSenderInstance(sender_map[i].fdcan_handle, sender_map[i].tx_id);
        }

        if (sender_assignment[i] == NULL) {
            g_dji_motor_debug.last_init_stage = 1U;
            return 0;
        }
    }

    sender_initialized = 1;
    return 1;
}

/**
 * @brief 电机分组,因为至多4个电机可以共用一帧CAN控制报文
 *
 * @param motor 电机实例指针
 * @param fdcan_config FDCAN初始化结构体
 */
static uint8_t MotorSenderGrouping(DJIMotor_Instance *motor, FDCAN_Init_Config_s *fdcan_config)
{
    uint8_t motor_id = fdcan_config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;
    uint8_t i;

    if (motor == NULL || fdcan_config == NULL || fdcan_config->fdcan_handle == NULL)
        return 0;

    if (fdcan_config->tx_id < 1 || fdcan_config->tx_id > 8)
        return 0;

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
            // 检查id是否冲突
            if (sender_assignment[motor_grouping] == NULL)
                return 0;

            for (i = 0; i < idx; ++i) {
                if (dji_motor_instances[i] != NULL &&
                    dji_motor_instances[i]->motor_fdcan_instance != NULL &&
                    dji_motor_instances[i]->motor_fdcan_instance->fdcan_handle == fdcan_config->fdcan_handle &&
                    dji_motor_instances[i]->motor_fdcan_instance->rx_id == fdcan_config->rx_id) {
                    return 0;
                }
            }

            sender_enable_flag[motor_grouping] = 1;
            motor->message_num = motor_send_num;
            motor->sender_group = motor_grouping;
            return 1;

        case GM6020:
            if (motor_id < 4) {
                motor_send_num = motor_id;
                motor_grouping = fdcan_config->fdcan_handle == &hfdcan1 ? 0 : 3;
            } else {
                motor_send_num = motor_id - 4;
                motor_grouping = fdcan_config->fdcan_handle == &hfdcan1 ? 2 : 5;
            }

            fdcan_config->rx_id                = 0x204 + motor_id + 1; // 把ID+1,进行分组设置
            if (sender_assignment[motor_grouping] == NULL)
                return 0;

            for (i = 0; i < idx; ++i) {
                if (dji_motor_instances[i] != NULL &&
                    dji_motor_instances[i]->motor_fdcan_instance != NULL &&
                    dji_motor_instances[i]->motor_fdcan_instance->fdcan_handle == fdcan_config->fdcan_handle &&
                    dji_motor_instances[i]->motor_fdcan_instance->rx_id == fdcan_config->rx_id) {
                    return 0;
                }
            }

            sender_enable_flag[motor_grouping] = 1;
            motor->message_num = motor_send_num;
            motor->sender_group = motor_grouping;
            return 1;

        default:
            return 0;
    }
}

/**
 * @brief 根据接收实例反查电机实例
 *
 * @note  正常情况下 fdcan_instance->id 就应该直接指向电机。
 *        这里保留一层兜底,避免绑定字段在调试/回退过程中被清空后整条接收链路失效。
 */
static DJIMotor_Instance *FindMotorByFdcanInstance(FDCAN_Instance *fdcan_instance)
{
    size_t i;
    DJIMotor_Instance *motor;

    if (fdcan_instance == NULL)
        return NULL;

    for (i = 0; i < idx; ++i) {
        motor = dji_motor_instances[i];
        if (motor == NULL || motor->motor_fdcan_instance == NULL)
            continue;

        if (motor->motor_fdcan_instance == fdcan_instance)
            return motor;

        if (motor->motor_fdcan_instance->fdcan_handle == fdcan_instance->fdcan_handle &&
            motor->motor_fdcan_instance->rx_id == fdcan_instance->rx_id) {
            return motor;
        }
    }

    return NULL;
}

/**
 * @brief 刷新电机实例与FDCAN实例的双向绑定
 */
static void RefreshMotorFdcanBinding(DJIMotor_Instance *motor)
{
    if (motor == NULL || motor->motor_fdcan_instance == NULL)
        return;

    motor->motor_fdcan_instance->can_module_callback = DecodeDJIMotor;
    motor->motor_fdcan_instance->id = motor;
}

/**
 * @brief 判断当前控制配置是否真的依赖新反馈
 *
 * @note  开环模式不应被 feedback_updated 绑死，否则参考值改了也可能继续发旧指令。
 */
static uint8_t DJIMotorControlNeedsFreshFeedback(const Motor_Control_Setting_s *motor_setting)
{
    if (motor_setting == NULL)
        return 0U;

    if ((motor_setting->close_loop_type & ANGLE_LOOP) &&
        motor_setting->outer_loop_type == ANGLE_LOOP) {
        return 1U;
    }

    if ((motor_setting->close_loop_type & SPEED_LOOP) &&
        (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))) {
        return 1U;
    }

    if (motor_setting->close_loop_type & CURRENT_LOOP) {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 将DJI电调原始rpm反馈转换为角速度(度/秒)
 */
static float DJIRawSpeedRpmToAps(int16_t speed_rpm)
{
    if (abs(speed_rpm) <= DJI_SPEED_ZERO_RPM)
        return 0.0f;

    return (float)speed_rpm * DJI_RPM_TO_DEG_PER_SEC;
}

/**
 * @brief 将累计编码器值换算成电机轴角度
 *
 * @note  application 仍然可以直接用累计ecd作为位置参考值；
 *        这里在模块内部统一换算为角度,让位置环参数继续保持物理意义。
 */
static float DJIEcdToAngleDeg(float ecd_value)
{
    return ecd_value * ECD_ANGLE_COEF_DJI;
}

/**
 * @brief 将角度归一化到[0, 360)
 */
static float DJINormalizeAngleDeg360(float angle_deg)
{
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg < 0.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/**
 * @brief 将单圈目标角映射到距离当前总角度最近的等效目标
 *
 * @note  application 仍然传单圈ecd参考值；模块内部负责取优弧。
 */
static float DJICalcNearestSingleTurnTargetDeg(float current_total_angle_deg, float target_single_turn_ecd)
{
    float current_single_angle_deg;
    float target_single_angle_deg;
    float delta_angle_deg;

    current_single_angle_deg = DJINormalizeAngleDeg360(current_total_angle_deg);
    target_single_angle_deg = DJINormalizeAngleDeg360(DJIEcdToAngleDeg(target_single_turn_ecd));
    delta_angle_deg = target_single_angle_deg - current_single_angle_deg;

    if (delta_angle_deg > 180.0f) {
        delta_angle_deg -= 360.0f;
    } else if (delta_angle_deg < -180.0f) {
        delta_angle_deg += 360.0f;
    }

    return current_total_angle_deg + delta_angle_deg;
}

/**
 * @brief 获取对应总线的调试结构
 */
static volatile FDCAN_Debug_Bus_s *DJIGetBusDebug(FDCAN_HandleTypeDef *handle)
{
    if (handle == &hfdcan1)
        return &g_fdcan1_debug;
    if (handle == &hfdcan2)
        return &g_fdcan2_debug;
    return NULL;
}

/**
 * @brief 拷贝8字节缓冲区到调试区
 */
static void DJICopy8(volatile uint8_t *dst, const uint8_t *src)
{
    uint8_t i;

    if (dst == NULL || src == NULL)
        return;

    for (i = 0; i < 8U; ++i) {
        dst[i] = src[i];
    }
}

/**
 * @brief 直接通过HAL发送DJI控制帧
 *
 * @note  这条路径与 Test.c 中已验证通过的直发逻辑保持一致。
 */
static uint8_t DJISendFrameDirect(FDCAN_Instance *sender_instance)
{
    FDCAN_TxHeaderTypeDef tx_header;
    volatile FDCAN_Debug_Bus_s *debug_bus;

    if (sender_instance == NULL || sender_instance->fdcan_handle == NULL)
        return 0U;

    tx_header.Identifier = sender_instance->tx_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    debug_bus = DJIGetBusDebug(sender_instance->fdcan_handle);

    if (HAL_FDCAN_GetTxFifoFreeLevel(sender_instance->fdcan_handle) == 0U) {
        if (debug_bus != NULL) {
            debug_bus->tx_timeout_count++;
            debug_bus->tx_fail_count++;
            debug_bus->last_tx_id = tx_header.Identifier;
            debug_bus->last_tx_dlc = tx_header.DataLength;
            debug_bus->last_tx_fdformat = tx_header.FDFormat;
            debug_bus->last_tx_tick_ms = (uint32_t)DWT_GetTimeline_ms();
            debug_bus->last_hal_status = HAL_TIMEOUT;
            debug_bus->last_error_code = sender_instance->fdcan_handle->ErrorCode;
            DJICopy8(debug_bus->last_tx_data, sender_instance->tx_buff);
        }
        return 0U;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(sender_instance->fdcan_handle, &tx_header, sender_instance->tx_buff) != HAL_OK) {
        if (debug_bus != NULL) {
            debug_bus->tx_fail_count++;
            debug_bus->last_tx_id = tx_header.Identifier;
            debug_bus->last_tx_dlc = tx_header.DataLength;
            debug_bus->last_tx_fdformat = tx_header.FDFormat;
            debug_bus->last_tx_tick_ms = (uint32_t)DWT_GetTimeline_ms();
            debug_bus->last_hal_status = HAL_ERROR;
            debug_bus->last_error_code = sender_instance->fdcan_handle->ErrorCode;
            DJICopy8(debug_bus->last_tx_data, sender_instance->tx_buff);
        }
        return 0U;
    }

    if (debug_bus != NULL) {
        debug_bus->tx_ok_count++;
        debug_bus->last_tx_id = tx_header.Identifier;
        debug_bus->last_tx_dlc = tx_header.DataLength;
        debug_bus->last_tx_fdformat = tx_header.FDFormat;
        debug_bus->last_tx_tick_ms = (uint32_t)DWT_GetTimeline_ms();
        debug_bus->last_hal_status = HAL_OK;
        debug_bus->last_error_code = sender_instance->fdcan_handle->ErrorCode;
        DJICopy8(debug_bus->last_tx_data, sender_instance->tx_buff);
    }

    return 1U;
}

/**
 * @brief dji电机的FDCAN回调函数,用于解析电机的反馈报文,并对电机的反馈数据进行滤波
 *
 * @param fdcan_instance  电机的FDCAN实例
 */
static void DecodeDJIMotor(FDCAN_Instance *fdcan_instance)
{
    uint8_t *rxbuff;
    DJIMotor_Instance *motor;
    DJI_Motor_Measure_s *measure;
    int32_t ecd_delta;
    int16_t raw_speed_rpm;
    int16_t raw_current;
    float raw_speed_aps;
    float speed_from_ecd;
    float delta_angle;

    if (fdcan_instance == NULL)
        return;

    if (fdcan_instance->rx_len < 7U) {
        if (fdcan_instance->use_canfd == 0U) {
            fdcan_instance->rx_len = 8U;
        } else {
            return;
        }
    }

    rxbuff = fdcan_instance->rx_buff;
    motor = (DJIMotor_Instance *)fdcan_instance->id;
    if (motor == NULL) {
        motor = FindMotorByFdcanInstance(fdcan_instance);
        if (motor != NULL) {
            RefreshMotorFdcanBinding(motor);
        }
    }
    if (motor == NULL)
        return;

    g_dji_motor_debug.decode_count++;

    measure = &motor->measure;

    if (motor->daemon != NULL) {
        DaemonReload(motor->daemon);
    }
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    // 解析数据并对电流和速度进行滤波
    measure->last_ecd           = measure->ecd;
    measure->ecd                = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    raw_speed_rpm               = (int16_t)((rxbuff[2] << 8) | rxbuff[3]);
    raw_current                 = (int16_t)((rxbuff[4] << 8) | rxbuff[5]);
    raw_speed_aps               = DJIRawSpeedRpmToAps(raw_speed_rpm);
    measure->speed_rpm          = raw_speed_rpm;
    measure->speed_aps_raw      = raw_speed_aps;
    measure->real_current = (int16_t)((1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                                      CURRENT_SMOOTH_COEF * (float)raw_current);
    measure->temperature = rxbuff[6];

    if (motor->feedback_initialized == 0U) {
        measure->last_ecd = measure->ecd;
        measure->speed_aps = raw_speed_aps;
        measure->total_round = 0;
        measure->total_ecd_raw = (int32_t)measure->ecd;
        measure->total_ecd_offset = measure->total_ecd_raw;
        measure->total_ecd = 0;
        measure->total_angle_raw = measure->angle_single_round;
        measure->total_angle = measure->total_angle_raw - measure->zero_offset;
        motor->feedback_initialized = 1U;
        motor->feedback_updated = 1U;

        g_dji_motor_debug.last_decode_rx_id = fdcan_instance->rx_id;
        g_dji_motor_debug.last_ecd = measure->ecd;
        g_dji_motor_debug.last_speed_raw = raw_speed_rpm;
        g_dji_motor_debug.last_current_raw = raw_current;
        g_dji_motor_debug.last_temperature = measure->temperature;
        g_dji_motor_debug.last_sender_group = motor->sender_group;
        g_dji_motor_debug.last_message_num = motor->message_num;
        g_dji_motor_debug.last_ecd_delta_norm = 0;
        g_dji_motor_debug.last_total_ecd = measure->total_ecd;
        g_dji_motor_debug.last_feedback_dt_ms = motor->dt * 1000.0f;
        g_dji_motor_debug.last_speed_aps = measure->speed_aps;
        g_dji_motor_debug.last_speed_from_ecd = 0.0f;
        g_dji_motor_debug.last_total_angle = measure->total_angle;
        return;
    }

    // 多圈角度计算
    ecd_delta = (int32_t)measure->ecd - (int32_t)measure->last_ecd;
    if (ecd_delta > DJI_ECD_HALF_RANGE) {
        ecd_delta -= DJI_ECD_RESOLUTION;
        measure->total_round--;
    } else if (ecd_delta < -DJI_ECD_HALF_RANGE) {
        ecd_delta += DJI_ECD_RESOLUTION;
        measure->total_round++;
    }

    if (motor->dt > 0.0f) {
        if (abs(ecd_delta) <= 1) {
            speed_from_ecd = 0.0f;
        } else {
            speed_from_ecd = (ECD_ANGLE_COEF_DJI * (float)ecd_delta) / motor->dt;
        }
    } else {
        speed_from_ecd = 0.0f;
    }

    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         SPEED_SMOOTH_COEF * raw_speed_aps;
    if (raw_speed_aps == 0.0f && abs(ecd_delta) <= 1) {
        measure->speed_aps = 0.0f;
    }

    delta_angle = ECD_ANGLE_COEF_DJI * (float)ecd_delta;
    measure->total_ecd_raw += ecd_delta;
    measure->total_ecd = measure->total_ecd_raw - measure->total_ecd_offset;
    measure->total_angle_raw += delta_angle;
    measure->total_angle = measure->total_angle_raw - measure->zero_offset;
    motor->feedback_updated = 1U;

    g_dji_motor_debug.last_decode_rx_id = fdcan_instance->rx_id;
    g_dji_motor_debug.last_ecd = measure->ecd;
    g_dji_motor_debug.last_speed_raw = raw_speed_rpm;
    g_dji_motor_debug.last_current_raw = raw_current;
    g_dji_motor_debug.last_temperature = measure->temperature;
    g_dji_motor_debug.last_sender_group = motor->sender_group;
    g_dji_motor_debug.last_message_num = motor->message_num;
    g_dji_motor_debug.last_ecd_delta_norm = ecd_delta;
    g_dji_motor_debug.last_total_ecd = measure->total_ecd;
    g_dji_motor_debug.last_feedback_dt_ms = motor->dt * 1000.0f;
    g_dji_motor_debug.last_speed_aps = measure->speed_aps;
    g_dji_motor_debug.last_speed_from_ecd = speed_from_ecd;
    g_dji_motor_debug.last_total_angle = measure->total_angle;
}

/**
 * @brief  电机守护进程的回调函数,用于检测电机是否丢失
 */
static void DJIMotorLostCallback(void *motor_ptr)
{
    DJIMotor_Instance *motor                 = (DJIMotor_Instance *)motor_ptr;
    if (motor == NULL || motor->motor_fdcan_instance == NULL)
        return;
    motor->stop_flag                         = MOTOR_STOP;
    uint16_t fdcan_bus __attribute__((unused)) = motor->motor_fdcan_instance->fdcan_handle == &hfdcan1 ? 1 : 2;
}

/**
 * @brief 每次控制周期开始前清空所有发送帧缓存,避免历史数据残留
 */
static void ClearSenderFrames(void)
{
    size_t i;

    for (i = 0; i < 6; ++i) {
        if (sender_enable_flag[i] && sender_assignment[i] != NULL) {
            memset(sender_assignment[i]->tx_buff, 0, 8);
        }
    }
}

/* ---------------------------------------- 公有函数实现  ------------------------------------- */

/**
 * @brief 调用此函数注册一个DJI智能电机
 */
DJIMotor_Instance *DJIMotorInit(Motor_Init_Config_s *config)
{
    FDCAN_Init_Config_s can_config;
    DJIMotor_Instance *motor;

    if (config == NULL) {
        g_dji_motor_debug.init_fail_count++;
        g_dji_motor_debug.last_init_stage = 2U;
        return NULL;
    }

    // 第一次调用时初始化发送实例
    if (!sender_initialized && !InitSenderInstances()) {
        g_dji_motor_debug.init_fail_count++;
        g_dji_motor_debug.last_init_stage = 3U;
        return NULL;
    }
    
    if (idx >= DJI_MOTOR_CNT) {
        g_dji_motor_debug.init_fail_count++;
        g_dji_motor_debug.last_init_stage = 4U;
        return NULL;
    }

    motor = (DJIMotor_Instance *)malloc(sizeof(DJIMotor_Instance));
    if (motor == NULL) {
        g_dji_motor_debug.init_fail_count++;
        g_dji_motor_debug.last_init_stage = 5U;
        return NULL;
    }
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

    can_config = config->can_init_config;
    if (!MotorSenderGrouping(motor, &can_config)) {
        free(motor);
        g_dji_motor_debug.init_fail_count++;
        g_dji_motor_debug.last_init_stage = 6U;
        return NULL;
    }

    // 电机的FDCAN初始化 - 关键:设置为经典CAN模式
    can_config.use_canfd = 0;  // DJI电机必须使用经典CAN
    can_config.can_module_callback = DecodeDJIMotor;
    can_config.id                  = motor;
    motor->motor_fdcan_instance    = FDCANRegister(&can_config);
    if (motor->motor_fdcan_instance == NULL) {
        free(motor);
        g_dji_motor_debug.init_fail_count++;
        g_dji_motor_debug.last_init_stage = 7U;
        return NULL;
    }

    // 再次显式绑定,避免后续定位时受到影子状态或异常覆盖影响
    RefreshMotorFdcanBinding(motor);

    // 注册守护进程
    Daemon_Init_Config_s daemon_config = {
        .callback     = DJIMotorLostCallback,
        .owner_id     = motor,
        .reload_count = 2,
    };
    motor->daemon = DaemonRegister(&daemon_config);
    if (motor->daemon == NULL) {
        motor->motor_fdcan_instance->can_module_callback = NULL;
        motor->motor_fdcan_instance->id = NULL;
        free(motor);
        g_dji_motor_debug.init_fail_count++;
        g_dji_motor_debug.last_init_stage = 8U;
        return NULL;
    }

    DJIMotorEnable(motor);

    dji_motor_instances[idx++] = motor;

    g_dji_motor_debug.init_ok_count++;
    g_dji_motor_debug.last_init_stage = 0U;
    g_dji_motor_debug.last_init_tx_id = can_config.tx_id;
    g_dji_motor_debug.last_init_rx_id = can_config.rx_id;
    g_dji_motor_debug.last_init_sender_group = motor->sender_group;
    g_dji_motor_debug.last_init_message_num = motor->message_num;
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
    if (motor == NULL)
        return;

    motor->measure.total_ecd_offset = motor->measure.total_ecd_raw;
    motor->measure.total_ecd = 0;
    motor->measure.zero_offset = motor->measure.total_angle + motor->measure.zero_offset;
}

/**
 * @brief 为所有电机实例计算三环PID,发送控制报文
 */
void DJIMotorControl(void)
{
    uint8_t group, num;
    uint8_t control_needs_feedback;
    int16_t set;
    size_t i;
    DJIMotor_Instance *motor;
    Motor_Control_Setting_s *motor_setting;
    Motor_Controller_s *motor_controller;
    DJI_Motor_Measure_s *measure;
    float pid_measure, pid_ref;

    ClearSenderFrames();
    g_dji_motor_debug.control_count++;
    g_dji_motor_debug.last_sender_enable_mask = 0U;

    // 遍历所有电机实例
    for (i = 0; i < idx; i++) {
        motor            = dji_motor_instances[i];
        if (motor == NULL || motor->motor_fdcan_instance == NULL)
            continue;

        RefreshMotorFdcanBinding(motor);

        motor_setting    = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure          = &motor->measure;
        set              = motor->last_set;
        control_needs_feedback = DJIMotorControlNeedsFreshFeedback(motor_setting);

        if (!control_needs_feedback || motor->feedback_updated) {
            pid_ref = motor_controller->pid_ref;

            if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
                pid_ref = -pid_ref;
            }

            // 计算位置环
            if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP) {
                if (motor_setting->angle_feedback_source == OTHER_FEED) {
                    pid_measure = *motor_controller->other_angle_feedback_ptr;
                } else {
                    pid_measure = measure->total_angle;
                    if (motor_setting->angle_mode == MOTOR_ANGLE_MODE_SINGLE_TURN) {
                        pid_ref = DJICalcNearestSingleTurnTargetDeg(measure->total_angle, pid_ref);
                    } else {
                        pid_ref = DJIEcdToAngleDeg(pid_ref);
                    }
                }
                if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) {
                    pid_measure = -pid_measure;
                }
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
                if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) {
                    pid_measure = -pid_measure;
                }
                pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
            }

            // 计算电流环
            if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD) {
                pid_ref += *motor_controller->current_feedforward_ptr;
            }
            if (motor_setting->close_loop_type & CURRENT_LOOP) {
                pid_measure = (float)measure->real_current;
                if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) {
                    pid_measure = -pid_measure;
                }
                pid_ref = PIDCalculate(&motor_controller->current_PID, pid_measure, pid_ref);
            }

            set = (int16_t)pid_ref;
            motor->last_set = set;
            if (control_needs_feedback) {
                motor->feedback_updated = 0U;
            }
        }

        // 分组填入发送数据
        group                                         = motor->sender_group;
        num                                           = motor->message_num;
        if (group >= 6 || sender_assignment[group] == NULL)
            continue;

        sender_assignment[group]->tx_buff[2 * num]     = (uint8_t)(set >> 8);
        sender_assignment[group]->tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff);

        g_dji_motor_debug.last_control_ref = motor_controller->pid_ref;
        g_dji_motor_debug.last_control_set = set;
        g_dji_motor_debug.last_sender_group = group;
        g_dji_motor_debug.last_message_num = num;

        // 若该电机处于停止状态,直接将buff置零
        if (motor->stop_flag == MOTOR_STOP) {
            memset(sender_assignment[group]->tx_buff + 2 * num, 0, 2);
        }

        g_dji_motor_debug.last_tx_std_id = sender_assignment[group]->tx_id;
        g_dji_motor_debug.last_packed_set =
            (int16_t)(((uint16_t)sender_assignment[group]->tx_buff[2 * num] << 8) |
                      (uint16_t)sender_assignment[group]->tx_buff[2 * num + 1]);
    }

    // 遍历flag,检查是否要发送这一帧报文
    for (i = 0; i < 6; ++i) {
        g_dji_motor_debug.sender_last_tx_ok[i] = 0U;
        if (sender_enable_flag[i]) {
            g_dji_motor_debug.last_sender_enable_mask |= (uint8_t)(1U << i);
        }
        if (sender_enable_flag[i] && sender_assignment[i] != NULL) {
            if (DJISendFrameDirect(sender_assignment[i])) {
                g_dji_motor_debug.sender_last_tx_ok[i] = 1U;
                g_dji_motor_debug.sender_last_tx_count[i]++;
            }
        }
    }
}

