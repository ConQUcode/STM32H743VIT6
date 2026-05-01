#include "Test.h"
#include "Flash.h"
#include "bsp_dwt.h"
#include "dji_motor.h"
#include "fdcan.h"
#include "remote.h" // 引用遥控器库

/* 当前应用层保留单电机角度环模板,后续扩多电机时直接按此结构增加实例即可 */
// 引入串口库以测试 TX 数据
#include "usart.h" 

// 引入串口库以测试 TX 数据
#include "usart.h" 

#define CHASSIS_DWT_FREQ_MHZ 400U
#define CHASSIS_MOTOR_RF_TX_ID 1U
#define CHASSIS_MOTOR_RF_TARGET_ECD 2048.0f

static DJIMotor_Instance *motor_rf;
static Remote_Data_s *test_remote; // 方便调试器直接追踪的遥控器指针，同 motor_rf

extern FDCAN_HandleTypeDef hfdcan1;

static void ChassisSetMotorRef(void)
{
    if (motor_rf == NULL) {
        return;
    }

    /* 当前固定目标用于烧录后直接验证角度环 */
    DJIMotorSetRef(motor_rf, CHASSIS_MOTOR_RF_TARGET_ECD);
}

static void TestChassisInit(void)
{
	  // 初始化并启动遥控器 DMA接收 
    RemoteControlInit();
    test_remote = remote_data; // 将底层暴露的指针赋给当前文件的静态变量，方便你在 Watch 窗口实时查看
	 DWT_Init(CHASSIS_DWT_FREQ_MHZ);
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.fdcan_handle = &hfdcan2,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 0.12f,
                .Ki = 0.0010f,
                .Kd = 0.0f,
                .MaxOut = 4500.0f,
                .DeadBand = 120.0f,
                .IntegralLimit = 1800.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit,
            },
            .angle_PID = {
                .Kp = 120.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .MaxOut = 9000.0f,
                .DeadBand = 0.5f,
                .IntegralLimit = 0.0f,
                .Improve = PID_IMPROVE_NONE,
            },
            .current_PID = {
                .Kp = 1.0f,
                .Ki = 0.01f,
                .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 10000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .angle_mode = MOTOR_ANGLE_MODE_SINGLE_TURN,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = M3508,
    };

    if (motor_rf != NULL) {
        return;
    }

   

    chassis_motor_config.can_init_config.tx_id = CHASSIS_MOTOR_RF_TX_ID;
    motor_rf = DJIMotorInit(&chassis_motor_config);
    

}

static void TestChassisTask(void)
{
    if (motor_rf == NULL) {
        return;
    }

    ChassisSetMotorRef();
}

/* 兼容旧测试入口,实际逻辑统一走底盘主入口 */
void Test(void)
{
    TestChassisTask();
}

void Test_Init(void)
{
    TestChassisInit();
}

/* 保留旧名字,避免外部已有调用点失效 */
void Test_all_cmd(void)
{
    TestChassisTask();
}
