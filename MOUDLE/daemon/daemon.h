/**
 * @file daemon.h
 * @author Bi Kaixiang (wexhicy@gmail.com)
 * @brief   守护进程
 * @version 0.1
 * @date 2024-01-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef DAEMON_H
#define DAEMON_H

#include "stdint.h"
#include "string.h"

#define DAEMON_MAX_NUM 32 // 支持的最大守护进程数量

/* 模块离线处理函数指针 */
typedef void (*offline_callback)(void *);

typedef struct daemon_ins {
    uint16_t reload_count;     // 重载值
    offline_callback callback; // 离线处理函数,当模块离线时调用

    uint16_t temp_count; // 当前值,减为零说明模块离线或异常
    void *id;            // 模块id,用于标识模块,初始化时传入
} Daemon_Instance;

/* daemon初始化配置 */
typedef struct
{
    uint16_t reload_count;     // 实际上这是app唯一需要设置的值?
    uint16_t init_count;       // 上线等待时间,有些模块需要收到主控的指令才会反馈报文,或pc等需要开机时间
    offline_callback callback; // 异常处理函数,当模块发生异常时会被调用

    void *owner_id; // id取拥有daemon的实例的地址,如DJIMotorInstance*,cast成void*类型
} Daemon_Init_Config_s;

/**
 * @brief 注册一个daemon实例
 *
 * @param config 初始化配置
 * @return DaemonInstance* 返回实例指针
 */
Daemon_Instance *DaemonRegister(Daemon_Init_Config_s *config);

/**
 * @brief 当模块收到新的数据或进行其他动作时,调用该函数重载temp_count,相当于"喂狗"
 *
 * @param daemon daemon实例指针
 */
void DaemonReload(Daemon_Instance *daemon);

/**
 * @brief 确认模块是否离线
 *
 * @param daemon
 * @return uint8_t 若在线且工作正常,返回1;否则返回零. 后续根据异常类型和离线状态等进行优化.
 */
uint8_t DaemonIsOnline(Daemon_Instance *daemon);

/**
 * @brief 放入rtos中,会给每个daemon实例的temp_count按频率进行递减操作.
 *        模块成功接受数据或成功操作则会重载temp_count的值为reload_count.
 *
 */
void DaemonTask(void);

#endif // DAEMON_H
