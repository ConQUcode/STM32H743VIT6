#include "daemon.h"
#include "bsp_dwt.h"
#include "stdlib.h"
#include "string.h"

/* 用于保存所有的daemon instance */
static Daemon_Instance *daemon_instances[DAEMON_MAX_NUM];
static uint8_t idx; // 用于记录当前注册的daemon数量

Daemon_Instance *DaemonRegister(Daemon_Init_Config_s *config)
{
    Daemon_Instance *daemon_instance = (Daemon_Instance *)malloc(sizeof(Daemon_Instance));
    memset(daemon_instance, 0, sizeof(Daemon_Instance));

    daemon_instance->id           = config->owner_id;
    daemon_instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count; // 默认重载值为100
    daemon_instance->callback     = config->callback;
    daemon_instance->temp_count   = config->init_count == 0 ? daemon_instance->reload_count : config->init_count; // 若未设置init_count,则使用reload_count

    daemon_instances[idx++] = daemon_instance;

    return daemon_instance;
}

/**
 * @brief 当模块收到新的数据或进行其他动作时,调用该函数重载temp_count,相当于"喂狗"
 *
 * @param daemon daemon实例指针
 */
void DaemonReload(Daemon_Instance *daemon)
{
    daemon->temp_count = daemon->reload_count;
}

/**
 * @brief 确认模块是否离线
 *
 * @param daemon
 * @return uint8_t 若在线且工作正常,返回1;否则返回零. 后续根据异常类型和离线状态等进行优化.
 */
uint8_t DaemonIsOnline(Daemon_Instance *daemon)
{
    return daemon->temp_count > 0;
}

/**
 * @brief 放入rtos中,会给每个daemon实例的temp_count按频率进行递减操作.
 *        模块成功接受数据或成功操作则会重载temp_count的值为reload_count.
 *
 */
void DaemonTask(void)
{
    Daemon_Instance *daemon;
    for (uint8_t i = 0; i < idx; i++) {
        daemon = daemon_instances[i];
        if (daemon->temp_count > 0) // 如果计数器还有值,说明上一次喂狗后还没有超时,则计数器减一
            daemon->temp_count--;
        else if (daemon->callback != NULL) // 等于零说明超时了,调用回调函数(如果有的话)
            daemon->callback(daemon->id);
        // @todo 可以加入蜂鸣器或者led等提示
    }
}
