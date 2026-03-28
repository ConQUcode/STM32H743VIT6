#ifndef TEST_H
#define TEST_H

#include "main.h"

/* 初始化应用层测试电机 */
void ChassisInit(void);

/* 旧工程风格的主任务入口 */
void ChassisTask(void);

/* 以下接口仅为兼容旧测试调用保留,新代码优先使用 ChassisInit/ChassisTask */
void Test(void);

void Test_Init(void);
void Test_all_cmd(void);

#endif
