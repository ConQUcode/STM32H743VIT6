#ifndef TEST_H
#define TEST_H

#include "main.h"
#include "chassis.h"

/* 以下接口仅为兼容旧测试调用保留,新代码优先使用 ChassisInit/ChassisTask */
void Test(void);

void Test_Init(void);
void Test_all_cmd(void);

#endif
