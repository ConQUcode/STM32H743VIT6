/**
 * @file catch.h
 * @brief 抓取/夹取机构模块
 *
 * 使用飞特舵机(FT_1~FT_4)控制夹具开合，配合 DJI M2006/M3508 升降机构。
 * CatchInit() 初始化硬件，CatchTask() 在 RTOS 任务中周期调用。
 */

#ifndef __CATCH_H
#define __CATCH_H

void CatchInit(void);
void CatchTask(void);

#endif
