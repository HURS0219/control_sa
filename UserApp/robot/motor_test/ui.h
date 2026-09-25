/*
 * ui.h — 制导飞镖发射架最小 UI 占位头文件
 *
 * 说明: 工程 UserApp/os_task.c 会无条件 #include "ui.h", 因此每个兵种目录
 *       都需要提供该头文件。本发射架默认使用 GIMBAL_BOARD, 不编译 UI 任务,
 *       故这里只提供占位声明。
 *
 * 若将来切换到 ONE_BOARD / CHASSIS_BOARD 且需要裁判系统 UI, 请实现
 * MyUIInit() / UITask() 以及 RobotGetInstance(), 并让 RobotInstance 提供
 * referee_data 成员。
 */

#ifndef GUIDED_DART_UI_H
#define GUIDED_DART_UI_H

#include "robot.h"

void MyUIInit(RobotInstance* robot);
void UITask(RobotInstance* robot);

#endif  // GUIDED_DART_UI_H
