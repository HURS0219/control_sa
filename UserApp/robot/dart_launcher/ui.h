/*
 * ui.h — 制导飞镖发射架 (dart_launcher) 的 UI 占位
 *
 * os_task.c 会无条件 #include "ui.h"。飞镖架可视化后续可接串口屏/OLED,
 * 目前只提供占位声明, 避免在 GIMBAL_BOARD 下引用不存在的 UI 实现。
 */
#ifndef DART_LAUNCHER_UI_H
#define DART_LAUNCHER_UI_H

#include "robot.h"

void MyUIInit(RobotInstance* robot);
void UITask(RobotInstance* robot);

#endif  // DART_LAUNCHER_UI_H
