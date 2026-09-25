/*
 * ui.h — UI 占位 (os_task.c 会无条件 #include "ui.h")
 */
#ifndef RUN_MOTOR1_UI_H
#define RUN_MOTOR1_UI_H

#include "robot.h"

void MyUIInit(RobotInstance* robot);
void UITask(RobotInstance* robot);

#endif  // RUN_MOTOR1_UI_H
