/*
 * robot.c — 制导飞镖发射架 (dart_launcher) v0.1 应用层入口
 *
 * RobotTask 由 os_task.c 的 StartROBOTTASK 以 ~1kHz 调用。
 * 注意: DJI 电机控制帧的发送已由 Modules/motor/motor_task.c 的 MotorControlTask
 *       统一负责, 应用层不再调用 DJIMotorTask()。
 */
#include "robot.h"

#include "launcher.h"
#include "user_lib.h"

RobotInstance* robot = NULL;

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));

  LauncherInit();
  robot->launcher = launcher;
}

void RobotTask(void) {
  LauncherTask();  // 状态机 + 蓄力/瞄准/释放控制
}
