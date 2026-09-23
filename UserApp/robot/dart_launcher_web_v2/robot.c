/*
 * robot.c — 制导飞镖发射架 · 网页控制版 v2 入口
 */
#include "robot.h"

#include "dart_fsm.h"
#include "dart_link.h"
#include "dart_motor.h"
#include "dart_servo.h"
#include "dart_store.h"
#include "dart_vision.h"
#include "main.h"
#include "user_lib.h"

RobotInstance* robot = NULL;

void RobotInit(void) {
  robot = (RobotInstance*)zmalloc(sizeof(RobotInstance));
  DartMotorInit();
  DartServoInit();
  DartVisionInit();
  DartFsmInit();
  DartStoreInit();
  DartLinkInit();
}

void RobotTask(void) {
  DartMotorTask();
  DartFsmTask();
  DartVisionTask();
  DartLinkTask();
  DartStoreTask();
}
