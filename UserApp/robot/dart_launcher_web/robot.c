/*
 * robot.c — 制导飞镖发射架 · 网页控制版 (dart_launcher_web) v0.1 入口
 *
 * RobotTask 由 os_task.c 的 StartROBOTTASK 以 ~1kHz 调用。
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

  DartMotorInit();   // CAN 总线 + 四电机
  DartServoInit();   // 扳机舵机
  DartVisionInit();  // 视觉坐标
  DartFsmInit();     // 状态机
  DartStoreInit();   // 恢复掉电保存的参数
  DartLinkInit();    // ESP32 串口协议
}

void RobotTask(void) {
  DartMotorTask();   // 读取反馈 + PID 输出 + 发送控制帧
  DartFsmTask();     // 状态机 / 自动化
  DartVisionTask();  // 视觉超时判定
  DartLinkTask();    // 遥测 + 扫描 + 断联保护
  DartStoreTask();   // 参数落盘
}
