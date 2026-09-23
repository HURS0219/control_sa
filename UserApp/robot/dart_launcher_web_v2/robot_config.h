/*
 * robot_config.h — 制导飞镖发射架 · 网页控制版 v2 (dart_launcher_web_v2)
 *
 * 链路: 手机网页 --WiFi--> ESP32 --UART(huart6)--> C板(STM32F407) --CAN1--> 4 电机
 *       OpenMV --SPI--> C板 (绿光中心 x)
 *
 * 四电机 (CAN1, 1Mbps):
 *   拉簧A  M3508 ID2  反馈 0x202
 *   拉簧B  M3508 ID3  反馈 0x203
 *   扳机   M3508 ID4  反馈 0x204
 *   Yaw    M2006 ID1  反馈 0x201
 *
 * 全部 TODO 参数需实测标定。
 */
#ifndef DART_LAUNCHER_WEB_V2_ROBOT_CONFIG_H
#define DART_LAUNCHER_WEB_V2_ROBOT_CONFIG_H

#define DART_V2_VERSION "2.0"

#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "simple_motor.h"

/* ================= CAN ================= */
#define DART_CAN_HANDLE (&hcan1)

/* ================= 电机角色 ================= */
#define DART_SPRING_A_TYPE SIMPLE_MOTOR_M3508
#define DART_SPRING_B_TYPE SIMPLE_MOTOR_M3508
#define DART_TRIGGER_TYPE  SIMPLE_MOTOR_M3508
#define DART_YAW_TYPE      SIMPLE_MOTOR_M2006

#define DART_SPRING_A_ID 2
#define DART_SPRING_B_ID 3
#define DART_TRIGGER_ID  4
#define DART_YAW_ID      1

#define DART_M3508_GEAR_RATIO 19.2032f
#define DART_M2006_GEAR_RATIO 36.0f

/* 方向 (1=反向, 0=正常); 可在网页上运行时切换并保存 */
#define DART_SPRING_A_REVERSE 0
#define DART_SPRING_B_REVERSE 1
#define DART_TRIGGER_REVERSE  0
#define DART_YAW_REVERSE      0

/* ================= ESP32 串口 ================= */
#define DART_UART_HANDLE (&huart6)
#define DART_RECV_SIZE   128
#define DART_CMD_TIMEOUT_MS 2000u
#define DART_FB_PERIOD_MS   150u

/* ================= 拉簧/圈数 ================= */
#define DART_SPRING_PREP_TURNS 5.0f    // TODO 预备位圈数
#define DART_SPRING_MAX_TURNS  30.0f   // 软限位
#define DART_POS_TOL_TURNS     0.03f
#define DART_TASK_STEP_TIMEOUT_MS 8000u

/* ================= 堵转保护 ================= */
#define DART_STALL_CUR       4000   // 电流阈值 (反馈原始值)
#define DART_STALL_MS        800u   // 持续该时间判定堵转
#define DART_STALL_GRACE_MS  400u   // 启动后忽略

/* ================= 扳机舵机 (PWM) ================= */
#define DART_SERVO_TIM     (&htim1)
#define DART_SERVO_CHANNEL TIM_CHANNEL_1
#define DART_SERVO_PERIOD_S 0.02f
#define DART_SERVO_MIN_US  500.0f
#define DART_SERVO_MAX_US  2500.0f
#define DART_SERVO_DEG_RANGE 270.0f   // 对应 500~2500us 的角度量程
#define DART_SERVO_STD_DEG  0.0f      // TODO 标准位角度
#define DART_SERVO_PREP_DEG 90.0f     // TODO 预备位角度

/* ================= Yaw / 视觉 ================= */
#define DART_VIS_CENTER      160
#define DART_YAW_AIM_KP      0.04f   // deg/pixel
#define DART_YAW_AIM_DEADBAND 2
#define DART_YAW_GUIDE_DEG   0.0f
#define DART_VIS_TIMEOUT_MS  500u

/* ================= 控制周期 ================= */
#define DART_CTRL_DT 0.001f

#endif  // DART_LAUNCHER_WEB_V2_ROBOT_CONFIG_H
