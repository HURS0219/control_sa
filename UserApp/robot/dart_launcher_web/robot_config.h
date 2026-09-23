/*
 * robot_config.h — 制导飞镖发射架 · 网页控制版 (dart_launcher_web) v0.1
 *
 * 链路: 手机网页 --WiFi--> ESP32 --UART(huart6)--> C板(STM32F407) --CAN1--> 4 电机
 *       OpenMV --SPI--> C板 (绿光中心 x)
 *
 * 四电机 (统一 CAN1, 1Mbps):
 *   拉簧电机 A  M3508  ID2   反馈 0x202
 *   拉簧电机 B  M3508  ID3   反馈 0x203
 *   扳机丝杆    M3508  ID4   反馈 0x204
 *   Yaw 电机    M2006  ID1   反馈 0x201
 *
 * !!! 注意: 附录 A 写“M2006 用 0x1FF”，实际 C610/M2006 与 C620/M3508 一样,
 *     ID1~4 都用 0x200 控制帧。本工程按真实协议实现 (simple_motor), 因此四个电机
 *     可以共用一帧 0x200 同时控制。若你的电调确为特殊固件请再核实。
 *
 * 所有标注 TODO 的参数必须实测标定后填入。
 */
#ifndef DART_LAUNCHER_WEB_ROBOT_CONFIG_H
#define DART_LAUNCHER_WEB_ROBOT_CONFIG_H

#define DART_LAUNCHER_WEB_VERSION "0.1"

#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "simple_motor.h"

/* ================= CAN ================= */
#define DART_CAN_HANDLE (&hcan1)

/* ================= 电机角色 / 类型 / ID ================= */
#define DART_SPRING_A_TYPE SIMPLE_MOTOR_M3508
#define DART_SPRING_B_TYPE SIMPLE_MOTOR_M3508
#define DART_TRIGGER_TYPE  SIMPLE_MOTOR_M3508
#define DART_YAW_TYPE      SIMPLE_MOTOR_M2006

#define DART_SPRING_A_ID 2
#define DART_SPRING_B_ID 3
#define DART_TRIGGER_ID  4
#define DART_YAW_ID      1

/* 减速比 (电机转子 : 输出轴) —— 用于角度/速度以输出轴为单位 */
#define DART_M3508_GEAR_RATIO 19.2032f  // 3591/187 (P19)
#define DART_M2006_GEAR_RATIO 36.0f     // P36

/* 方向: 1 = 反向, 0 = 正常 (镜像安装时设 1) */
#define DART_SPRING_A_REVERSE 0
#define DART_SPRING_B_REVERSE 1
#define DART_TRIGGER_REVERSE  0
#define DART_YAW_REVERSE      0

/* ================= ESP32 串口 (与 wifi_gm6020 一致) ================= */
#define DART_UART_HANDLE (&huart6)
#define DART_RECV_SIZE   96
#define DART_CMD_TIMEOUT_MS 1500u  // 网页断联后自动停车
#define DART_FB_PERIOD_MS   150u   // 遥测周期

/* ================= 扳机舵机 (PWM) ================= */
#define DART_SERVO_TIM     (&htim1)
#define DART_SERVO_CHANNEL TIM_CHANNEL_1  // PE9, 与 dart_launcher 一致
#define DART_SERVO_PERIOD_S 0.02f         // 50Hz
#define DART_SERVO_MIN_US  500.0f
#define DART_SERVO_MAX_US  2500.0f
#define DART_SERVO_STD_US  1000.0f  // TODO 标准位脉宽
#define DART_SERVO_PREP_US 2000.0f  // TODO 预备位脉宽

/* ================= 拉簧状态机参数 ================= */
#define DART_SPRING_PREP_TURNS 5.0f   // TODO 预备位圈数 (输出轴)
#define DART_SPRING_MAX_TURNS  20.0f  // 软限位
#define DART_SPRING_POS_TOL    0.05f  // 到位判定 (输出轴圈)
#define DART_FSM_STEP_TIMEOUT_MS 6000u

/* ================= Yaw / 视觉 ================= */
#define DART_VIS_CENTER      160      // 画面中心像素 (TODO 按分辨率)
#define DART_VIS_X_MIN       0
#define DART_VIS_X_MAX       320
#define DART_VIS_LOST_PIXELS 60       // 距中心超过该像素视为丢目标
#define DART_YAW_AIM_KP      0.05f    // deg/pixel (TODO 标定)
#define DART_YAW_AIM_DEADBAND 2       // pixel
#define DART_YAW_GUIDE_DEG   0.0f     // 制导位

/* ================= 控制周期 ================= */
#define DART_CTRL_DT 0.001f  // RobotTask ~1kHz

#endif  // DART_LAUNCHER_WEB_ROBOT_CONFIG_H
