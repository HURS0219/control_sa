/*
 * run_motor.h — 最简 GM6020 转动接口 (直接用 HAL 发 CAN, 不依赖 dji_motor)
 *
 * GM6020 CAN 协议:
 *   控制帧: 标准帧 ID=0x1FF(拨码1~4)/0x2FF(拨码5~7), 8字节=4电机x2字节,
 *           每个电机按 (ID-1)%4 占 2 字节, 大端 int16, 范围 -30000~+30000
 *   反馈帧: 标准帧 ID=0x204+ID, [0:1]编码器 [2:3]RPM [4:5]电流 [6]温度
 */
#ifndef RUN_MOTOR1_RUN_MOTOR_H
#define RUN_MOTOR1_RUN_MOTOR_H

#include <stdint.h>

typedef struct {
  uint16_t ecd;
  int16_t rpm;
  int16_t current;
  uint8_t temp;
  uint32_t fb_count;
} RunMotorInstance;

extern RunMotorInstance g_run_motor;

void RunMotorInit(void);
void RunMotorSet(int16_t value);
void RunMotorReadFeedback(void);

#endif  // RUN_MOTOR1_RUN_MOTOR_H
