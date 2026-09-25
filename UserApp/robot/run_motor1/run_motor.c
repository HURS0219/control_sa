/*
 * run_motor.c — 最简 GM6020 转动实现 (直接用 HAL 操作 CAN1)
 */
#include "run_motor.h"

#include "can.h"   // hcan1
#include "main.h"  // HAL_*

#define GM6020_ID 2
#define GM6020_CTRL_ID ((GM6020_ID <= 4) ? 0x1FF : 0x2FF)

RunMotorInstance g_run_motor;

void RunMotorInit(void) {
  /* 接收滤波器: 只收本电机反馈帧 (0x204 + ID) */
  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.FilterIdHigh = (uint16_t)((0x204 + GM6020_ID) << 5);
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &filter);

  /* main.c 的 MX_CAN1_Init 只做了 HAL_CAN_Init, 这里要 Start */
  HAL_CAN_Start(&hcan1);
}

void RunMotorSet(int16_t value) {
  uint8_t data[8] = {0};

  uint8_t slot = (uint8_t)((GM6020_ID - 1) % 4);  // ID2 -> slot1 -> 第2~3字节
  data[slot * 2] = (uint8_t)(value >> 8);
  data[slot * 2 + 1] = (uint8_t)(value & 0xFF);

  CAN_TxHeaderTypeDef tx = {0};
  tx.StdId = GM6020_CTRL_ID;
  tx.IDE = CAN_ID_STD;
  tx.RTR = CAN_RTR_DATA;
  tx.DLC = 8;
  tx.TransmitGlobalTime = DISABLE;

  uint32_t t0 = HAL_GetTick();
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
    if (HAL_GetTick() - t0 > 5) return;
  }

  uint32_t mailbox = 0;
  HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox);
}

void RunMotorReadFeedback(void) {
  CAN_RxHeaderTypeDef rx;
  uint8_t buf[8];

  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx, buf) != HAL_OK) break;
    if (rx.StdId == (0x204 + GM6020_ID)) {
      g_run_motor.ecd = (uint16_t)((buf[0] << 8) | buf[1]);
      g_run_motor.rpm = (int16_t)((buf[2] << 8) | buf[3]);
      g_run_motor.current = (int16_t)((buf[4] << 8) | buf[5]);
      g_run_motor.temp = buf[6];
      g_run_motor.fb_count++;
    }
  }
}
