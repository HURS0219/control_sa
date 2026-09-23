/*
 * dart_link.c — ESP32 <-> C板 串口协议 (v2)
 *
 * ESP32 -> STM32 ('\n'):
 *   PING                       -> PONG
 *   S                          扫描, 回 S,<n>,<id>...
 *   Z / Z,<slot>               设零(当前位置=0)
 *   M,<slot>,<mode>,<value>    0停 1速度(rpm*10) 2角度(deg*10) 3圈数(turns*100)
 *   P,<slot>,<id>,<value>      设参数(浮点*100)
 *   R,<slot>                   恢复默认参数
 *   D,<slot>                   方向取反
 *   SAVE                       -> SAVED
 *   W,<turns*100>              拉簧预备圈数
 *   Y,<mode>                   yaw 0手动 1自瞄 2制导
 *   C,<x>,<center>             注入视觉坐标
 *   V,<a>,<b>                  舵机 0设标准 1设预备 2去标准 3去预备 4直接角 5取零
 *   G,<cmd>                    0拉簧标准 1拉簧预备 2舵机标准 3舵机预备
 *                              10自动开始 11停止 12急停 13解除
 *   H                          心跳(仅刷新超时)
 *
 * STM32 -> ESP32:
 *   F,<n>,<每电机12项>*n,<舵机4>,<任务4>,<视觉4>,<每电机11参数>*n
 *     电机12: slot,type,id,online,dir,mode,target100,rpm,angle100,turns100,temp,cur
 */
#include "dart_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_usart.h"
#include "dart_fsm.h"
#include "dart_motor.h"
#include "dart_servo.h"
#include "dart_store.h"
#include "dart_vision.h"
#include "main.h"
#include "robot_config.h"
#include "simple_motor.h"

static USARTInstance* s_usart = NULL;
static uint32_t s_last_cmd = 0;

static uint8_t s_scan_active = 0;
static uint32_t s_scan_t0 = 0;

static int SplitInts(const char* s, long* out, int max) {
  int n = 0;
  const char* p = s;
  while (*p && n < max) {
    char* end = NULL;
    long v = strtol(p, &end, 10);
    if (end == p) break;
    out[n++] = v;
    p = end;
    if (*p == ',') p++;
    else break;
  }
  return n;
}

static void DartDecode(void) {
  char* buf = (char*)s_usart->recv_buff;
  s_last_cmd = HAL_GetTick();

  if (buf[0] == 'P' && buf[1] == 'I' && buf[2] == 'N' && buf[3] == 'G') {
    DartLinkSendBlocking("PONG\n");
    return;
  }
  if (buf[0] == 'S' && buf[1] == 'A' && buf[2] == 'V' && buf[3] == 'E') {
    DartStoreMarkDirty();
    return;
  }

  char cmd = buf[0];
  long a[4] = {0};
  int n = (buf[1] == ',') ? SplitInts(buf + 2, a, 4) : 0;

  switch (cmd) {
    case 'S':
      if (!g_dart_scanning) s_scan_active = 1;
      break;
    case 'Z':
      DartMotorZero(n > 0 ? (int)a[0] : -1);
      DartStoreMarkDirty();
      break;
    case 'M':
      if (n >= 3) {
        float v = 0.0f;
        if (a[1] == 1) v = (float)a[2] / 10.0f;
        else if (a[1] == 2) v = (float)a[2] / 10.0f;
        else if (a[1] == 3) v = (float)a[2] / 100.0f;
        DartMotorSetCtrl((int)a[0], (int)a[1], v);
      }
      break;
    case 'P':
      if (n >= 3) {
        DartMotorSetParam((int)a[0], (int)a[1], (int)a[2]);
        DartStoreMarkDirty();
      }
      break;
    case 'R':
      if (n >= 1) {
        DartMotorResetParams((int)a[0]);
        DartStoreMarkDirty();
      }
      break;
    case 'D':
      if (n >= 1) {
        DartMotorToggleDir((int)a[0]);
        DartStoreMarkDirty();
      }
      break;
    case 'W':
      if (n >= 1) {
        g_spring_turns = (float)a[0] / 100.0f;
        if (g_spring_turns < 0) g_spring_turns = 0;
        if (g_spring_turns > DART_SPRING_MAX_TURNS) g_spring_turns = DART_SPRING_MAX_TURNS;
        DartStoreMarkDirty();
      }
      break;
    case 'Y':
      if (n >= 1) {
        g_yaw_mode = (int)a[0];
        DartStoreMarkDirty();
      }
      break;
    case 'C':
      if (n >= 1) DartVisionSet((int)a[0], n >= 2 ? (int)a[1] : 0);
      break;
    case 'V':
      if (n >= 1) {
        switch (a[0]) {
          case 0: DartServoSetPos(DART_SERVO_STD, (float)a[1] / 10.0f); DartStoreMarkDirty(); break;
          case 1: DartServoSetPos(DART_SERVO_PREP, (float)a[1] / 10.0f); DartStoreMarkDirty(); break;
          case 2: DartServoGo(DART_SERVO_STD); break;
          case 3: DartServoGo(DART_SERVO_PREP); break;
          case 4: DartServoSetDeg((float)a[1] / 10.0f); break;
          case 5: DartServoZero(); DartStoreMarkDirty(); break;
          default: break;
        }
      }
      break;
    case 'G':
      if (n >= 1) {
        switch (a[0]) {
          case 0: DartFsmSpringGo(0); break;
          case 1: DartFsmSpringGo(1); break;
          case 2: DartFsmServoGo(DART_SERVO_STD); break;
          case 3: DartFsmServoGo(DART_SERVO_PREP); break;
          case 10: DartFsmAutoStart(); break;
          case 11: DartFsmAutoStop(); break;
          case 12: DartFsmSetEstop(1); break;
          case 13: DartFsmSetEstop(0); break;
          default: break;
        }
      }
      break;
    default:
      break;
  }
}

static void ScanTask(void) {
  if (!s_scan_active) return;
  if (s_scan_t0 == 0) {
    s_scan_t0 = HAL_GetTick();
    g_dart_scanning = 1;
    SimpleMotorScanBegin();
    return;
  }
  SimpleMotorScanTask();
  if (HAL_GetTick() - s_scan_t0 >= 1000u) {
    uint16_t ids[8];
    uint8_t n = SimpleMotorScanResult(ids, 8);
    static char buf[96];
    int len = snprintf(buf, sizeof(buf), "S,%d", n);
    for (uint8_t i = 0; i < n && len < (int)sizeof(buf) - 8; i++)
      len += snprintf(buf + len, sizeof(buf) - len, ",%d", ids[i]);
    buf[len++] = '\n';
    buf[len] = 0;
    DartLinkSendBlocking(buf);
    SimpleMotorReconfigFilters();
    g_dart_scanning = 0;
    s_scan_active = 0;
    s_scan_t0 = 0;
  }
}

static void SendTelemetry(void) {
  static char buf[960];
  int len = snprintf(buf, sizeof(buf), "F,%d", DART_MOTOR_COUNT);

  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    SimpleMotor_t* m = &g_motors[i];
    DartCtrl_t* c = &g_dart_ctrl[i];
    int mode = (int)c->mode;
    int target100;
    if (mode == DART_MODE_SPEED) target100 = (int)(c->target * 100.0f);
    else if (mode == DART_MODE_ANGLE) target100 = (int)(c->target * 100.0f);
    else if (mode == DART_MODE_TURNS) target100 = (int)(c->target * 100.0f);
    else target100 = 0;
    len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", i,
                    (int)m->type, (int)m->id, (int)c->online, (int)c->reverse, mode, target100,
                    (int)m->rpm, (int)(DartMotorOutAngle(i) * 100.0f),
                    (int)(DartMotorOutTurns(i) * 100.0f), (int)m->temp, (int)m->current);
  }

  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d", (int)(g_servo_cur_deg * 10.0f),
                  g_servo_state, (int)(g_servo_std_deg * 10.0f), (int)(g_servo_prep_deg * 10.0f));
  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d", (int)(g_spring_turns * 100.0f),
                  g_task_step, g_yaw_mode, g_estop);
  len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d", g_vis_x, g_vis_ok, g_vis_center,
                  g_vis_err);

  for (int i = 0; i < DART_MOTOR_COUNT; i++) {
    DartParam_s* p = &g_dart_params[i];
    len += snprintf(buf + len, sizeof(buf) - len, ",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                    (int)(p->speed_ff * 100), (int)(p->speed_kp * 100), (int)(p->speed_ki * 100),
                    (int)(p->speed_i_limit * 100), (int)(p->angle_kp * 100), (int)(p->angle_kd * 100),
                    (int)(p->angle_slew * 100), (int)(p->gear_ratio * 100),
                    (int)(p->angle_deadband * 100), (int)(p->angle_speed_max * 100), (int)p->max_value);
  }

  buf[len++] = '\n';
  buf[len] = 0;
  DartLinkSend(buf);
}

void DartLinkInit(void) {
  USART_Init_Config_s cfg = {
      .recv_buff_size = DART_RECV_SIZE,
      .usart_handle = DART_UART_HANDLE,
      .module_callback = DartDecode,
  };
  s_usart = USARTRegister(&cfg);
}

void DartLinkTask(void) {
  ScanTask();

  if (DartStoreTakeSaved()) DartLinkSendBlocking("SAVED\n");

  static uint8_t timed_out = 0;
  if (s_last_cmd != 0 && HAL_GetTick() - s_last_cmd > DART_CMD_TIMEOUT_MS) {
    if (!timed_out) {
      timed_out = 1;
      DartMotorStopAll();
      DartFsmAutoStop();
    }
  } else {
    timed_out = 0;
  }

  static uint32_t last_fb = 0;
  if (HAL_GetTick() - last_fb >= DART_FB_PERIOD_MS) {
    last_fb = HAL_GetTick();
    SendTelemetry();
  }
}

void DartLinkSend(const char* s) {
  if (s_usart == NULL || s == NULL) return;
  USARTSend(s_usart, (uint8_t*)s, (uint16_t)strlen(s), USART_TRANSFER_IT);
}

void DartLinkSendBlocking(const char* s) {
  if (s_usart == NULL || s == NULL) return;
  uint32_t t0 = HAL_GetTick();
  while (s_usart->usart_handle->gState != HAL_UART_STATE_READY) {
    if (HAL_GetTick() - t0 > 50) break;
  }
  USARTSend(s_usart, (uint8_t*)s, (uint16_t)strlen(s), USART_TRANSFER_BLOCKING);
}
