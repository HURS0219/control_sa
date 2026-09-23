/*
 * dart_vision.h — 绿光中心坐标输入 (OpenMV -> C 板)
 *
 * v0.1 先用网页/串口注入坐标做联调, SPI 读 OpenMV 留接口 (DartVisionSet)。
 */
#ifndef DART_LAUNCHER_WEB_DART_VISION_H
#define DART_LAUNCHER_WEB_DART_VISION_H

#include <stdint.h>

extern int g_vis_x;       // 绿光中心水平坐标 (像素)
extern int g_vis_center;  // 画面中心
extern int g_vis_ok;      // 1 = 当前帧有效
extern int g_vis_err;     // x - center

void DartVisionInit(void);
void DartVisionSet(int x, int center);  // 视觉帧到达时调用
void DartVisionLost(void);              // 显式丢目标
void DartVisionTask(void);

#endif  // DART_LAUNCHER_WEB_DART_VISION_H
