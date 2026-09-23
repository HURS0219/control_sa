/*
 * dart_store.h — 参数掉电保存 (STM32F407 内部 Flash, sector 11)
 */
#ifndef DART_LAUNCHER_WEB_V2_DART_STORE_H
#define DART_LAUNCHER_WEB_V2_DART_STORE_H

#include <stdint.h>

void DartStoreInit(void);
void DartStoreTask(void);
void DartStoreMarkDirty(void);
int DartStoreTakeSaved(void);  // 返回并清除“刚保存”标志

#endif  // DART_LAUNCHER_WEB_V2_DART_STORE_H
