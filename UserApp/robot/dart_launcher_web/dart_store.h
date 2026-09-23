/*
 * dart_store.h — 参数掉电保存 (STM32F407 内部 Flash, sector 11)
 */
#ifndef DART_LAUNCHER_WEB_DART_STORE_H
#define DART_LAUNCHER_WEB_DART_STORE_H

void DartStoreInit(void);
void DartStoreTask(void);
void DartStoreMarkDirty(void);

#endif  // DART_LAUNCHER_WEB_DART_STORE_H
