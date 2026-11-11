#pragma once
#include <stdint.h>
typedef uint32_t TickType_t; typedef uint32_t StackType_t; typedef struct { int x; } StaticTask_t;
#define M2T(x) (x)
#define taskENTER_CRITICAL()
#define taskEXIT_CRITICAL()
static inline TickType_t xTaskGetTickCount(void) { return 0; }
static inline void vTaskDelayUntil(TickType_t* w, TickType_t d) { (void)w; (void)d; }
static inline void* xTaskCreateStatic(void (*f)(void*), const char* n, int s, void* a, int p, StackType_t* st, StaticTask_t* t) { (void)f;(void)n;(void)s;(void)a;(void)p;(void)st;(void)t; return 0; }
static inline unsigned uxTaskGetStackHighWaterMark(void* t) { (void)t; return 1000; }
