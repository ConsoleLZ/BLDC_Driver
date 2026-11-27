#ifndef __MAIN_H_
#define __MAIN_H_
#include <stm32f10x.h>

typedef struct {
    float rpm; // 当前电机的转速
} App;

extern App app;

#endif