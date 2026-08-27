#ifndef utils_h
#define utils_h
#include <stm32f10x.h>

void Delay_us(uint16_t us);
void Delay_ms(uint16_t ms);

typedef enum {FALSE = 0, TRUE = !FALSE} BOOL;

#endif