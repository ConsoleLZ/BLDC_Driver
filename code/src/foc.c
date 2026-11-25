#include "foc.h"
#include <math.h>
#include "as5600.h"
#include "timer.h"

FOC_COORDINATES foc_coordinates;

// 获取电角度
uint16_t AS5600_ReadEAngle(void)
{
    uint16_t angle = AS5600_ReadAngle();
    uint16_t e_angle = (uint16_t)((angle * POLE_PAIRS) % PWM_PERIOD);

    return e_angle;
}

/**
 * Park反变换
 */

void Park_Reverse(void)
{
    uint16_t e_angle = AS5600_ReadEAngle();

    foc_coordinates.Ix = (Id * cos(e_angle)) - (Iq * sin(e_angle));
    foc_coordinates.Iy = (Iq * cos(e_angle)) + (Id * sin(e_angle));
}

/**
 * Clark反变换
 */

void Clark_Reverse(void)
{
}