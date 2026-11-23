#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/* --------------------------- 电机控制对外接口 ---------------------------
 * 封装电机 / 传感器 / 驱动的初始化与主循环，main.cpp 只负责调用。
 * ----------------------------------------------------------------------*/

void motorControlSetup();   // 上电初始化：编码器、TIM2、FOC、开环预检、进闭环
void motorControlLoop();    // 主循环：暖机升速、斜坡限速、堵转与误码保护

#endif  // MOTOR_CONTROL_H
