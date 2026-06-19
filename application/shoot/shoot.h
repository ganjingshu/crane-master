#ifndef SHOOT_H
#define SHOOT_H

#include "dji_motor.h"
/**
 * @brief 发射初始化,会被RobotInit()调用
 * 
 */
void ShootInit();

/**
 * @brief 发射任务
 * 
 */
void ShootTask();

/**
 * @brief 拨弹电机防堵转处理函数
 * 
 */
void f_Loader_AntiBlock_Handle(DJIMotorInstance *loader, float current_ref);

#endif // SHOOT_H