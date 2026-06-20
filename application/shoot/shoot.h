#ifndef SHOOT_H
#define SHOOT_H

#include "dji_motor.h"

/**
 * @brief 发射初始化,会被RobotInit()调用
 *
 */
void ShootInit();

/**
 * @brief 发射任务,两个2006电机由视觉控制运动状态
 *
 */
void ShootTask();

#endif // SHOOT_H
