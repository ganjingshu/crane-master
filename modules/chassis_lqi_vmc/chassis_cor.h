#ifndef __CHASSIS_COR_H__ // 防止重复包含
#define __CHASSIS_COR_H__

#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"
#include "message_center.h"
#include "robot_def.h"
#include "message_center.h"
#include "robot_def.h"
#include "message_center.h"
#include "robot_def.h"
#include <math.h>

// ==================== 1. 常量与矩阵定义 ====================
#define DT 0.005f            // 控制周期 5ms
#define R  0.075f            // 麦轮半径 (m)
#define LXY 0.695f            // 轮距加轴距 (lx + ly)

// ==================== VMC 虚拟参数 ====================
// 1. 先定义基础的虚拟质量 (M) 和 响应刚度 (Kp)
#define M_VIRT_X 19.0f  // X轴虚拟质量 (kg，约等于整车重量)
#define KP_X 40.0f      // X轴响应刚度 (越大越跟手，太大会抽搐)

#define M_VIRT_Y 19.0f  // Y轴虚拟质量
#define KP_Y 40.0f      // Y轴响应刚度

#define M_VIRT_Z 2.0f   // Z轴自转虚拟转动惯量 (比平移小很多)
#define KP_Z 40.0f      // Z轴响应刚度

// 2. 使用临界阻尼公式自动计算 Kd (保证绝对不震荡的最佳手感)
// 注意：宏定义里直接写公式，编译器在编译时会自动算出常数，不会占用单片机算力
#define KD_X (2.0f * sqrtf(M_VIRT_X * KP_X))  // 算出来约等于 42.4f
#define KD_Y (2.0f * sqrtf(M_VIRT_Y * KP_Y))  // 算出来约等于 42.4f
#define KD_Z (2.0f * sqrtf(M_VIRT_Z * KP_Z))  // 算出来约等于 8.9f       

#define k_zhuanju 0.3 //转矩常数，单位是N·m/A

// LQI 增益矩阵 (从 MATLAB 脚本直接复制过来)
extern const float Kp[4][4];

extern const float Ki[4][4];

// ==================== 3. 全局状态变量声明 ====================
extern float V_cmd[3];     // 遥控器期望速度 [Vx, Vy, Wz]
extern float V_virt[3];    // 虚拟底盘妥协速度
extern float V_real[3];    // 真实底盘速度反馈

extern float w_real[4];    // 4个轮子的真实转速 (从CAN接收的编码器数据中解析)
extern float w_target[4];  // LQI的目标跟踪转速
extern float err_int[4];   // LQI积分累加器
extern float torque[4];    // LQI输出的力矩
extern int16_t current_cmd[4]; // 最终下发给电调的电流值



void TIM_Chassis_Control_Loop(Chassis_Ctrl_Cmd_s *cmd);                    // 初始化函数

#endif