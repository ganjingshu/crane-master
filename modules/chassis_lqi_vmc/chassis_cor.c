#include "chassis_cor.h"
#include "dji_motor.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "chassis.h"

extern DJIMotorInstance *chassis_motors[4];

// LQI 增益矩阵定义
const float Kp[4][4] = {
    {0.408257, -0.048682, -0.048682, -0.048682},
    {-0.048682, 0.408257, -0.048682, -0.048682},
    {-0.048682, -0.048682, 0.408257, -0.048682},
    {-0.048682, -0.048682, -0.048682, 0.408257}
};

const float Ki[4][4] = {
    {2.932486, -0.587842, -0.587842, -0.587842},
    {-0.587842, 2.932486, -0.587842, -0.587842},
    {-0.587842, -0.587842, 2.932486, -0.587842},
    {-0.587842, -0.587842, -0.587842, 2.932486}
};

// const float Kp[4][4] = {
//     {0.113870, -0.012396, -0.012396, -0.012396},
//     {-0.012396, 0.113870, -0.012396, -0.012396},
//     {-0.012396, -0.012396, 0.113870, -0.012396},
//     {-0.012396, -0.012396, -0.012396, 0.113870}
// };

// const float Ki[4][4] = {
//     {2.669581, -0.464343, -0.464343, -0.464343},
//     {-0.464343, 2.669581, -0.464343, -0.464343},
//     {-0.464343, -0.464343, 2.669581, -0.464343},
//     {-0.464343, -0.464343, -0.464343, 2.669581}
// };
// ==================== 全局状态变量定义 ====================
float V_cmd[3] = {0};     // 遥控器期望速度 [Vx, Vy, Wz]
float V_virt[3] = {0};    // 虚拟底盘妥协速度
float V_real[3] = {0};    // 真实底盘速度反馈

float w_real[4] = {0};    // 4个轮子的真实转速 (从CAN接收的编码器数据中解析)
float w_target[4] = {0};  // LQI的目标跟踪转速
float err_int[4] = {0};   // LQI积分累加器
float torque[4] = {0};    // LQI输出的力矩
int16_t current_cmd[4] = {0}; // 最终下发给电调的电流值

void TIM_Chassis_Control_Loop(Chassis_Ctrl_Cmd_s *cmd) {
    
    // ---------------------------------------------------------
    // 步骤 0：获取遥控器控制命令
    // ---------------------------------------------------------
            // 将遥控器命令转换为底盘速度命令
            // 注意：这里需要根据实际遥控器映射关系进行调整
            V_cmd[0] = cmd->vx;  // X方向速度
            V_cmd[1] = cmd->vy;  // Y方向速度  
            V_cmd[2] = cmd->wz;  // 旋转角速度
    
    
    // ---------------------------------------------------------
    // 步骤 1：读取编码器并进行正运动学解算 (获取反馈)
    // 注意：w_real必须是 rad/s。如果读回来是 rpm，需要换算: rad/s = rpm * 2PI / 60
    // ---------------------------------------------------------
    const float GEAR_RATIO = 19.2032f;
    w_real[0] =  chassis_motors[0]->measure.speed_aps * DEGREE_2_RAD / GEAR_RATIO;
    w_real[1] =  chassis_motors[1]->measure.speed_aps * DEGREE_2_RAD / GEAR_RATIO;
    w_real[2] =  -chassis_motors[2]->measure.speed_aps * DEGREE_2_RAD / GEAR_RATIO; // [修复] 直接在此取反
    w_real[3] =  -chassis_motors[3]->measure.speed_aps * DEGREE_2_RAD / GEAR_RATIO; // [修复] 直接在此取反

// 步骤 1：读取编码器并进行正运动学解算 (获取反馈)
    V_real[0] = (R / 4.0f) * ( w_real[0] + w_real[1] + w_real[2] + w_real[3]); // Vx
    V_real[1] = (R / 4.0f) * (-w_real[0] + w_real[1] - w_real[2] + w_real[3]); // Vy
    V_real[2] = (R / (4.0f * LXY)) * (-w_real[0] - w_real[1] + w_real[2] + w_real[3]); // Wz
    
    // ---------------------------------------------------------
    // 步骤 2：上层 VMC 导纳控制模型计算 (以 X 轴为例)
    // ---------------------------------------------------------
    float a_virt_x = (KP_X * (V_cmd[0] - V_virt[0]) - KD_X * (V_virt[0] - V_real[0])) / M_VIRT_X;
    float a_virt_y = (KP_Y * (V_cmd[1] - V_virt[1]) - KD_Y * (V_virt[1] - V_real[1])) / M_VIRT_Y;
    float a_virt_z = (KP_Z * (V_cmd[2] - V_virt[2]) - KD_Z * (V_virt[2] - V_real[2])) / M_VIRT_Z;
    
    V_virt[0] += a_virt_x * DT; // 积分更新 X 轴虚拟速度
    V_virt[1] += a_virt_y * DT; // 积分更新 Y 轴虚拟速度
    V_virt[2] += a_virt_z * DT; // 积分更新 Z 轴虚拟速度

    // ---------------------------------------------------------
    // 步骤 3：虚拟主轴逆运动学 (目标下发)
    // ---------------------------------------------------------
    w_target[0] = (1.0f / R) * (V_virt[0] - V_virt[1] - LXY * V_virt[2]);
    w_target[1] = (1.0f / R) * (V_virt[0] + V_virt[1] - LXY * V_virt[2]);
    w_target[2] = (1.0f / R) * (V_virt[0] - V_virt[1] + LXY * V_virt[2]);
    w_target[3] = (1.0f / R) * (V_virt[0] + V_virt[1] + LXY * V_virt[2]);

    // ---------------------------------------------------------    
    // 步骤 4：底层 LQI 力矩解算 (全连接矩阵乘法)
    // ---------------------------------------------------------
float e[4]; // 临时存储当前周期的误差
    
    // [修复] 循环1：先更新所有的误差和积分状态，保证数据在同一时间戳
    for (int i = 0; i < 4; i++) {
        e[i] = w_target[i] - w_real[i];
        err_int[i] += e[i] * DT;
        
        // 积分抗饱和处理
        if (err_int[i] > 1.5f) err_int[i] = 1.5f;
        if (err_int[i] < -1.5f) err_int[i] = -1.5f;
    }
    
    // [修复] 循环2：再执行 4x4 矩阵运算
    for (int i = 0; i < 4; i++) {
        torque[i] = 0.0f;
        for (int j = 0; j < 4; j++) {
            torque[i] += Kp[i][j] * e[j] + Ki[i][j] * err_int[j];
        }
        
        // 物理力矩限幅 (轮端力矩限幅)
        if (torque[i] > 3.0f) torque[i] = 3.0f;   // 3.0N.m 在轮端太小了，这里稍微放大一点，具体根据实际调整
        if (torque[i] < -3.0f) torque[i] = -3.0f;
    }


// ---------------------------------------------------------
    // 步骤 5：电流转换与下发
    // ---------------------------------------------------------
    for (int i = 0; i < 4; i++) {
        int16_t current = (int16_t)(torque[i] * (16384.0f / 20.0f) / k_zhuanju);
        
        // if (i == 2 || i == 3) {
        //     current = -current;
        // }
        chassis_motors[i]->motor_controller.pid_ref = (float)current;
    }
        //     int16_t current = (int16_t)(torque[2] * (16384.0f / 20.0f) / k_zhuanju);
        // chassis_motors[2]->motor_controller.pid_ref = (float)current;
}