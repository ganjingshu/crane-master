/**
 * @file chassis.c
 * @author NeoZeng neozng1@hnu.edu.cn
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向
 *
 * @version 0.1
 * @date 2022-12-04
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "chassis.h"
#include "robot_def.h"
#include "power_control.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"

#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "dji_motor.h"
#include <stdbool.h>

#include "shoot.h" 

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm; // 双板通信CAN comm
attitude_t *Chassis_IMU_data;
#endif // CHASSIS_BOARD
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif                                              // !ONE_BOARD
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static PIDInstance buffer_PID;             // 用于底盘的缓冲能量PID
static PIDInstance chassis_follow_PID;     // 用于底盘跟随云台的PID控制
static PID_Init_Config_s chassis_follow_pid_conf; // 底盘跟随云台PID配置
static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI


// 添加发射模块订阅者
static Subscriber_t *shoot_sub; // 发射模块订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 发射控制命令

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back
//motor_l为x方向，motor_r为y方向

//调试变量
static float chassis_lb_v=0; // 底盘lb电机速度
static float chassis_rb_v=0; // 底盘rb电机速度
float chassis_lb_speed=400;

static float chassis_pid_output[4];
static float chassis_pid_totaloutput;
static DJIMotorInstance *chassis_motor_instance[4]; 

static float chassis_power_limit,chassis_input_power,chassis_power_buffer;//裁判系统获取的功率限制值、当前功率值、当前缓冲能量值
static float chassis_power_max;//计算使用的最大功率值
static float chassis_power_offset = -5; // 功率冗余，可修改
static float output_zoom_coeff = 0;     // 输出缩放系数

// 交叉耦合同步补偿器参数
#define CHASSIS_SYNC_KP 0.3f       // 同步P增益，需根据实测调节，建议从0.1开始逐步增大
#define CHASSIS_SYNC_MAX 500.0f    // 同步补偿最大输出限幅，单位与speed_aps一致(度/秒)

//三个系数
float toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
float k1 = 1.26e-07;                      // k1，9.50000043e-08
float k2 = 1.95000013e-07;                // k2
float constant_coefficient = 3.5f;

//标示是否处在缓冲能量低状态
bool isLowBuffer = false;

// 辅助函数定义
static float float_Square(float x) { return x * x; }
static float abs_limit(float x, float limit) { return (x > limit) ? limit : ((x < -limit) ? -limit : x); }
 
#define CHASSIS_POWER_COFFICIENT (1 - (float)(120 - 45) / (float)(135 - 50)) // 这个量出现是因为我们的电机阻力较大，导致理论值和实际值相差较大，用于补偿



/* 用于自旋变速策略的时间变量 */
// static float t;

/* 私有函数计算的中介变量,设为静态避免参数传递的开销 */
static float chassis_vx, chassis_vy;                      // 将云台系的速度投影到底盘
static float vt_lf, vt_rf, vt_lb, vt_rb;                  // 底盘速度解算后的临时输出,待进行限幅

void ChassisInit()
{

    // 为每个电机创建独立的配置结构体

    Motor_Init_Config_s chassis_motor_config_lf = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 16, // 17
                .Ki = 4,   // 3.5
                .Kd = 0,   // 0
                .IntegralLimit = 5000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 10000,
                .Output_LPF_RC = 0.3,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .power_limit_flag = POWER_LIMIT_ON
        },
        .motor_type = M3508,
    };

    // 为motor_lb创建单独的配置，你可以在这里修改PID参数
    Motor_Init_Config_s chassis_motor_config_lb = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 16,   // 单独为motor_lb设置不同的Kp值
                .Ki = 4, // 单独为motor_lb设置不同的Ki值
                
                .Kd = 0, // 单独为motor_lb设置不同的Kd值
                .IntegralLimit = 5000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 10000,
                .Output_LPF_RC = 0.3,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .power_limit_flag = POWER_LIMIT_ON
        },
        .motor_type = M3508,
    };

    Motor_Init_Config_s chassis_motor_config_rb = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 16, // 17
                .Ki = 4,   // 3.5
                .Kd = 0,   // 0
                .IntegralLimit = 5000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 10000,
                .Output_LPF_RC = 0.3,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .power_limit_flag = POWER_LIMIT_ON
        },
        .motor_type = M3508,
    };

    Motor_Init_Config_s chassis_motor_config_rf = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 16, // 17
                .Ki = 4,   // 3.5
                .Kd = 0,   // 0
                .IntegralLimit = 5000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 10000,
                .Output_LPF_RC = 0.3,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .power_limit_flag = POWER_LIMIT_ON
        },
        .motor_type = M3508,
    };

    //  @todo: 当前还没有设置电机的正反转,仍然需要手动添加reference的正负号,需要电机module的支持,待修改.
    //使用功率控制的电机需要使用PowerControlInit()函数初始化,因为电机的控制方式不同

    chassis_motor_config_lf.can_init_config.tx_id = 3;
    chassis_motor_config_lf.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = PowerControlInit(&chassis_motor_config_lf);

    chassis_motor_config_lb.can_init_config.tx_id = 2;
    chassis_motor_config_lb.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lb = PowerControlInit(&chassis_motor_config_lb);
    
    chassis_motor_config_rb.can_init_config.tx_id = 1;
    chassis_motor_config_rb.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = PowerControlInit(&chassis_motor_config_rb);

    chassis_motor_config_rf.can_init_config.tx_id = 4;
    chassis_motor_config_rf.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_rf = PowerControlInit(&chassis_motor_config_rf);




    

    // 发布订阅初始化,如果为双板,则需要can comm来传递消息（但目前是单板方案）
#ifdef CHASSIS_BOARD
    Chassis_IMU_data = INS_Init(); // 底盘IMU初始化

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    chasiss_can_comm = CANCommInit(&comm_conf); // can comm初始化
#endif                                          // CHASSIS_BOARD

#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    // 添加发射模块订阅者初始化
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
#endif // ONE_BOARD
}

#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 *        用宏进行预替换减小开销,运动解算具体过程参考教程
 */
static void MecanumCalculate()
{
    vt_lf = chassis_vx;
    vt_rf = chassis_vy;
    vt_lb = chassis_vx;
    vt_rb = chassis_vy;
}


/**
 * @brief 交叉耦合同步补偿器
 *        计算同侧前后电机的速度差，通过P增益将补偿量反补回目标速度，
 *        使前后电机相互靠拢，提升同步性。
 *        左侧(lf/lb)为X方向，右侧(rf/rb)为Y方向。
 */
static void ChassisSyncCompensate()
{
    // 左侧前后电机速度差 (lf - lb)，单位:度/秒
    float sync_err_l = motor_lf->measure.speed_aps - motor_lb->measure.speed_aps;
    // 右侧前后电机速度差 (rf - rb)，单位:度/秒
    float sync_err_r = motor_rf->measure.speed_aps - motor_rb->measure.speed_aps;

    // P增益补偿量，含限幅防止补偿过大
    float compensate_l = abs_limit(sync_err_l * CHASSIS_SYNC_KP, CHASSIS_SYNC_MAX);
    float compensate_r = abs_limit(sync_err_r * CHASSIS_SYNC_KP, CHASSIS_SYNC_MAX);

    // 快的减速，慢的加速，相互靠拢
    vt_lf -= compensate_l;
    vt_lb += compensate_l;
    vt_rf -= compensate_r;
    vt_rb += compensate_r;
}

/**
 * @brief 对功率进行缩放，参考西交利物浦的方案（但目前并不是使用西交利物浦的方案进行功率限制）
 *
 */
static void LimitChassisOutput()
{
    chassis_pid_totaloutput = 0;
    chassis_power_limit = referee_data->GameRobotState.chassis_power_limit; // 从裁判系统获取的能量限制

    chassis_input_power = referee_data->PowerHeatData.chassis_power;
    chassis_power_buffer = referee_data->PowerHeatData.buffer_energy;

    // if (chassis_power_limit >= 100)
    // {
    //     chassis_power_limit = 100;
    // }

  //  chassis_power_limit=50;

    // 根据缓冲能量和当前功率限制，计算最大功率值
    chassis_power_offset = -1 * CHASSIS_POWER_COFFICIENT * (chassis_power_limit)-0;

    chassis_power_max = chassis_power_limit + chassis_power_offset;



    if (isLowBuffer)
    {
        chassis_power_max = chassis_power_max - 25;
        if (chassis_power_buffer >= 55.0f)
        {
            isLowBuffer = false;
        }
    }
    else
    {
        // 缓冲能量判断，如果缓冲能量少，则马上减小功率，减少量待测
        if (chassis_power_buffer < 10.0f)
        {
            isLowBuffer = true;
            chassis_power_max = chassis_power_max - 35;
        }
    }

  

    if (chassis_pid_totaloutput > chassis_power_max) // 超出功率
    {
        output_zoom_coeff = chassis_power_max / chassis_pid_totaloutput;
        for (int i = 0; i < 4; i++)
        {
            chassis_pid_output[i] *= output_zoom_coeff;
            if (chassis_pid_output[i] < 0)
            {
                continue;
            }

            //公式法解力矩功率与实际电机功率的一元二次方程
            float a = k1;
            float b = toque_coefficient * chassis_motor_instance[i]->measure.speed_rpm;
            float c = k2 * float_Square(chassis_motor_instance[i]->measure.speed_rpm) - chassis_pid_output[i] + constant_coefficient;
            // k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_give_power[i] + constant;

            if (chassis_motor_instance[i]->motor_controller.pid_output > 0)
            {
                float temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
                DJIMotorSetOutputLimit(chassis_motor_instance[i], abs_limit(temp, 13000));
            }
            else
            {
                float temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
                DJIMotorSetOutputLimit(chassis_motor_instance[i], abs_limit(temp, 13000));
            }
        }
    }
    else
    {
        for (int i = 0; i < 4; i++)
            DJIMotorSetOutputLimit(chassis_motor_instance[i], chassis_motor_instance[i]->motor_controller.pid_output);
    }
}


/**
 * @brief 设定底盘电机速度参考值
 *
 */
static void ChassisSetRef()
{
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 根据每个轮子的速度反馈,计算底盘的实际运动速度,逆运动解算
 *        对于双板的情况,考虑增加来自底盘板IMU的数据
 *
 */
static void EstimateSpeed()
{
    // 根据电机速度和陀螺仪的角速度进行解算,还可以利用加速度计判断是否打滑(如果有)
    // chassis_feedback_data.vx vy wz =
    //  ...
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 后续增加没收到消息的处理(双板的情况)
    // 获取新的控制信息
#ifdef ONE_BOARD
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
#endif
#ifdef CHASSIS_BOARD
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(chasiss_can_comm);
#endif // CHASSIS_BOARD



    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    { // 如果出现重要模块离线或遥控器设置为急停,让电机停止（电流为0）
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
    }
    else
    { // 正常工作
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
    }



    chassis_vx = chassis_cmd_recv.vx;
    chassis_vy = chassis_cmd_recv.vy;

    // 根据控制模式进行正运动学解算,计算底盘输出
    MecanumCalculate();

    // 交叉耦合同步补偿，减小前后电机速度差
    ChassisSyncCompensate();

    // 设定底盘闭环参考值
    ChassisSetRef();

    // 根据裁判系统的反馈数据和电容数据对输出限幅并设定闭环参考值
   // LimitChassisOutput();

    // 根据电机的反馈速度和IMU(如果有)计算真实速度
    // EstimateSpeed();

    chassis_lb_v = motor_lb->measure.speed_aps/6.0;
    chassis_rb_v = motor_rb->measure.speed_aps/6.0;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif
#ifdef CHASSIS_BOARD
    CANCommSend(chasiss_can_comm, (void *)&chassis_feedback_data);
#endif // CHASSIS_BOARD
}