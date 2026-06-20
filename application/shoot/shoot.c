#include "shoot.h"
#include "robot_def.h"

#include "servo_motor.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "remote_control.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *servo_l, *servo_r; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息


static Subscriber_t *vision_gimbal_sub;
static Subscriber_t *vision_recv_r_data_sub;
static Vision_Gimbal_Data_s vision_gimbal_data_recv;
static Vision_Recv_s vision_recv_data_2_shoot_r;

static uint8_t single_shot_triggered = 0;


// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
#define IS_HIBERNATED (hibernate_time + dead_time > DWT_GetTimeline_ms())

static int count=0;
int shoot1=40000;

void ShootInit()
{
    
    // 左摩擦轮
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 20, 
                .Ki = 10, 
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
            .current_PID = {
                .Kp = 1.0, 
                .Ki = 1.0,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 10000,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,

            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_REVERSE,
        },
        .motor_type = M2006};
    friction_config.can_init_config.tx_id = 3,//3
    friction_l = DJIMotorInit(&friction_config);
    //右摩擦轮
    friction_config.can_init_config.tx_id = 4; //4  // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    // 初始化舵机
Servo_Init_Config_s servo_config = {
    .servo_type      = PWM_Servo,    // 或 Bus_Servo
    .servo_id        = 2,
    .pwm_init_config = {
        .htim      = &htim1,
        .channel   = TIM_CHANNEL_1,
        .period    = 0.02f,
        .dutyratio = 0.075f,         // 初始位置 90°
    },
};

    servo_l = ServoInit(&servo_config);
    servo_r = ServoInit(&servo_config);


    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));//射击模块的发布
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));//射击模块的订阅
    vision_gimbal_sub = SubRegister("vision_gimbal_data", sizeof(Vision_Gimbal_Data_s));
    vision_recv_r_data_sub = SubRegister("vision_recv_data", sizeof(Vision_Recv_s));
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    // 从cmd获取控制数据
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    SubGetMessage(vision_gimbal_sub, &vision_gimbal_data_recv);
    SubGetMessage(vision_recv_r_data_sub, &vision_recv_data_2_shoot_r);
   // f_Loader_AntiBlock_Handle(loader, loader->measure.real_current);

    // 对shoot mode等于SHOOT_STOP的情况特殊处理,直接停止所有电机(紧急停止)
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        ServoSetAngle(servo_l, 0.0f);
        ServoSetAngle(servo_r, 0.0f);
    }
    else // 恢复运行
    {
        ServoSetAngle(servo_l, 270.0f);
        ServoSetAngle(servo_r, 270.0f);
    }


    if (shoot_cmd_recv.friction_mode == FRICTION_UP)
    {
        DJIMotorSetRef(friction_l, 5000);
        DJIMotorSetRef(friction_r, 5000);
    }
     else if (shoot_cmd_recv.friction_mode == FRICTION_DOWN)
    {
        DJIMotorSetRef(friction_l, -5000);
        DJIMotorSetRef(friction_r, -5000);
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }


    

  

    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}


