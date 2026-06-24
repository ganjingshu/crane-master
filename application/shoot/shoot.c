#include "shoot.h"
#include "robot_def.h"

#include "servo_motor.h"
#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "remote_control.h"

/*
 * 四电机齿条主从同步:
 *   左齿条: l1=Master(位置环), l2=Slave(速度跟随l1)
 *   右齿条: r1=Master(位置环), r2=Slave(速度跟随r1)
 *   Master 负责轨迹跟踪, Slave 顺从跟随 + 位置微调防漂移
 */
static DJIMotorInstance *rack_ml, *rack_sl;  // 左齿条 Master(l1) + Slave(l2)
static DJIMotorInstance *rack_mr, *rack_sr;  // 右齿条 Master(r1) + Slave(r2)
static DJIMotorInstance *servo_l, *servo_r;

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data;

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

// dt 计算
static float rack_last_t = 0;

// 位置微调: 记录每对主从上电时的物理位置差, 防止长期漂移
// 物理位置: REVERSE电机 = -total_angle, NORMAL电机 = +total_angle
// 左齿条: l1(REVERSE)物理Pos=-tl1, l2(NORMAL)物理Pos=+tl2 → offset = tl2 + tl1
// 右齿条: r1(NORMAL)物理Pos=+tr1, r2(REVERSE)物理Pos=-tr2 → offset = tr1 + tr2
static float rack_offset_l, rack_offset_r;
static uint8_t rack_offset_inited = 0;

void ShootInit()
{
    // ==================== Master 配置 (位置环) ====================
    Motor_Init_Config_s master_config = {
        .can_init_config = { .can_handle = &hcan2 },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 60, 
                .Ki = 5, 
                .Kd = 3, 
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 2000, 
                .MaxOut = 8000,
            },
            .speed_PID = {
                .Kp = 10, 
                .Ki = 150, 
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 9000, 
                .MaxOut = 20000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
        },
        .motor_type = M2006,
    };

    // ==================== Slave 配置 (速度环, 跟随Master) ====================
    Motor_Init_Config_s slave_config = {
        .can_init_config = { .can_handle = &hcan2 },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 8,
                .Ki = 80, 
                .Kd = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 5000, .MaxOut = 15000,
            },
            // other_speed_feedback_ptr 和 speed_feedforward_ptr 在 Master 创建后设置
        },
        .controller_setting_init_config = {
            .speed_feedback_source = OTHER_FEED,       // 跟踪 Master 实际速度
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .feedforward_flag = SPEED_FEEDFORWARD,      // Master 速度前馈
        },
        .motor_type = M2006,
    };

    // ---- 左齿条: l1=Master (tx_id=3, REVERSE), l2=Slave (tx_id=1, NORMAL) ----
    master_config.can_init_config.tx_id = 3;
    master_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    rack_ml = DJIMotorInit(&master_config);  // Master 先创建

    slave_config.can_init_config.tx_id = 1;
    slave_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    // Slave 的反馈和前馈指针 → 指向刚创建的 Master
    slave_config.controller_param_init_config.other_speed_feedback_ptr = &rack_ml->measure.speed_aps;
    slave_config.controller_param_init_config.speed_feedforward_ptr    = &rack_ml->measure.speed_aps;
    rack_sl = DJIMotorInit(&slave_config);

    // ---- 右齿条: r1=Master (tx_id=4, NORMAL), r2=Slave (tx_id=2, REVERSE) ----
    master_config.can_init_config.tx_id = 4;
    master_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    rack_mr = DJIMotorInit(&master_config);

    slave_config.can_init_config.tx_id = 2;
    slave_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    slave_config.controller_param_init_config.other_speed_feedback_ptr = &rack_mr->measure.speed_aps;
    slave_config.controller_param_init_config.speed_feedforward_ptr    = &rack_mr->measure.speed_aps;
    rack_sr = DJIMotorInit(&slave_config);

    // 初始化舵机
    Servo_Init_Config_s servo_config = {
        .servo_type      = PWM_Servo,
        .servo_id        = 2,
        .pwm_init_config = {
            .htim      = &htim1,
            .channel   = TIM_CHANNEL_1,
            .period    = 0.02f,
            .dutyratio = 0.075f,
        },
    };
    servo_l = ServoInit(&servo_config);
    servo_r = ServoInit(&servo_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    vision_gimbal_sub = SubRegister("vision_gimbal_data", sizeof(Vision_Gimbal_Data_s));
    vision_recv_r_data_sub = SubRegister("vision_recv_data", sizeof(Vision_Recv_s));
}

/**
 * @brief 齿条主从同步设定
 *
 *   Master: 位置环, 闭环增量模式跟踪虚拟轴
 *     REVERSE电机 (l1): ref = delta - total_angle  →  error = -delta  →  反转
 *     NORMAL 电机 (r1): ref = total_angle + delta   →  error = +delta  →  正转
 *
 *   Slave: 速度环, 跟随 Master 实际速度 + 位置微调 (防漂移)
 *     速度来源:  other_speed_feedback_ptr 指向 Master->speed_aps
 *     前馈:      speed_feedforward_ptr  = Master->speed_aps
 *     位置微调:  检测物理位置偏差, 小增益修正 Slave 的速度参考
 *
 *   l1(REVERSE)↔l2(NORMAL) 同向: l2速度目标 = -l1速度 (物理同向)
 *   r1(NORMAL)↔r2(REVERSE) 同向: r2速度目标 = r1速度 (取反后物理同向)
 */
static void RackSetRefWithSync(float base_ref)
{
    float now = DWT_GetTimeline_ms();
    float dt = (rack_last_t == 0) ? 0.005f : (now - rack_last_t) * 0.001f;
    rack_last_t = now;

    // ---- 首次运行, 记录主从物理位置偏移 ----
    if (!rack_offset_inited)
    {
        rack_offset_l = rack_sl->measure.total_angle + rack_ml->measure.total_angle;
        rack_offset_r = rack_mr->measure.total_angle + rack_sr->measure.total_angle;
        rack_offset_inited = 1;
    }

    // ==================== 停止 ====================
    if (base_ref == 0)
    {
        // Masters: 锁定当前位置
        DJIMotorSetRef(rack_ml, -rack_ml->measure.total_angle);          // REVERSE
        DJIMotorSetRef(rack_mr,  rack_mr->measure.total_angle);          // NORMAL
        // Slaves: 速度目标 = 0 (Master 速度为 0, 前馈 = 0)
        DJIMotorSetRef(rack_sl, 0);
        DJIMotorSetRef(rack_sr, 0);
        return;
    }

    // ==================== 运行 ====================
    float delta = base_ref * dt;  // 周期位置增量

    // -- Masters: 闭环增量位置跟踪 --
    DJIMotorSetRef(rack_ml, delta - rack_ml->measure.total_angle);       // REVERSE
    DJIMotorSetRef(rack_mr, rack_mr->measure.total_angle + delta);       // NORMAL

    // -- Slaves: 速度跟随 Master + 位置微调 --
    // 位置微调: 检测物理位置漂移, P 增益柔和修正
    float drift_l = (rack_sl->measure.total_angle + rack_ml->measure.total_angle) - rack_offset_l;
    float drift_r = (rack_mr->measure.total_angle + rack_sr->measure.total_angle) - rack_offset_r;
    float trim_l = -drift_l * 0.2f;  // 小增益, 防漂移
    float trim_r = -drift_r * 0.2f;

    // Slave 速度参考 = 位置微调值 (Master 速度由前馈提供, 不重复加)
    DJIMotorSetRef(rack_sl, trim_l);  // NORMAL: 正 trim → 正向加速
    DJIMotorSetRef(rack_sr, trim_r);  // REVERSE: 正 trim → 取反后负向加速
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    SubGetMessage(shoot_sub, &shoot_cmd_recv);
    SubGetMessage(vision_gimbal_sub, &vision_gimbal_data_recv);
    SubGetMessage(vision_recv_r_data_sub, &vision_recv_data_2_shoot_r);

    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        ServoSetAngle(servo_l, 0.0f);
        ServoSetAngle(servo_r, 0.0f);
    }
    else
    {
        ServoSetAngle(servo_l, 270.0f);
        ServoSetAngle(servo_r, 270.0f);
    }

    if (shoot_cmd_recv.friction_mode == FRICTION_UP)
        RackSetRefWithSync(5000);
    else if (shoot_cmd_recv.friction_mode == FRICTION_DOWN)
        RackSetRefWithSync(-5000);
    else
        RackSetRefWithSync(0);

    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
