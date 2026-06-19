#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "general_def.h"
#include "remote_control.h"

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 拨盘电机
// static servo_instance *lid; 需要增加弹舱盖

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv; // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 来自cmd的发射控制信息
static float speed_2006=0;
static float angle_2006=0;

static Subscriber_t *vision_gimbal_sub;
static Subscriber_t *vision_recv_r_data_sub;
static Vision_Gimbal_Data_s vision_gimbal_data_recv;
static Vision_Recv_s vision_recv_data_2_shoot_r;

static uint8_t single_shot_triggered = 0;


// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;
#define IS_HIBERNATED (hibernate_time + dead_time > DWT_GetTimeline_ms())
// 防堵转参数定义（根据实际情况调整，目前都还没有改）
#define LOADER_BLOCK_CURRENT_THRESHOLD    4000    // 堵转电流阈值 (3000 = 3A)（还没测）
#define LOADER_BLOCK_COUNT_THRESHOLD      50      // 连续堵转计数阈值
#define LOADER_REVERSE_ANGLE              45.0f   // 反转角度 (度)（看实际情况）
#define LOADER_STABLE_COUNT_THRESHOLD     50       // 电流稳定计数阈值

// 拨弹电机防堵转控制变量
static uint32_t loader_block_count = 0;           // 堵转计数
static uint32_t loader_stable_count = 0;          // 电流稳定计数
static float loader_original_ref = 0;            // 原始设定值
 uint8_t loader_is_reversing = 0;           // 反转标志

static uint32_t shoot_speed = 0; 
uint32_t shoot_speed_l = 0;

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
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 2,//2
    friction_l = DJIMotorInit(&friction_config);
    //右摩擦轮
    friction_config.can_init_config.tx_id = 4; //4  // 右摩擦轮,改txid和方向就行
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan2,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 20, // 10
                .Ki = 80,
                .Kd = 0.1,
                .MaxOut = 50000, 
                .Improve = PID_Integral_Limit | PID_DerivativeFilter,
                .IntegralLimit = 20000,
                .Derivative_LPF_RC =0,
                //.DeadBand=750,
            },
            .speed_PID = {
                .Kp = 8, //8, // 10
                .Ki = 1,//0.800000012, // 1
                .Kd = 0,// 0.0250000004,
                .Improve = PID_Integral_Limit | PID_DerivativeFilter,
                .IntegralLimit = 3000,
                .MaxOut = 9000,
                .Derivative_LPF_RC = 0.04,
            },
            .current_PID = {
                .Kp = 1, //5, // 0.7
                .Ki = 0.1, //0.3, // 0.1
                .Kd = 0,//0.00079999998,
                .Improve = PID_Integral_Limit | PID_DerivativeFilter,
                .IntegralLimit = 3000,
                .MaxOut = 10000,
                .Derivative_LPF_RC = 0.00125,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = ANGLE_LOOP,//SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP | ANGLE_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);//拨弹电机初始化

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
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
    }
    else // 恢复运行
    {
        DJIMotorEnable(friction_l);
        DJIMotorEnable(friction_r);
        DJIMotorEnable(loader);
    }

    // 如果上一次触发单发或3发指令的时间加上不应期仍然大于当前时间(尚未休眠完毕),直接返回即可
    // 单发模式主要提供给能量机关激活使用(以及英雄的射击大部分处于单发)
    if (hibernate_time + dead_time > DWT_GetTimeline_ms())
        return;

    // 若不在休眠状态,根据robotCMD传来的控制模式进行拨盘电机参考值设定和模式切换
    switch (shoot_cmd_recv.load_mode)
    {
    //停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        loader_is_reversing = 0;  // 重置防堵转状态
        loader_block_count = 0;
        loader_stable_count = 0;
        single_shot_triggered = 0;  // 重置触发标志
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
        DJIMotorOuterLoop(loader, ANGLE_LOOP);
        DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE* 36);
        hibernate_time = DWT_GetTimeline_ms();
        dead_time = 150;
        single_shot_triggered = 1;  // 标记已触发

        break;
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                  // 切换到速度环
        DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE * 36); // 增加3发（*36是减速比）

        angle_2006=loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE * 36;
        hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
        dead_time = 300;    //300                                                                    // 完成3发弹丸发射的时间
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);                                                  // 切换到速度环
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);   //设置电机的参考值（设定值）
 
        //拨弹电机电流检测防堵转
        f_Loader_AntiBlock_Handle(loader, loader->measure.real_current);
   
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    case LOAD_1_REVERSE:                                                                     // 激活能量机关/干扰对方用,英雄用.
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
        DJIMotorSetRef(loader, (loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE)); // 控制量增加一发弹丸的角度
        hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
        dead_time = 500;                                                                    // 完成1发弹丸发射的时间
        break;
    // 也有可能需要从switch-case中独立出来
    
    
    //板盘反转，退弹1发（用于卡弹检测）
    case LOAD_stovepipe:
           if(!IS_HIBERNATED)
        {
            count++;
            DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
            DJIMotorSetRef(loader, loader->measure.total_angle - 0.5 *ONE_BULLET_DELTA_ANGLE*36);   // 控制量增加一发弹丸的角度
            speed_2006=loader->measure.total_angle - 0.5 *ONE_BULLET_DELTA_ANGLE*36;
            hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
            dead_time = 1000;
            if(count==1)
            {
            shoot_cmd_recv.load_mode=LOAD_STOP;
            }                                                                    // 完成1发弹丸发射的时间
         } 
        break;

    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        // ...
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }


    // 确定是否开启摩擦轮,后续可能修改为键鼠模式下始终开启摩擦轮(上场时建议一直开启)
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        // 根据收到的弹速设置设定摩擦轮电机参考值,需实测后填入（现在还没有填入）
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        case SMALL_AMU_18:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        case SMALL_AMU_30:
            DJIMotorSetRef(friction_l, 0);
            DJIMotorSetRef(friction_r, 0);
            break;
        default: // 当前为了调试设定的默认值4000,因为还没有加入裁判系统无法读取弹速.
            DJIMotorSetRef(friction_l, 40000);//40000（25）
            DJIMotorSetRef(friction_r, 40000);
            break;
        }
    }
    else // 关闭摩擦轮
    {
        DJIMotorSetRef(friction_l, 0);
        DJIMotorSetRef(friction_r, 0);
    }

    // 开关弹舱盖（现在规则不需要遥控开关弹舱盖了）
    if (shoot_cmd_recv.lid_mode == LID_CLOSE)
    {
        //...
    }
    else if (shoot_cmd_recv.lid_mode == LID_OPEN)
    {
        //...
    }
    

    shoot_speed=-friction_l->measure.speed_aps;


    // 反馈数据,目前暂时没有要设定的反馈数据,后续可能增加应用离线监测以及卡弹反馈
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}



//拨弹电机电流检测防堵转函数
void f_Loader_AntiBlock_Handle(DJIMotorInstance *loader, float current_ref)
{
    // 获取电机实际电流 (real_current单位: 1 = 0.001A)
    int16_t real_current = loader->measure.real_current;
    
    if (!loader_is_reversing)
    {
        // 正常状态：检测电流是否超过阈值
        if (abs(real_current) > LOADER_BLOCK_CURRENT_THRESHOLD)
        {
            loader_block_count++;
            // 连续堵转计数达到阈值，进入反转状态
            if (loader_block_count >= LOADER_BLOCK_COUNT_THRESHOLD)
            {
                loader_is_reversing = 1;
                loader_original_ref = current_ref;
                loader_block_count = 0;
                loader_stable_count = 0;
                //设置板盘反转标志位
                shoot_cmd_recv.load_mode=LOAD_stovepipe;
                angle_2006=1;
            }
        }
        else{
            // 电流正常，清零堵转计数
            loader_block_count = 0;}
    }
    else
    {
        // 反转状态：检测电流是否稳定
        if (abs(real_current) < LOADER_BLOCK_CURRENT_THRESHOLD - 1000)
        {
            loader_stable_count++;
            // 电流稳定计数达到阈值，解除堵转
            if (loader_stable_count >= LOADER_STABLE_COUNT_THRESHOLD)
            {
                loader_is_reversing = 0;
                loader_stable_count = 0;
                angle_2006=2;
            }
        }
        else
        {
            // 电流仍不稳定，清零稳定计数
            loader_stable_count = 0;
        }
    }
}