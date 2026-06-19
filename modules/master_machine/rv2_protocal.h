//
// Created by 26090 on 25-1-13.
//

#ifndef RV2_PROTOCAL_H
#define RV2_PROTOCAL_H
#include "master_process.h"
#include "stdbool.h"



#pragma pack(1)
typedef struct
{
    uint16_t header ;//SP
    uint8_t mode;
    float q[4];
    float yaw;
    float pitch;
    float yaw_vel;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;// 子弹累计发送次数
    uint16_t crc_16;
} rv2_send_protocol_TongJi;

typedef struct  __attribute__ ((packed))
{
    uint16_t header ;//SP
    uint8_t mode; // 0:不控制 (N0_TARGET),1:控制云台但不开火(TRACKING),2:控制云台且开火(READY_TO_FIRE)
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc; 
    uint16_t crc16;
} rv2_recv_protocol_TongJi_s;

#pragma pack()

void RV2_GimbalToSendData(Vision_Send_s *send,uint8_t *tx_buf,uint16_t *tx_buf_len);
void RV2_GimbalToRecvData(Vision_Recv_s *receive, uint8_t *rx_buf, uint16_t rx_buf_len);



#endif //RV2_PROTOCAL_H