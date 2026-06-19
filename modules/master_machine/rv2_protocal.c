/*
 * @Description: 
 * @Author: changfeng
 * @brief: 
 * @version: 
 * @Date: 2025-02-14 13:28:09
 * @LastEditors:  
 * @LastEditTime: 2025-02-16 16:53:29
 */
//
// Created by 26090 on 25-1-13.
//

#include "rv2_protocal.h"

#include <bsp_log.h>
#include <crc16.h>
#include <string.h>

static rv2_recv_protocol_TongJi_s rv2_recv_data_TongJi={0};


void RV2_GimbalToSendData(Vision_Send_s *send,uint8_t *tx_buf,uint16_t *tx_buf_len)
{
    static rv2_send_protocol_TongJi rv2_send_data={
        .header = 0x5053,  // SP
        .mode = 1,
    };
    // // 姿态部分
    rv2_send_data.pitch=send->pitch;
    rv2_send_data.yaw=send->yaw;

    rv2_send_data.pitch_vel=send->pitch_vel;
    rv2_send_data.yaw_vel=send->yaw_vel;

    rv2_send_data.bullet_speed = send->bullet_speed;
    // rv2_send_data.bullet_speed = send->bullet_speed;
    // rv2_send_data.bullet_count = send->bullet_count;

    rv2_send_data.q[0]=send->q[0];
    rv2_send_data.q[1]=send->q[1];
    rv2_send_data.q[2]=send->q[2];
    rv2_send_data.q[3]=send->q[3];

    //CRC校验
    rv2_send_data.crc_16=crc_16((uint8_t *)&rv2_send_data,sizeof(rv2_send_data)-2);

    memcpy(tx_buf,&rv2_send_data,sizeof(rv2_send_data));
 
    *tx_buf_len=sizeof(rv2_send_data);
}


void RV2_GimbalToRecvData(Vision_Recv_s *receive, uint8_t *rx_buf, uint16_t rx_buf_len)
{
    //包头校验
    if(rx_buf[0]==0x53 && rx_buf[1]==0x50)
    {
        //CRC校验
        uint16_t checksum=crc_16(rx_buf,rx_buf_len-2);
       if(rx_buf[rx_buf_len-1]==((checksum&0xFF00)>>8)&&(rx_buf[rx_buf_len-2]==(checksum&0x00FF)))
        // if(1) //暂时不校验crc
        {
            //(&rv2_recv_data,rx_buf,sizeof(rv2_recv_data));
           memcpy(&rv2_recv_data_TongJi,rx_buf,sizeof(rv2_recv_data_TongJi));
           receive->yaw=rv2_recv_data_TongJi.yaw;
           receive->pitch=rv2_recv_data_TongJi.pitch;
           receive->yaw_vel=rv2_recv_data_TongJi.yaw_vel;
           receive->pitch_vel=rv2_recv_data_TongJi.pitch_vel;
           receive->mode = rv2_recv_data_TongJi.mode;

        }
        else
        {
            LOGERROR("RV2 Receive+ checksum error");
            return;
        }
    }
    else
    {
        receive->mode = 0;
        LOGERROR("RV2 Receive Header error");
        return;
    }
}