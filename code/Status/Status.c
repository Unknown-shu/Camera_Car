/***********************************************
* @brief : ����        ��������ֹͣ״̬
* @date  : �޸�����     2024.10.24
* @author: ����        SJX
************************************************/

#include "Status.h"

uint8 g_Car_Status = status_car_stop ;
uint8 g_started_debug = 0;
void Car_Stop(void)
{
    g_Car_Status = status_car_stop;                      //�رյ�������ò˵�
//  pit_disable(CCU61_CH0);
    target_left = 0;
    target_right = 0;
    Motor_Stop();
}

void Car_Start(void)
{
    system_delay_ms(1000);
    g_Car_Status = status_car_start;
    PID_clear(&Motor_Speed_PID_Left);
    PID_clear(&Motor_Speed_PID_Right);
    PID_clear(&Turn_Speed_PID);
    PID_clear(&Straight_Speed_PID);
    pit_enable(CCU61_CH0);
}

Car_Status Get_Car_Status(void)
{
    return g_Car_Status;
}

void Car_Starus_Flush(void)
{
    if(g_Car_Status ==status_car_stop)
    {
        Car_Stop();
    }
    else
    {
        Car_Start();
    }
}


