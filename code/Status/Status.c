/***********************************************
* @brief : 描述        车子启动停止状态
* @date  : 修改日期     2024.10.24
* @author: 作者        SJX
************************************************/

#include "Status.h"

uint8 g_Car_Status = status_car_stop ;
uint8 g_started_debug = 0;

/***********************************************
* @brief : 停车
* @param : void
* @return: void
* @date  : 2024_10
* @author: SJX
************************************************/
void Car_Stop(void)
{
    g_Car_Status = status_car_stop;                      //关闭电机，启用菜单
//  pit_disable(CCU61_CH0);
    target_left = 0;
    target_right = 0;
    Motor_Stop();
}

/***********************************************
* @brief : 启动
* @param : void
* @return: void
* @date  : 2024_10
* @author: SJX
************************************************/
void Car_Start(void)
{
    system_delay_ms(1000);
    Beep_ShortRing();
    g_Car_Status = status_car_start;
    PID_clear(&Motor_Speed_PID_Left);
    PID_clear(&Motor_Speed_PID_Right);
    PID_clear(&Turn_Speed_PID);
    PID_clear(&Straight_Speed_PID);
    PID_clear(&Angle_AccPID);
    PID_clear(&Angle_PID);
    PID_clear(&Speed_PID);
    Circle_Static_Flag = 0;
    cross_road_status = 0;
    pit_enable(CCU61_CH0);
}

/***********************************************
* @brief : 获取车辆状态位
* @param : void
* @return: Car_Status
* @date  : 2024_10
* @author: SJX
************************************************/
Car_Status Get_Car_Status(void)
{
    return g_Car_Status;
}

/***********************************************
* @brief : 根据全局状态位刷新车辆状态
* @param : void
* @return: void
* @date  : 2024_10
* @author: SJX
************************************************/
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

/***********************************************
* @brief : 脱离保护
* @param : void
* @return: void
* @date  : 2024_10
* @author: SJX
************************************************/
void Track_Out_Protect(void)
{
    static uint8 i = 0;
    if(i++ > 100)
    {

        if(camera_process_cnt <= 8)
        {
            Car_Stop();
        }
        i = 0;
        camera_process_cnt = 0;
    }
}

/***********************************************
* @brief : 小陀螺模式
* @param : void
* @return: void
* @date  : 2024年11月12日09:52:12
* @author: SJX
************************************************/
void Gyroscope_Run(void)
{
    g_Car_Status = status_car_gyroscope_run;
    Encoder_Distance_Typedef Gyroscope_Run_Distance_Structure;
    Encoder_Distance_Start(&Gyroscope_Run_Distance_Structure);
//    pit_close(CCU61_CH0);
    target_left = 90;
    target_right = -90;
//    Car_Start();
//    MotorSetPWM(2000, -2000);
//    do{
//
//    }while
//        {
//
//    };
    do
       {
            target_left = 90;
            target_right = -90;
            Get_Encoder_Distance(&Gyroscope_Run_Distance_Structure);
//            printf("%f, %f\r\n",Gyroscope_Run_Distance_Structure.left_distance, Gyroscope_Run_Distance_Structure.right_distance);
//            printf("%d, %d\r\n",target_left, target_right);
       }while( Gyroscope_Run_Distance_Structure.left_distance < 221 ||  Gyroscope_Run_Distance_Structure.right_distance > -221);
//    pit_start(CCU61_CH0);
    Car_Stop();
}
/***********************************************
* @brief : 按键发车停车
* @param : void
* @return: void
* @date  : 2024年12月7日19:38:02
* @author: SJX
************************************************/
void Key_RUN(void)
{
    if(key_get_state(KEY_6) == KEY_SHORT_PRESS)
    {
        Beep_ShortRing();
        if(g_Car_Status == status_car_stop)
        {
            Car_Start();
        }
        else
        {
            Car_Stop();
        }
        key_clear_state(KEY_6);

    }
}


