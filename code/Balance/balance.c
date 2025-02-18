#include "balance.h"
//int16 TARGET_SPEED = -350;
uint8 Balance_Flag = 0;
uint8 Action_Flag = 0;
void Balance_Angle_acc(void)
{
	
	int16 Speed = 0;
	
	
	float Gyroscope_Ya = 0;		//陀螺仪Y的加速度
	
	icm20602_get_gyro();
	Gyroscope_Ya = icm20602_gyro_transition(icm20602_gyro_y);

	Speed = -(int16)Positional_PID(&Angle_AccPID,Angle_AccPID.target_val, Gyroscope_Ya, 10);

	Seekfree_FOC_Duty_Set(Speed, Speed);
//	printf("%d\r\n", Speed);

}

uint8_t Timer_Count = 0;
void Balance_Angle(void)
{
	if(Timer_Count++ >= 2)
	{
		get_gyorscope_data();
		Timer_Count = 0;
		Angle_AccPID.target_val = Positional_PID(&Angle_PID,Mechanical_Zero + Angle_PID.target_val, pitch[0], 25);
	}
}

uint8_t Timer_Count1 = 0;
void Balance_Speed(void)
{
	if(Timer_Count1++ >= 20)
	{
//	    printf("%d, %d\r\n", target_left + target_right, Encoder_speed_l + Encoder_speed_r);
        Angle_PID.target_val = Positional_PID(&Speed_PID,target_left + target_right ,-Encoder_speed_l - Encoder_speed_r, 0);
		Timer_Count1 = 0;
	}
}
