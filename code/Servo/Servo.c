/*
 * Servo.c
 *
 *  Created on: 2025年1月12日
 *      Author: 15955
 */
// ------------------ 舵机占空比计算方式 ------------------
//
// 舵机对应的 0-180 活动角度对应 控制脉冲的 0.5ms-2.5ms 高电平
//
// 那么不同频率下的占空比计算方式就是
// PWM_DUTY_MAX/(1000/freq)*(1+Angle/180) 在 50hz 时就是 PWM_DUTY_MAX/(1000/50)*(1+Angle/180)
//
// 那么 100hz 下 90度的打角 即高电平时间1.5ms 计算套用为
// PWM_DUTY_MAX/(1000/100)*(1+90/180) = PWM_DUTY_MAX/10*1.5
//
// ------------------ 舵机占空比计算方式 ------------------
#include "zf_common_headfile.h"
/***********************************************
* @brief :舵机初始化
* @param :void
* @return:
* @date  :2024.1.14
* @author:yuu.
************************************************/
void Servo_Init(void)
{
    pwm_init(Servo_LeftFront,SERVO_MOTOR_FREQ,(uint32)SERVO_MOTOR_DUTY(90));
    pwm_init(Servo_LeftRear,SERVO_MOTOR_FREQ,(uint32)SERVO_MOTOR_DUTY(90));
    pwm_init(Servo_RightFront,SERVO_MOTOR_FREQ,(uint32)SERVO_MOTOR_DUTY(90));
    pwm_init(Servo_RightRear,SERVO_MOTOR_FREQ,(uint32)SERVO_MOTOR_DUTY(90));
}
/***********************************************
* @brief :驱动舵机
* @param :void
* @return:
* @date  :2024.1.14
* @author:yuu.
************************************************/
void Set_Servo(pwm_channel_enum pwm_channel, uint16 set_angle)
{
    pwm_set_duty(pwm_channel, (uint32)SERVO_MOTOR_DUTY(set_angle));
}
