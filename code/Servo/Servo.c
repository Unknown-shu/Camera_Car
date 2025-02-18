/*
 * Servo.c
 *
 *  Created on: 2025��1��12��
 *      Author: 15955
 */
// ------------------ ���ռ�ձȼ��㷽ʽ ------------------
//
// �����Ӧ�� 0-180 ��Ƕȶ�Ӧ ��������� 0.5ms-2.5ms �ߵ�ƽ
//
// ��ô��ͬƵ���µ�ռ�ձȼ��㷽ʽ����
// PWM_DUTY_MAX/(1000/freq)*(1+Angle/180) �� 50hz ʱ���� PWM_DUTY_MAX/(1000/50)*(1+Angle/180)
//
// ��ô 100hz �� 90�ȵĴ�� ���ߵ�ƽʱ��1.5ms ��������Ϊ
// PWM_DUTY_MAX/(1000/100)*(1+90/180) = PWM_DUTY_MAX/10*1.5
//
// ------------------ ���ռ�ձȼ��㷽ʽ ------------------
#include "zf_common_headfile.h"
/***********************************************
* @brief :�����ʼ��
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
* @brief :�������
* @param :void
* @return:
* @date  :2024.1.14
* @author:yuu.
************************************************/
void Set_Servo(pwm_channel_enum pwm_channel, uint16 set_angle)
{
    pwm_set_duty(pwm_channel, (uint32)SERVO_MOTOR_DUTY(set_angle));
}
