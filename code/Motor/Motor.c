#include "MOTOR.H"
#include "zf_driver_pwm.h"
#include "zf_driver_gpio.h"

//Limit����
#define LIMIT_VAL(a,min,max) ((a)<(min)?(min):((a)>(max)?(max):(a)))

void Motor_Init(void)
{
    pwm_init(MOTOR_LEFT_1,12500,0);                   //��ʼ������
    gpio_init(MOTOR_LEFT_2, GPO, 0, GPO_PUSH_PULL);
    pwm_init(MOTOR_RIGHT_1,12500,0);                  //��ʼ���ҵ��
    gpio_init(MOTOR_RIGHT_2, GPO, 0, GPO_PUSH_PULL);
}

void MotorSetPWM(int pwm_left,int pwm_right)
{
    //����������pwm�����޷�

    LIMIT_VAL(pwm_left, MOTOR_PWM_MAX, MOTOR_PWM_MIN);
    LIMIT_VAL(pwm_right, MOTOR_PWM_MAX, MOTOR_PWM_MIN);
    //���Ƶ������ת��ת��
    //����
    gpio_set_level(MOTOR_LEFT_2, 1);
    pwm_set_duty(MOTOR_LEFT_1,5000+(int)(pwm_left/2));
    //�ҵ��
    gpio_set_level(MOTOR_RIGHT_2, 1);
    pwm_set_duty(MOTOR_RIGHT_1,5000-(int)(pwm_right/2));
}


