/*
 * Servo.h
 *
 *  Created on: 2025Äê1ÔÂ12ÈÕ
 *      Author: 15955
 */

#ifndef CODE_SERVO_H_
#define CODE_SERVO_H_
#include "zf_common_headfile.h"

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#define SERVO_MOTOR_FREQ 300

#define Servo_LeftFront  ATOM3_CH1_P33_5
#define Servo_LeftRear   ATOM3_CH3_P33_7
#define Servo_RightFront ATOM3_CH2_P33_6
#define Servo_RightRear  ATOM3_CH0_P33_4

void Servo_Init(void);
void Set_Servo(pwm_channel_enum pwm_channel, uint16 set_angle);

#endif /* CODE_SERVO_H_ */
