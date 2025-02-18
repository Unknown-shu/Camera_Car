#ifndef __PIN_H__
#define __PIN_H__

#include "MOTOR.h"

#define Beep P33_10                             //蜂鸣器引脚

#define ENCODER_L         TIM2_ENCODER_CH1_P33_7        //左编码器计数引脚
#define ENCODER_DIR_L     TIM2_ENCODER_CH2_P33_6        //左编码器方向引脚
#define ENCODER_R         TIM6_ENCODER_CH1_P20_3        //右编码器计数引脚
#define ENCODER_DIR_R     TIM6_ENCODER_CH2_P20_0        //右编码器方向引脚

#define Switch_ENCODER_L         TIM3_ENCODER_CH1_P02_6        //右编码器计数引脚
#define Switch_ENCODER_R         TIM3_ENCODER_CH2_P02_7        //右编码器方向引脚

#if MOTOR_MODE == 0
#define  MOTOR_LEFT_1   ATOM0_CH1_P21_3
#define  MOTOR_LEFT_2   P21_2//ATOM0_CH7_P02_7
#define  MOTOR_RIGHT_1  ATOM0_CH3_P21_5
#define  MOTOR_RIGHT_2  P21_4//ATOM0_CH6_P02_6
#endif

#if MOTOR_MODE == 1
#define  MOTOR_LEFT_1  ATOM0_CH0_P21_2
#define  MOTOR_LEFT_2 ATOM0_CH1_P21_3
#define  MOTOR_RIGHT_2 ATOM0_CH2_P21_4
#define  MOTOR_RIGHT_1 ATOM0_CH3_P21_5
#endif

#endif
