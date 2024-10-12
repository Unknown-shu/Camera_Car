#ifndef __PIN_H__
#define __PIN_H__

#define Beep P33_10                                     //蜂鸣器引脚

#define ENCODER_L         TIM2_ENCODER_CH1_P33_7        //左编码器计数引脚
#define ENCODER_DIR_L     TIM2_ENCODER_CH2_P33_6        //左编码器方向引脚
#define ENCODER_R         TIM6_ENCODER_CH1_P20_3        //右编码器计数引脚
#define ENCODER_DIR_R     TIM6_ENCODER_CH2_P20_0        //右编码器方向引脚

#define  MOTOR_LEFT_1   ATOM0_CH5_P02_5
#define  MOTOR_LEFT_2   P02_7//ATOM0_CH7_P02_7
#define  MOTOR_RIGHT_1  ATOM0_CH4_P02_4
#define  MOTOR_RIGHT_2  P02_6//ATOM0_CH6_P02_6


#endif
