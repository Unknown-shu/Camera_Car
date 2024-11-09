#ifndef __MyEncoder_H
#define __MyEncoder_H

#include "zf_common_headfile.h"
#include "IfxGpt12_IncrEnc.h"


extern float speed_L;//×óÂÖ±àÂëÆ÷ËÙ¶È
extern float speed_R;//ÓÒÂÖ±àÂëÆ÷ËÙ¶È
extern int16 Encoder_speed_l;
extern int16 Encoder_speed_r;
extern int switch_encoder_num;
extern int switch_encoder_change_num ;
extern uint8 switch_encode_bring_flag;
extern uint8 switch_encode_change_get_buff_flag;


void MyEncoder_Init(void);
//int16 Encoder_MTM(encoder_index_enum gptn,int n,uint8 direct);
void Get_Encoder_Cnt(void);
void Get_Switch_Num(void);
int16 My_Switch_encoder_get_count (encoder_index_enum encoder_n);
uint8 If_Switch_Encoder_Change(void);
void Encoder_Distance_Stop(void);
void Encoder_Distance_Start(void);
void Get_Encoder_Distance(float *left_distance, float *right_distance);



#endif
