#ifndef __MyEncoder_H
#define __MyEncoder_H

#include "MyHeadFile.h"
#include "IfxGpt12_IncrEnc.h"

typedef struct
{
        uint8 distance_record_status;
        float left_distance_start_point ;
        float right_distance_start_point ;
        float left_distance;
        float right_distance;
        float AVG_distance;
}Encoder_Distance_Typedef;


extern int16 Encoder_speed_l;
extern int16 Encoder_speed_r;
extern int switch_encoder_num;
extern int switch_encoder_change_num ;
extern uint8 switch_encode_bring_flag;
extern uint8 switch_encode_change_get_buff_flag;


void MyEncoder_Init(void);
//int16 Encoder_MTM(encoder_index_enum gptn,int n,uint8 direct);
void GetSpeed(void);
void Get_Switch_Num(void);
int16 My_Switch_encoder_get_count (encoder_index_enum encoder_n);
uint8 If_Switch_Encoder_Change(void);
void Get_Encoder_Distance(Encoder_Distance_Typedef *Distance_Structure);
void Encoder_Distance_Start(Encoder_Distance_Typedef *Distance_Structure);
void Encoder_Distance_Stop(Encoder_Distance_Typedef *Distance_Structure);
void Distance_StartPoint_Check(Encoder_Distance_Typedef *Distance_Structure);
uint8 Encoder_Distance_MaxLimit(Encoder_Distance_Typedef *Distance_Structure, float distance_max);


#endif
