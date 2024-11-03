#ifndef __MyEncoder_H
#define __MyEncoder_H

#include "MyHeadFile.h"

void MyEncoder_Init(void);
//int16 Encoder_MTM(encoder_index_enum gptn,int n,uint8 direct);
void GetSpeed(void);
void Get_Switch_Num(void);

extern int16 Encoder_speed_l;
extern int16 Encoder_speed_r;
extern int16 switch_encoder_num;


#endif
