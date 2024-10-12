#ifndef __MyEncoder_H
#define __MyEncoder_H

void MyEncoder_Init(void);
int16 Encoder_MTM(encoder_index_enum gptn,int n,uint8 direct);
void getspeed(void);

extern int16 Encoder_speed_l;
extern int16 Encoder_speed_r;


#endif
