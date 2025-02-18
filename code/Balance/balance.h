#ifndef __BALANCE_H
#define __BALANCE_H

#include "MyHeadfile.h"

#define Mechanical_Zero -3.f
extern int16 TARGET_SPEED;

extern uint8 Balance_Flag;
extern uint8 Action_Flag;
void Balance_Angle_acc(void);
void Balance_Angle(void);
void Balance_Speed(void);
#endif
