/*
 * Test.h
 *
 *  Created on: 2024��10��24��
 *      Author: SJX
 */
#ifndef    __SUATUS_H
#define    __SUATUS_H

#include "MyHeadFile.h"

typedef enum
{
    status_car_stop,
    status_car_start,
}Car_Status;

extern uint8 g_Car_Status;
extern uint8 g_started_debug;

void Car_Stop(void);
void Car_Start(void);
Car_Status Get_Car_Status(void);
void Car_Starus_Flush(void);


#endif