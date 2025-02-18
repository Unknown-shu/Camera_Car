/*
 * Test.h
 *
 *  Created on: 2024Äê10ÔÂ24ÈÕ
 *      Author: SJX
 */
#ifndef    __SUATUS_H
#define    __SUATUS_H

#include "MyHeadFile.h"

typedef enum
{
    status_car_stop,
    status_car_start,
    status_car_gyroscope_run,
}Car_Status;

typedef enum
{
    close_status,
    open_status,
}Status_Flag;

extern uint8 g_Car_Status;
extern uint8 g_started_debug;

void Car_Stop(void);
void Car_Start(void);
Car_Status Get_Car_Status(void);
void Car_Starus_Flush(void);
void Track_Out_Protect(void);
void Gyroscope_Run(void);
void Key_RUN(void);


#endif
