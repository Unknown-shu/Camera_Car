#ifndef __SYS_TIMER_
#define __SYS_TIMER_

#include "zf_driver_timer.h"

extern uint32 g_past_time;

void SysTimer_Start(void);
void SysTimer_Stop(void);
uint32 GetPastTime(void);

#endif
