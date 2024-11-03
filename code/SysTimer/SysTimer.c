#include "SysTimer.h"

uint32 timer_start_time, timer_stop_time, g_past_time;

void SysTimer_Start(void)
{
    timer_start_time = system_getval_us();
}

void SysTimer_Stop(void)
{
    timer_stop_time = system_getval_us();
    g_past_time = timer_stop_time - timer_start_time - 7;//start stop����ʱ��Ϊ7ns����ȥ
}

uint32 GetPastTime(void)
{
    g_past_time = timer_stop_time - timer_start_time ;
    return g_past_time;
}
