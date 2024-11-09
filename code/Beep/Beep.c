#include "MYHEADFILE.h"

void Beep_Init(void)
{
    gpio_init(Beep, GPO, 0, GPO_PUSH_PULL);
    system_delay_init();
    pit_ms_init(CCU61_CH1, SHORT_RING_TIME);            //�������ж�
    pit_close(CCU61_CH1);
}


void Beep_Start(void)
{
    gpio_set_level(Beep, 1);
}

void Beep_Stop(void)
{
    gpio_set_level(Beep, 0);
}

void Beep_ShortRing(void)
{
    Beep_Start();
    system_delay_ms(SHORT_RING_TIME);
//    system_delay_us(200);
    Beep_Stop();
}

void Beep_MediumRing(void)
{
    Beep_Start();
    system_delay_ms(100);
    Beep_Stop();
}

void Beep_LongRing(void)
{
    Beep_Start();
    system_delay_ms(1000);
    Beep_Stop();
}

void Beep_Ring(uint16_t Time)
{
    Beep_Start();
    system_delay_ms(Time);
    Beep_Stop();
}
/***********************************************
* @brief : ��ʱ�����ط���������
* @param : void
* @return: void
* @date  : 2024��11��6��12:24:04
* @author: SJX
************************************************/
void Beep_Timer_ShortRing(void)
{
    Beep_Start();
//    pit_start(CCU61_CH1);
//    pit_enable(CCU61_CH1);

}



