#ifndef __BEEP_H__
#define __BEEP_H__

#define    SHORT_RING_TIME          3

void Beep_Start(void);
void Beep_Init(void);
void Beep_Stop(void);
void Beep_ShortRing(void);
void Beep_MediumRing(void);
void Beep_LongRing(void);
void Beep_Ring(uint16_t Time);
void Beep_Timer_ShortRing(void);

#endif
