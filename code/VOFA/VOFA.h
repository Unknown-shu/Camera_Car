#ifndef __VOFA_H
#define __VOFA_H

#define send_uart_n  UART_0             //uart_index_enum

#include "zf_driver_uart.h"
//void printf_buffer(uint8 *send_buff, uint8 len);
void VOFA_Send(float data_a, float data_b, float data_c, float data_d);
#endif
