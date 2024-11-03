#ifndef __UART_H
#define __UART_H

#include "zf_driver_uart.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"

#define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_0
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // 默认 UART0_TX_P14_0
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // 默认 UART0_RX_P14_1

extern fifo_struct uart_data_fifo;

void UART_Init(void);
//void MyUART_Write_Buffer(float DATA1, float DATA2, float DATA3, float DATA4 );

#endif
