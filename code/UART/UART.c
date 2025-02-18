#include "UART.h"

uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区

fifo_struct uart_data_fifo;

void UART_Init(void)
{
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);
//    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化串口
//    uart_init(UART_2, UART_BAUDRATE, UART2_TX_P10_5, UART2_RX_P10_6);
    uart_init(UART_0, UART_BAUDRATE, UART0_TX_P14_0, UART0_RX_P14_1);
//    uart_init(UART_1, 460800, UART1_TX_P20_10, UART1_RX_P11_10);
//    uart_rx_interrupt(UART_1, 1);
}

//void MyUART_Write_Buffer(float DATA1, float DATA2, float DATA3, float DATA4 )
//{
//    uint8 buff[4] ;
//    buff[0] = DATA1 ;
//    buff[1] = DATA2 ;
//    buff[2] = DATA3 ;
//    buff[3] = DATA4 ;
//    uart_write_buffer(UART_INDEX, buff, 4);
//    uart_write_string(UART_INDEX, "\r\n");
//}
