#include "VOFA.h"

uint8 tail[4] = {0x00, 0x00, 0x80, 0x7f} ;


void VOFA_Send(float data_a, float data_b, float data_c, float data_d)
{
    float send_data[4];
    send_data[0] = data_a;
    send_data[1] = data_b;
    send_data[2] = data_c;
    send_data[3] = data_d;
//    uart_write_buffer(send_uart_n, send_data, 4);
//    uart_write_buffer(send_uart_n, tail, 4);
    printf("%f,%f,%f,%f\r\n", send_data[0], send_data[1], send_data[2], send_data[3]);
//    printf("%d,%d,%d,%d\r\n", tail[0], tail[1], tail[2], tail[3]);
}

//void printf_buffer(float *send_buff, uint8 len)
//{
//    int i = 0;
//    for(i=0; i<len; i++)
//    {
//        printf("%f ",send_buff[i]);
//    }
//}
