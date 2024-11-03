/*
 * Wifi_Image.c
 *
 *  Created on: 2024年10月26日
 *      Author: sun
 */
#include "WIFI_IMAGE.H"

uint8 g_wifi_image_open_flag = 1;
uint8 image_copy[MT9V03X_H][MT9V03X_W];

/***********************************************
* @brief : wifi图传初始化
* @param : 无
* @return: 无
* @date  : 2024.10.27
* @author: SJX
************************************************/
void Wifi_Image_Init(void)
{
    int i = 0;
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST) )
    {
        i++;
        g_wifi_image_open_flag = 0;
        if(Key_IfEnter())
            break;
        printf("\r\n connect wifi failed. \r\n");
        ips200_show_string(150, 18*2, "failed");
        system_delay_ms(1000);                                                   // 初始化失败 等待 100ms
        break;
    }
    if(i == 0)
    {
        ips200_show_string(150, 18*2, "suss");
        printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // 模块 IP 地址
    }
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    Beep_ShortRing();

}
/***********************************************
* @brief : wifi图传发送摄像头图像
* @param : 无
* @return: 无
* @date  : 2024.10.27
* @author: SJX
************************************************/
void Wifi_Image_Send_Camera(void)
{
//    if(g_wifi_image_open_flag == 1 && Camera_Wifi_Image_Send_Flag == 1 )
    if(g_wifi_image_open_flag == 1)
    {
        Camera_Wifi_Image_Send_Flag = 0;
        memcpy(image_copy[0], image[0], MT9V03X_IMAGE_SIZE);
        seekfree_assistant_camera_send();
    }
}


