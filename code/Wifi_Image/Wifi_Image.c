/*
 * Wifi_Image.c
 *
 *  Created on: 2024��10��26��
 *      Author: sun
 */
#include "WIFI_IMAGE.H"

uint8 g_wifi_image_open_flag = 1;
uint8 image_copy[MT9V03X_H][MT9V03X_W];

/***********************************************
* @brief : wifiͼ����ʼ��
* @param : ��
* @return: ��
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
        system_delay_ms(1000);                                                   // ��ʼ��ʧ�� �ȴ� 100ms
        break;
    }
    if(i == 0)
    {
        ips200_show_string(150, 18*2, "suss");
        printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // ģ�� IP ��ַ
    }
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    Beep_ShortRing();

}
/***********************************************
* @brief : wifiͼ����������ͷͼ��
* @param : ��
* @return: ��
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


