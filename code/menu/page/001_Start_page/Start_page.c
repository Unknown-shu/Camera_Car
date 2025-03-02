/*
 * Gyro_page.c
 *
 *  Created on: 2023年3月25日
 *      Author: Dell
 */
#include "zf_common_headfile.h"
#include "mymenu.h"

void Start_page_process(int Event_Code)
{
    line_number_max=3;


   //显示菜单
    ips200_show_string_color(105, 0,"START", PenColor);
    Menu_Exit_Show(line_number);

    Menu_Display_Line_String(line_number, 2, "Start");
    Menu_Display_Line_String(line_number, 3, "Wifi_Image_Init");
//    Menu_Display_Line_String(line_number, 4, "Value");
//    Menu_Display_Line_String(line_number, 5, "Camera");
//    if(line_number!=2)     ips200_show_string_color(0, 18,"Start", PenColor);
//     else                  ips200_show_string_color(0, 18,"Start", PenColor_else);
//    if(line_number!=3)     ips200_show_string_color(0, 18*2,"Wifi_Image_Init", PenColor);
//     else                  ips200_show_string_color(0, 18*2,"Wifi_Image_Init", PenColor_else);
//    if(line_number!=4)     ips200_show_string_color(0, 18*3,"", PenColor);
//     else                  ips200_show_string_color(0, 18*3,"", PenColor_else);
//    if(line_number!=5)     ips200_show_string_color(0, 18*4,"", PenColor);
//     else                  ips200_show_string_color(0, 18*4,"", PenColor_else);
//    ips200_show_int_color(100, 18*3,FLAG.startmode,1, PenColor);

    //指针向上
//    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
//    {
//        gpio_set_level(Beep,1);
//        system_delay_ms(10);
//        gpio_set_level(Beep,0);
//        key_clear_state(KEY_1);
//        line_number--;//高亮选择往下
//        pagelimit(&line_number,line_number_max);
//    }
//    //指针向下
//    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
//    {
//        gpio_set_level(Beep,1);
//        system_delay_ms(10);
//        gpio_set_level(Beep,0);
//        key_clear_state(KEY_3);
//        line_number++; //高亮选择往上
//        pagelimit(&line_number,line_number_max);
//    }
    Line_Num_Flush(&line_number);
    //指针向左
    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        gpio_set_level(Beep,1);
        system_delay_ms(10);
        gpio_set_level(Beep,0);
        key_clear_state(KEY_2);
        Set_Menu(&menu, MAIN_PAGE);
        line_number=1;
        ips200_clear();
    }
    //指针按下
    if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        gpio_set_level(Beep,1);
        system_delay_ms(10);
        gpio_set_level(Beep,0);
        switch(line_number)
        {
            case 1:
                Set_Menu(&menu, MAIN_PAGE);
                break;
            case 2:

                Car_Start();
                break ;
            case 3:
                Wifi_Image_Init();
                g_wifi_image_open_flag = 1;
                break;
            case 4:

                break;
            default:
                break ;
        }
        key_clear_state(KEY_4);
    }

}


