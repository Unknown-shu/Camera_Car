/*
 * Gyro_page.c
 *
 *  Created on: 2023年3月25日
 *      Author: Dell
 */
#include "zf_common_headfile.h"
#include "mymenu.h"

#define line_number navigation_line_number

void navigation_page_process(int Event_Code)
{
    line_number_max=3;

   //显示菜单
    ips200_show_string_color(60, 0,"Setting", PenColor);
    ips200_show_string_color(0, 18,"Speed = ", PenColor);

    if(line_number!=1)     ips200_show_string_color(0, 18*2,"+Speed", PenColor);
     else                  ips200_show_string_color(0, 18*2,"+Speed", PenColor_else);
    if(line_number!=2)     ips200_show_string_color(0, 18*3,"-Speed", PenColor);
     else                  ips200_show_string_color(0, 18*3,"-Speed", PenColor_else);
//    if(line_number!=3)     ips200_show_string_color(0, 18*3,"start_mode", PenColor);
//     else                  ips200_show_string_color(0, 18*3,"start_mode", PenColor_else);
    if(line_number!=3)     ips200_show_string_color(0, 18*4,"ENGINE_START", PenColor);
     else                  ips200_show_string_color(0, 18*4,"ENGINE_START", PenColor_else);

    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
    {
        gpio_set_level(Beep,1);
        system_delay_ms(10);
        gpio_set_level(Beep,0);
        key_clear_state(KEY_1);
        line_number--;//高亮选择往下
        pagelimit(&line_number,line_number_max);
    }
    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
    {
        gpio_set_level(Beep,1);
        system_delay_ms(10);
        gpio_set_level(Beep,0);
        key_clear_state(KEY_3);
        line_number++; //高亮选择往上
        pagelimit(&line_number,line_number_max);
    }
    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        gpio_set_level(Beep,1);
        system_delay_ms(10);
        gpio_set_level(Beep,0);
        key_clear_state(KEY_2);
        Set_Menu(&menu, MAIN_PAGE);
//        line_number=1;
        ips200_clear();
    }
    if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        gpio_set_level(Beep,1);
        system_delay_ms(10);
        gpio_set_level(Beep,0);
        switch(line_number)
        {
            case 1:

                break ;
            case 2:

                break ;
            default:
                break ;
        }
        key_clear_state(KEY_4);
    }

}


