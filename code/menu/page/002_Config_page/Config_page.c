/*
 * Null_page.c
 *
 *  Created on: 2024年9月27日
 *      Author: SJX
 */

#include "MyHeadfile.h"
 /*
 * 空白页
 * 使用时请更改Null_line_number
 *       在page_setting添加null_page_process()
 *                      和extern Null_line_number
 *
 */
#include "zf_common_headfile.h"
#include "mymenu.h"

#define line_number Config_line_number

void Config_page_process(int Event_Code)
{
    line_number_max=3;

   //显示菜单

    ips200_show_string_color(105, 0,"Config", PenColor);

//  1.组合导航
    if(line_number!=1) ips200_show_string_color(0, 18,"flash", PenColor);
    else              ips200_show_string_color(0, 18,"flash", PenColor_else);
//    if(g_flash_enable_flag == 1)
//        ips200_show_string(190,18,"Open ");
//    else
//        ips200_show_string(190,18,"Close");
    menu_Set_CFG_OpenClose_Show(18, g_flash_enable_flag);
////   2.电机控制
    if(line_number!=2) ips200_show_string_color(0, 18*2,"wifi_image_send", PenColor);
     else              ips200_show_string_color(0, 18*2,"wifi_image_send", PenColor_else);
    menu_Set_CFG_OpenClose_Show(18*2, g_wifi_image_open_flag);
//
    if(line_number!=3) ips200_show_string_color(0, 18*3,"Started_Debug", PenColor);
     else              ips200_show_string_color(0, 18*3,"Started_Debug", PenColor_else);
    menu_Set_CFG_OpenClose_Show(18*3, g_started_debug);

    if(line_number!=4) ips200_show_string_color(0, 18*4,"", PenColor);
     else              ips200_show_string_color(0, 18*4,"", PenColor_else);

    //指针向上
//       if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
//       {
//           gpio_set_level(Beep,1);
//           system_delay_ms(10);
//           gpio_set_level(Beep,0);
//           key_clear_state(KEY_1);
//           line_number--;//高亮选择往下
//           pagelimit(&line_number,line_number_max);
//       }
//       //指针向下
//       if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
//       {
//           gpio_set_level(Beep,1);
//           system_delay_ms(10);
//           gpio_set_level(Beep,0);
//           key_clear_state(KEY_3);
//           line_number++; //高亮选择往上
//           pagelimit(&line_number,line_number_max);
//       }
        Line_Num_Flush(&line_number);
       //指针向左
       if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
       {
           gpio_set_level(Beep,1);
           system_delay_ms(10);
           gpio_set_level(Beep,0);
           key_clear_state(KEY_2);
           Set_Menu(&menu, MAIN_PAGE);
//           line_number=1;
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
//                   g_flash_enable_flag = ~ g_flash_enable_flag;
                   menu_Set_CFG_Value_Toggle(&g_flash_enable_flag);
//                   line_number=1;
//                   ips200_clear();

                   break ;
               case 2:
                   menu_Set_CFG_Value_Toggle(&g_wifi_image_open_flag);
                   break;
               case 3:
                   menu_Set_CFG_Value_Toggle(&g_started_debug);
                   Car_Stop();
                   break;
               case 4:

                   break;
               default:
                   break ;
           }
           key_clear_state(KEY_4);
       }

   }

