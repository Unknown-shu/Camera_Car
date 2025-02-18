/*
 * nav.c
 *
 *  Created on: 2024年10月25日
 *      Author: SJX
 */
#include "zf_common_headfile.h"
#include "mymenu.h"

#define line_number navigation_line_number

#define VAL_SHOW_NUM_BIT          2
#define VAL_SHOW_POINT_BIT        3
#define VAL_SHOW_START_COL        162

uint8 enter_flag = 1;
uint8 Test;

void navigation_page_process(int Event_Code)
{
    line_number_max=10;
    static uint16 i = 0;
    static uint16 Test = 0;
   //显示菜单
    ips200_show_string_color(60, 0,"Value", PenColor);

    enter_flag += Test;
//    printf("%d\r\n",enter_flag);
    ips200_show_float(VAL_SHOW_START_COL-8, 18 * 11,pitch[0] , 3, VAL_SHOW_POINT_BIT);

    if(enter_flag > 0 || If_Switch_Encoder_Change() == 1)
    {
        Line_Num_Flush(&line_number);
        i = 0;
        enter_flag = 0;

        Menu_Exit_Show(line_number);
        // Left Speed PID
        Menu_Display_Line_Float_Parameter(line_number, 2, "Speed_KP", Speed_PID.Kp);
        Menu_Display_Line_Float_Parameter(line_number, 3, "Speed_Ki", Speed_PID.Ki);
        Menu_Display_Line_Float_Parameter(line_number, 4, "Speed_Kd", Speed_PID.Kd);

        // Right Speed PID
        Menu_Display_Line_Float_Parameter(line_number, 5, "Angle_KP", Angle_PID.Kp);
        Menu_Display_Line_Float_Parameter(line_number, 6, "Angle_Ki", Angle_PID.Ki);
        Menu_Display_Line_Float_Parameter(line_number, 7, "Angle_Kd", Angle_PID.Kd);

        // Turn Speed PID
        Menu_Display_Line_Float_Parameter(line_number, 8, "Angle_Acc_KP", Angle_AccPID.Kp);
        Menu_Display_Line_Float_Parameter(line_number, 9, "Angle_Acc_Ki", Angle_AccPID.Ki);
        Menu_Display_Line_Float_Parameter(line_number, 10, "Angle_Acc_Kd", Angle_AccPID.Kd);

        Menu_Display_Line_Float_Parameter(line_number, 11, "Angleroll_KP", Angleroll_PID.Kp);
        Menu_Display_Line_Float_Parameter(line_number, 12, "Angleroll_Ki", Angleroll_PID.Ki);
        Menu_Display_Line_Float_Parameter(line_number, 13, "Angleroll_Kd", Angleroll_PID.Kd);

        Menu_Display_Line_Float_Parameter(line_number, 14, "Speedcirle_KP", Speedcirle_PID.Kp);
        Menu_Display_Line_Float_Parameter(line_number, 15, "Speedcirle_Ki", Speedcirle_PID.Ki);
        Menu_Display_Line_Float_Parameter(line_number, 16, "Speedcirle_Kd", Speedcirle_PID.Kd);
        ips200_show_string_color(0, 18 * 11, "Pitch:", PenColor_else);
//        if (line_number != 10)     ips200_show_string_color(0, 18 * 10, "basic_V0", PenColor);
//        else                       ips200_show_string_color(0, 18 * 10, "basic_V0", PenColor_else);
//        ips200_show_float(VAL_SHOW_START_COL, 18*10, basic_V0, 3, 2);
//
//        // Straight Speed PID
//        if (line_number != 11)    ips200_show_string_color(0, 18 * 11, "Straight_Speed_KP", PenColor);
//        else                      ips200_show_string_color(0, 18 * 11, "Straight_Speed_KP", PenColor_else);
//        ips200_show_float(VAL_SHOW_START_COL, 18 * 11, Straight_Speed_PID.Kp, VAL_SHOW_NUM_BIT, VAL_SHOW_POINT_BIT);
//
//        if (line_number != 12)    ips200_show_string_color(0, 18 * 12, "Straight_Speed_Ki", PenColor);
//        else                      ips200_show_string_color(0, 18 * 12, "Straight_Speed_Ki", PenColor_else);
//        ips200_show_float(VAL_SHOW_START_COL, 18 * 12, Straight_Speed_PID.Ki, VAL_SHOW_NUM_BIT, VAL_SHOW_POINT_BIT);
//
//        if (line_number != 13)    ips200_show_string_color(0, 18 * 13, "Straight_Speed_Kd", PenColor);
//        else                      ips200_show_string_color(0, 18 * 13, "Straight_Speed_Kd", PenColor_else);
//        ips200_show_float(VAL_SHOW_START_COL, 18 * 13, Straight_Speed_PID.Kd, VAL_SHOW_NUM_BIT, VAL_SHOW_POINT_BIT);
    }
    Test = Key_IfEnter();

//    if(key_get_state(KEY_1) == KEY_SHORT_PRESS)
//    {
//        Beep_ShortRing();
//        line_number--;//高亮选择往下
//        pagelimit(&line_number,line_number_max);
//        i++;
//    }
//    if(key_get_state(KEY_3) == KEY_SHORT_PRESS)
//    {
//        Beep_ShortRing();
//        line_number++; //高亮选择往上
//        pagelimit(&line_number,line_number_max);
//        i++;
//    }

    if(key_get_state(KEY_2) == KEY_SHORT_PRESS)
    {
        Beep_ShortRing();
        Set_Menu(&menu, MAIN_PAGE);
        ips200_clear();
        i++;
    }
    if(key_get_state(KEY_4) == KEY_SHORT_PRESS)
    {
        Beep_ShortRing();
        switch(line_number)
        {
            case 1:
                Set_Menu(&menu, MAIN_PAGE);
                ips200_clear();
                break;
            case 2:
                menu_Val_CFG(&Speed_PID.Kp, 18 * line_number, 1);
                break;
            case 3:
                menu_Val_CFG(&Speed_PID.Ki, 18 * line_number, 0.1);
                break;
            case 4:
                menu_Val_CFG(&Speed_PID.Kd, 18 * line_number, 1);
                break;

            // 右轮速度PID参数调整
            case 5:
                menu_Val_CFG(&Angle_PID.Kp, 18 * line_number, 1);
                break;
            case 6:
                menu_Val_CFG(&Angle_PID.Ki, 18 * line_number, 0.1);
                break;
            case 7:
                menu_Val_CFG(&Angle_PID.Kd, 18 * line_number, 1);
                break;

            // 转向PID参数调整
            case 8:
                menu_Val_CFG(&Angle_AccPID.Kp, 18 * line_number, 0.1);
                break;
            case 9:
                menu_Val_CFG(&Angle_AccPID.Ki, 18 * line_number, 0.1);
                break;
            case 10:
                menu_Val_CFG(&Angle_AccPID.Kd, 18 * line_number, 1);
                break;
            case 11:
                menu_Val_CFG(&Angleroll_PID.Kp, 18 * line_number, 0.1);
                break;
            case 12:
                menu_Val_CFG(&Angleroll_PID.Ki, 18 * line_number, 0.1);
                break;
            case 13:
                menu_Val_CFG(&Angleroll_PID.Kd, 18 * line_number, 1);
                break;
            case 14:
                menu_Val_CFG(&Speedcirle_PID.Kp, 18 * line_number, 0.1);
                break;
            case 15:
                menu_Val_CFG(&Speedcirle_PID.Ki, 18 * line_number, 0.1);
                break;
            case 16:
                menu_Val_CFG(&Speedcirle_PID.Kd, 18 * line_number, 1);
                break;
            default:
                break ;
        }
        i++;
        Flash_WriteAllVal();
    }

    key_clear_all_state();
}
