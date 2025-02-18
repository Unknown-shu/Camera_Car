/*
 * Flashbluetooth.c
 *
 *  Created on: 2024年10月16日
 *      Author: SJX
 */

/*
 * 密钥0x0d000721 分配0:0
 * 速度环PID参数分配空间0:32-47  其中左32-39，右40-47
 * 转向环PID参数分配空间0:48-63
 * 基准速度参数分配空间0:64-71
 */
#include "FLASH.H"

uint8 g_flash_enable_flag = 0;                         //flash启用标志位，为1代表启用，为0默认启用,extern全局变量

void Flash_Init(void)
{
    if(g_flash_enable_flag == 1)
    {
        uint32 flash_Wrote_status = 0;                     //flash曾写入状态符，密钥为0d000721，若符合则代表曾写入，数据可信
        uint8 flash_check_flag;
        flash_check_flag = flash_check(0,0);

        if(flash_check_flag == 1)
        {
            flash_buffer_clear();
            flash_read_page_to_buffer(0,0);
            flash_Wrote_status = flash_union_buffer[0].uint32_type;
            if(flash_Wrote_status == FLASH_KEY)                //曾写入状态符与密钥吻合，将flash数据读出
            {
                //速度环
                Motor_Speed_PID_Left.Kp = flash_union_buffer[32].float_type;
                Motor_Speed_PID_Left.Ki = flash_union_buffer[33].float_type;
                Motor_Speed_PID_Left.Kd = flash_union_buffer[34].float_type;

                Motor_Speed_PID_Right.Kp = flash_union_buffer[40].float_type;
                Motor_Speed_PID_Right.Ki = flash_union_buffer[41].float_type;
                Motor_Speed_PID_Right.Kd = flash_union_buffer[42].float_type;

                //转向环
                Turn_Speed_PID.Kp = flash_union_buffer[48].float_type;
                Turn_Speed_PID.Ki = flash_union_buffer[49].float_type;
                Turn_Speed_PID.Kd = flash_union_buffer[50].float_type;

                Straight_Speed_PID.Kp = flash_union_buffer[51].float_type;
                Straight_Speed_PID.Ki = flash_union_buffer[52].float_type;
                Straight_Speed_PID.Kd = flash_union_buffer[53].float_type;

                basic_V0 = flash_union_buffer[64].float_type;
            }
            else
            {

            }
        }
    }
}

void Flash_WriteAllVal(void)
{
    flash_buffer_clear();
    //速度环
    flash_union_buffer[32].float_type = Motor_Speed_PID_Left.Kp;
    flash_union_buffer[33].float_type = Motor_Speed_PID_Left.Ki;
    flash_union_buffer[34].float_type = Motor_Speed_PID_Left.Kd;

    flash_union_buffer[40].float_type = Motor_Speed_PID_Right.Kp;
    flash_union_buffer[41].float_type = Motor_Speed_PID_Right.Ki;
    flash_union_buffer[42].float_type = Motor_Speed_PID_Right.Kd;

    //转向环
    flash_union_buffer[48].float_type = Turn_Speed_PID.Kp;
    flash_union_buffer[49].float_type = Turn_Speed_PID.Ki;
    flash_union_buffer[50].float_type = Turn_Speed_PID.Kd;

    flash_union_buffer[51].float_type = Straight_Speed_PID.Kp;
    flash_union_buffer[52].float_type = Straight_Speed_PID.Ki;
    flash_union_buffer[53].float_type = Straight_Speed_PID.Kd;


    flash_union_buffer[64].float_type = basic_V0;

    flash_union_buffer[0].uint32_type = FLASH_KEY;

    flash_write_page_from_buffer(0, 0);
    flash_buffer_clear();
}
