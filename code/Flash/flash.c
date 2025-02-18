/*
 * Flashbluetooth.c
 *
 *  Created on: 2024��10��16��
 *      Author: SJX
 */

/*
 * ��Կ0x0d000721 ����0:0
 * �ٶȻ�PID��������ռ�0:32-47  ������32-39����40-47
 * ת��PID��������ռ�0:48-63
 * ��׼�ٶȲ�������ռ�0:64-71
 */
#include "FLASH.H"

uint8 g_flash_enable_flag = 0;                         //flash���ñ�־λ��Ϊ1�������ã�Ϊ0Ĭ������,externȫ�ֱ���

void Flash_Init(void)
{
    if(g_flash_enable_flag == 1)
    {
        uint32 flash_Wrote_status = 0;                     //flash��д��״̬������ԿΪ0d000721���������������д�룬���ݿ���
        uint8 flash_check_flag;
        flash_check_flag = flash_check(0,0);

        if(flash_check_flag == 1)
        {
            flash_buffer_clear();
            flash_read_page_to_buffer(0,0);
            flash_Wrote_status = flash_union_buffer[0].uint32_type;
            if(flash_Wrote_status == FLASH_KEY)                //��д��״̬������Կ�Ǻϣ���flash���ݶ���
            {
                //�ٶȻ�
                Motor_Speed_PID_Left.Kp = flash_union_buffer[32].float_type;
                Motor_Speed_PID_Left.Ki = flash_union_buffer[33].float_type;
                Motor_Speed_PID_Left.Kd = flash_union_buffer[34].float_type;

                Motor_Speed_PID_Right.Kp = flash_union_buffer[40].float_type;
                Motor_Speed_PID_Right.Ki = flash_union_buffer[41].float_type;
                Motor_Speed_PID_Right.Kd = flash_union_buffer[42].float_type;

                //ת��
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
    //�ٶȻ�
    flash_union_buffer[32].float_type = Motor_Speed_PID_Left.Kp;
    flash_union_buffer[33].float_type = Motor_Speed_PID_Left.Ki;
    flash_union_buffer[34].float_type = Motor_Speed_PID_Left.Kd;

    flash_union_buffer[40].float_type = Motor_Speed_PID_Right.Kp;
    flash_union_buffer[41].float_type = Motor_Speed_PID_Right.Ki;
    flash_union_buffer[42].float_type = Motor_Speed_PID_Right.Kd;

    //ת��
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
