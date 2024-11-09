#include "MYENCODER.h"

int16 Encoder_speed_l = 0;
int16 Encoder_speed_r = 0;

int switch_encoder_num = 0;
int switch_encoder_change_num = 0;
uint8 switch_encode_bring_flag;
uint8 switch_encode_change_get_buff_flag = 0;                   //�仯���壬�����仯δ���Ͼͽ��仯ֵ����
void MyEncoder_Init(void)
{
    encoder_dir_init(TIM2_ENCODER, ENCODER_L, ENCODER_DIR_L);//���ֱ�����
    encoder_dir_init(TIM6_ENCODER, ENCODER_R, ENCODER_DIR_R);//���ֱ�����
    encoder_quad_init(TIM3_ENCODER, Switch_ENCODER_L, Switch_ENCODER_R);
//    encoder_dir_init(TIM3_ENCODER, Switch_ENCODER_L, Switch_ENCODER_R);

}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      �������ɼ�
//  @param      gptn    ����������Ӧ���
//  @param      n       ��n�ξ�ֵ�˲�
//  @param      direct  ��1����0 ���� ��������������
//  @return     int16
//  @note       Ҫ�Ǳ����������˾Ͱ�direct��һ�� 1����0
//-------------------------------------------------------------------------------------------------------------------
int16 Encoder_MTM(encoder_index_enum gptn,int n,uint8 direct)
{
    int16 Coder = 0;
    int16 CoderOut = 0;
    switch(gptn)
    {
        case TIM2_ENCODER:
            for(int i = 0;i < n;i++)
            {
                if(direct)
                {
                    Coder -=  encoder_get_count(TIM2_ENCODER);
                }
                else
                {
                    Coder +=  encoder_get_count(TIM2_ENCODER);
                }

            }
            CoderOut = Coder/n;
            break;
        case TIM6_ENCODER:
            for(int i = 0;i < n;i++)
            {
                if(direct)
                {
                    Coder +=  encoder_get_count(TIM6_ENCODER);
                }
                else
                {
                    Coder -=  encoder_get_count(TIM6_ENCODER);
                }

            }
            CoderOut = Coder/n;
            break;
        case TIM3_ENCODER:
            for(int i = 0;i < n;i++)
            {
                if(direct)
                {
                    Coder +=  encoder_get_count(TIM3_ENCODER);
                }
                else
                {
                    Coder -=  encoder_get_count(TIM3_ENCODER);
                }

            }
            CoderOut = Coder/n;
            break;
        default:
            break;
    }
    encoder_clear_count(gptn);    //���������

    return CoderOut;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      ���ת��
//  @param      gptn����������Ӧ���
//  @param      n   ��n�ξ�ֵ�˲�
//  @return     int16
//  @note       ������Լӣ��Լ�����ʵ�������������ǰ��������������ֵ��Ϊ��
//-------------------------------------------------------------------------------------------------------------------
void GetSpeed(void)
{
 // ��ȡ��������ֵ
    Encoder_speed_l = -Encoder_MTM(TIM2_ENCODER,3,1);
    Encoder_speed_r = -Encoder_MTM(TIM6_ENCODER,3,1);
//    Encoder_speed_r = -Encoder_MTM(TIM3_ENCODER,1,1);
    //JustFloat_Test();
};

/***********************************************
* @brief : ��ȡ��ת������ֵ
* @param : void
* @return: void
* @date  : 2024��11��6��12:23:25
* @author: SJX
************************************************/
void Get_Switch_Num(void)
{
    int tmp = 0;
    static int encoder_cnt, timer_cnt, last_switch_encoder_num = 0;
    timer_cnt = -My_Switch_encoder_get_count(TIM3_ENCODER);
    encoder_clear_count(TIM3_ENCODER);

    if(abs(timer_cnt) < 4)
    {
        encoder_cnt += timer_cnt;
    }
    else
    {
        tmp = timer_cnt / 4;
        switch_encoder_num += tmp;
        tmp = timer_cnt % 4;
        encoder_cnt += tmp;
    }
    if(abs(encoder_cnt) >= 4)
    {
        tmp = encoder_cnt / 4;
        switch_encoder_num += tmp;
        tmp = encoder_cnt % 4;
        encoder_cnt = 0;
        encoder_cnt += tmp;
    }
//    printf("%d, %d, %d, %d\r\n", switch_encoder_change_num, switch_encode_change_get_buff_flag,
//            last_switch_encoder_num, switch_encoder_num);
    if((last_switch_encoder_num != switch_encoder_num ) && switch_encode_change_get_buff_flag == 0)
    {
        switch_encode_change_get_buff_flag = 1;
        switch_encoder_change_num = switch_encoder_num - last_switch_encoder_num;
        Beep_Timer_ShortRing();

    }
    else if((last_switch_encoder_num != switch_encoder_num )&& switch_encode_change_get_buff_flag == 1)
    {
        switch_encoder_change_num = switch_encoder_change_num + switch_encoder_num - last_switch_encoder_num;
        Beep_Timer_ShortRing();
    }
    if((last_switch_encoder_num == switch_encoder_num ) && switch_encode_change_get_buff_flag == 0)
    {
        switch_encoder_change_num = 0;
        Beep_Stop();
    }
    last_switch_encoder_num = switch_encoder_num;


}
/***********************************************
* @brief : ��ת��������ȡ��������������ת������
* @param : void
* @return: void
* @date  : 2024��11��6��12:26:53
* @author: SJX
************************************************/
int16 My_Switch_encoder_get_count (encoder_index_enum encoder_n)
{
    int16 encoder_data = 0;
    switch(encoder_n)
    {
        case TIM2_ENCODER: encoder_data = (int16)IfxGpt12_T2_getTimerValue(&MODULE_GPT120); break;
        case TIM3_ENCODER: encoder_data = (int16)IfxGpt12_T3_getTimerValue(&MODULE_GPT120); break;
        case TIM4_ENCODER: encoder_data = (int16)IfxGpt12_T4_getTimerValue(&MODULE_GPT120); break;
        case TIM5_ENCODER: encoder_data = (int16)IfxGpt12_T5_getTimerValue(&MODULE_GPT120); break;
        case TIM6_ENCODER: encoder_data = (int16)IfxGpt12_T6_getTimerValue(&MODULE_GPT120); break;
        default: encoder_data = 0;
    }
    return encoder_data;
}
/***********************************************
* @brief : �ж���ת�������Ƿ���ֱ仯
* @param : void
* @return: uint8            1�仯 0����
* @date  : 2024��11��6��12:27:38
* @author: SJX
************************************************/
uint8 If_Switch_Encoder_Change(void)
{
    switch_encode_change_get_buff_flag = 0;
    if(switch_encoder_change_num != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
