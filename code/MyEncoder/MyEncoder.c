#include "MYENCODER.h"

int16 Encoder_speed_l = 0;
int16 Encoder_speed_r = 0;

int switch_encoder_num = 0;
int switch_encoder_change_num = 0;
uint8 switch_encode_bring_flag;
uint8 switch_encode_change_get_buff_flag = 0;                   //�仯���壬�����仯δ���Ͼͽ��仯ֵ����

uint8 encoder_distance_open_flag = open_status;                           //�Ƿ�������̼�
int left_encoder_distance_cnt = 0;
int right_encoder_distance_cnt = 0;
float left_encoder_distance = 0;
float right_encoder_distance = 0;

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
    if(encoder_distance_open_flag == 1)
    {
        left_encoder_distance_cnt += Encoder_speed_l;
        right_encoder_distance_cnt += Encoder_speed_r;
        left_encoder_distance = left_encoder_distance_cnt * 0.00008797830909 * 100;
        right_encoder_distance = right_encoder_distance_cnt * 0.00008797830909 * 100;
    }
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
//        Beep_Stop();
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

/***********************************************
* @brief : ��ȡ������·��
* @param : void
* @return: void
* @date  : 2024��11��9��13:26
* @author: SJX
* @note  : ��λcm
************************************************/
void Get_Encoder_Distance(Encoder_Distance_Typedef *Distance_Structure)
{
    if(Distance_Structure->distance_record_status == open_status)
    {
        Distance_StartPoint_Check(Distance_Structure);
        Distance_Structure->left_distance = left_encoder_distance - Distance_Structure->left_distance_start_point;
        Distance_Structure->right_distance = right_encoder_distance - Distance_Structure->right_distance_start_point;
        Distance_Structure->AVG_distance = (Distance_Structure->left_distance + Distance_Structure->right_distance) / 2;
    }
    else
    {
        printf("Distance_Status is close ! pls sure code");
    }
}

/***********************************************
* @brief : ������������̼�¼
* @param : �ṹ���ַ
* @return: void
* @date  : 2024��11��9��13:31
* @author: SJX
************************************************/
void Encoder_Distance_Start(Encoder_Distance_Typedef *Distance_Structure)
{
    if(Distance_Structure->distance_record_status == close_status)
    {
        Distance_Structure->distance_record_status = open_status;
        Distance_Structure->left_distance_start_point = left_encoder_distance;
        Distance_Structure->right_distance_start_point = right_encoder_distance;
    }
}

/***********************************************
* @brief : ������������̼�¼
* @param : �ṹ���ַ
* @return: void
* @date  : 2024��11��9��13:33
* @author: SJX
************************************************/
void Encoder_Distance_Stop(Encoder_Distance_Typedef *Distance_Structure)
{
//    if(Distance_Structure.distance_record_status == open_status)
//   {
        Distance_Structure->distance_record_status = close_status;
        Distance_Structure->left_distance_start_point = 0;
        Distance_Structure->right_distance_start_point = 0;
//   }
}

/***********************************************
* @brief : ����޷��ж�
* @param : *left_distance   ��·�̵�ַ
*          *right_distance  ��·�̵�ַ
*          distance_max     ·�����ֵ,��λcm
* @return: uint8            С��maxֵʱ����0,����1ʱ����0
* @date  : 2024��11��10��14:49:35
* @author: SJX
* @exp   : Get_Encoder_Distance(&left_distance, &right_distance);
*          i = Encoder_Distance_MaxLimit(&left_distance, &right_distance, 20);
*          if(i == 1)   return 1;
************************************************/
uint8 Encoder_Distance_MaxLimit(Encoder_Distance_Typedef *Distance_Structure, float distance_max)
{
    if(Distance_Structure->distance_record_status == open_status)
    {
        Encoder_Distance_Start(Distance_Structure);
        Get_Encoder_Distance(Distance_Structure);
        if(Distance_Structure->AVG_distance < distance_max)
        {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    return 0;
}

//uint8 Encoder_Distance_Left_MaxLimit(Encoder_Distance_Typedef *Distance_Structure, float left_distance_max, float right_distance_max)
//{
//    if(Distance_Structure->distance_record_status == open_status)
//    {
//        Encoder_Distance_Start(Distance_Structure);
//        Get_Encoder_Distance(Distance_Structure);
//        if(Distance_Structure->AVG_distance < left_distance_max)
//        {
//            return 0;
//        }
//        else
//        {
//            return 1;
//        }
//    }
//    return 0;
//}



/***********************************************
* @brief : ����飬ȷ��������
* @param : �ṹ���ַ
* @return: uint8            С��maxֵʱ����0,����1ʱ����0
* @date  : 2024��11��11��10:14:32
* @author: SJX
* @exp   :
************************************************/
void Distance_StartPoint_Check(Encoder_Distance_Typedef *Distance_Structure)
{
    float i, j;
    i = fabs(Distance_Structure->left_distance_start_point - 0);
    j = fabs(Distance_Structure->right_distance_start_point - 0);
    if(i > 10E-7 && j > 10E-7)
    {
        return;
    }
    else
    {
//        printf("distance_start_point empty\r\n");
    }
}
