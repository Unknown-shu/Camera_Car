#include "zf_common_headfile.h"
#include "MYENCODER.h"

float speed_L;//左轮编码器速度
float speed_R;//右轮编码器速度

int switch_encoder_num = 0;
int switch_encoder_change_num = 0;
uint8 switch_encode_bring_flag;
uint8 switch_encode_change_get_buff_flag = 0;                   //变化缓冲，谨防变化未用上就将变化值清零

uint8 encoder_distance_open_flag = 0;
int left_encoder_distance_cnt = 0;
int right_encoder_distance_cnt = 0;
float left_encoder_distance = 0;
float right_encoder_distance = 0;

void MyEncoder_Init(void)
{
    encoder_dir_init(TIM2_ENCODER, ENCODER_L, ENCODER_DIR_L);//左轮编码器
    encoder_dir_init(TIM6_ENCODER, ENCODER_R, ENCODER_DIR_R);//右轮编码器
//    encoder_quad_init(TIM3_ENCODER, Switch_ENCODER_L, Switch_ENCODER_R);
//    encoder_dir_init(TIM3_ENCODER, Switch_ENCODER_L, Switch_ENCODER_R);

}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      编码器采集
//  @param      gptn    ：编码器对应编号
//  @param      n       ：n次均值滤波
//  @param      direct  ：1或者0 决定 编码器的正负号
//  @return     int16
//  @note       要是编码器方向反了就把direct改一下 1或者0
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
    encoder_clear_count(gptn);    //编码器清空

    return CoderOut;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      获得编码器读值
//  @param      gptn：编码器对应编号
//  @param      n   ：n次均值滤波
//  @return     int16
//  @note       里面的自加，自减根据实际情况调，车往前进是两个编码器值都为正
//-------------------------------------------------------------------------------------------------------------------
void Get_Encoder_Cnt(void)
{
 // 获取编码器的值
    speed_L = -Encoder_MTM(TIM2_ENCODER,3,1);
    speed_R = -Encoder_MTM(TIM6_ENCODER,3,1);
    if(encoder_distance_open_flag == 1)
    {
        left_encoder_distance_cnt += speed_L;
        right_encoder_distance_cnt += speed_R;
    }
};

/***********************************************
* @brief : 获取旋转编码器值
* @param : void
* @return: void
* @date  : 2024年11月6日12:23:25
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
* @brief : 旋转编码器获取函数，仅用于旋转编码器
* @param : void
* @return: void
* @date  : 2024年11月6日12:26:53
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
* @brief : 判断旋转编码器是否出现变化
* @param : void
* @return: uint8            1变化 0不变
* @date  : 2024年11月6日12:27:38
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
* @brief :获取编码器路程
* @param : void
* @return: 左路程和右路程的地址
* @date  : 2024年11月9日13:26
* @author: SJX
************************************************/
void Get_Encoder_Distance(float *left_distance, float *right_distance)
{
    left_encoder_distance = left_encoder_distance_cnt * 0.0000879783;
    right_encoder_distance = right_encoder_distance_cnt * 0.0000879783;
    *left_distance = left_encoder_distance;
    *right_distance = right_encoder_distance;
}

/***********************************************
* @brief : 开启编码器里程记录
* @param : void
* @return: void
* @date  : 2024年11月9日13:31
* @author: SJX
************************************************/
void Encoder_Distance_Start(void)
{
    encoder_distance_open_flag = 1;
    left_encoder_distance = 0;
}

/***********************************************
* @brief : 结束编码器里程记录
* @param : void
* @return: void
* @date  : 2024年11月9日13:33
* @author: SJX
************************************************/
void Encoder_Distance_Stop(void)
{
    encoder_distance_open_flag = 0;
}
