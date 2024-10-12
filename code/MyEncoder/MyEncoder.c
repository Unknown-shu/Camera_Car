#include "MyHeadfile.h"

int16 Encoder_speed_l = 0;
int16 Encoder_speed_r = 0;

void MyEncoder_Init(void)
{
    encoder_dir_init(TIM2_ENCODER, ENCODER_L, ENCODER_DIR_L);//左轮编码器
    encoder_dir_init(TIM6_ENCODER, ENCODER_R, ENCODER_DIR_R);//右轮编码器

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
        default:
            break;
    }
    encoder_clear_count(gptn);    //编码器清空

    return CoderOut;
}
//-------------------------------------------------------------------------------------------------------------------
//  @brief      获得转速
//  @param      gptn：编码器对应编号
//  @param      n   ：n次均值滤波
//  @return     int16
//  @note       里面的自加，自减根据实际情况调，车往前进是两个编码器值都为正
//-------------------------------------------------------------------------------------------------------------------
void getspeed(void)
{
 // 获取编码器的值
    Encoder_speed_l = -Encoder_MTM(TIM2_ENCODER,3,1);
    Encoder_speed_r = -Encoder_MTM(TIM6_ENCODER,3,1);
    //JustFloat_Test();
};
