#include "MyHeadfile.h"

int16 Encoder_speed_l = 0;
int16 Encoder_speed_r = 0;

void MyEncoder_Init(void)
{
    encoder_dir_init(TIM2_ENCODER, ENCODER_L, ENCODER_DIR_L);//���ֱ�����
    encoder_dir_init(TIM6_ENCODER, ENCODER_R, ENCODER_DIR_R);//���ֱ�����

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
void getspeed(void)
{
 // ��ȡ��������ֵ
    Encoder_speed_l = -Encoder_MTM(TIM2_ENCODER,3,1);
    Encoder_speed_r = -Encoder_MTM(TIM6_ENCODER,3,1);
    //JustFloat_Test();
};
