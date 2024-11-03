/*
 * Sys.c
 *
 *  Created on: 2024��10��27��
 *      Author: sun
 */
#include "Sys.h"
/***********************************************
* @brief : ��С���˷�
* @param : void
* @return: ����ֵ
* @date  :
* @author:
************************************************/
float slope_calculate (uint8 begin, uint8 end,int * border)
{
    float xsum = 0, ysum = 0,xysum = 0, x2sum = 0;
    int16  i = 0;
    float result = 0;
    static float resultlast;
    for (i = begin; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        xysum += i * (border[i]);
        x2sum += i * i;
    }
    if ((end - begin) * x2sum - xsum * xsum) //�жϳ����Ƿ�Ϊ��
    {
    result = ((end - begin) * xysum - xsum * ysum) / ((end - begin) * x2sum - xsum * xsum);
    resultlast = result;
    }
    else
    {
        result = resultlast;
    }
    return result;
}
/***********************************************
* @brief : ����б�ʽؾ�
* @param : void
* @return: ����ֵ
* @date  : start ��ʼ
*          end ��ֹ
*          border �߽�����
*          *slope_new ���б��
*          *distance_new ��Žؾ�
* @author:
************************************************/
void caculate_distance(uint8 start,uint8 end,int *border,float *slope_new,float *distance_new)
{
    uint16 i, num = 0;
    uint16 xsum = 0,ysum = 0;
    float y_average, x_average;
    num = 0;
    xsum = 0;
    ysum = 0;
    y_average = 0;
    x_average = 0;
    for (i = start; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        num++;
    }
    // �������ƽ����
        if(num)
    {
        x_average = (float)(xsum / num);
        y_average = (float)(ysum / num);
        y_average = (float)(ysum / num);
        }
        /*����б��*/
        *slope_new = slope_calculate(start,end,border);//б��
        *distance_new = y_average - (*slope_new)*x_average;//�ؾ�
}


