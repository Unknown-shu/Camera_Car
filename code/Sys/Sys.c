/*
 * Sys.c
 *
 *  Created on: 2024年10月27日
 *      Author: sun
 */
#include "Sys.h"
/***********************************************
* @brief : 最小二乘法
* @param : void
* @return: 返回值
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
    if ((end - begin) * x2sum - xsum * xsum) //判断除数是否为零
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
* @brief : 计算斜率截距
* @param : void
* @return: 返回值
* @date  : start 起始
*          end 终止
*          border 边界数组
*          *slope_new 存放斜率
*          *distance_new 存放截距
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
    // 计算各个平均数
        if(num)
    {
        x_average = (float)(xsum / num);
        y_average = (float)(ysum / num);
        y_average = (float)(ysum / num);
        }
        /*计算斜率*/
        *slope_new = slope_calculate(start,end,border);//斜率
        *distance_new = y_average - (*slope_new)*x_average;//截距
}


