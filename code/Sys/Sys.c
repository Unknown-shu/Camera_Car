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


// 曲率计算函数
/***********************************************
* @brief : 曲率计算函数
* @param : array 需要计算的数组
*          start 数组起始点 推荐从3开始谨防出现不必要的bug
*          size 需要计算的个数
* @return: double 计算出的曲率值
* @date  :2024年11月4日10:06:55
* @author:SJX
* @exp   :Calculate_Curvature(left, i, 10);
************************************************/
float Calculate_Curvature(int array[], int start, int size)
{
    float total_curvature = 0.0;
    int i = 0;
    for (i = start; i < start + size; i++)
    {
        // 当前点和前后点的坐标
        float x1 = i - 1, y1 = array[i - 1];
        float x2 = i, y2 = array[i];
        float x3 = i + 1, y3 = array[i + 1];

        // 分子部分
        float numerator = (x3 - x2) * (y1 - y2) - (y3 - y2) * (x1 - x2);

        // 分母部分
        float denominator = pow(pow(x3 - x2, 2) + pow(y3 - y2, 2), 1.5);

        // 避免除以零
        if (denominator != 0)
        {
            total_curvature += fabs(numerator / denominator);
        }
    }

    // 返回平均曲率
    return total_curvature / (size);
}


double distance(double a[2],double b[2])//求两点间距离，数组a[2]为点a的坐标信息，a[0]为a的x坐标，a[1]为a的y坐标
{
     double dis;//两点间距离
     double x,y,x2,y2;
     x=a[0]-b[0];
     y=a[1]-b[1];
     x2=x*x;
     y2=y*y;
     dis=sqrt((x2+y2));//double sqrt(double x)为求平方根函数
     return dis;
}
int collinear(double a[2],double b[2],double c[2])//判断三点是否共线，共线返回1
{
      double k1,k2;
      double kx1,ky1,kx2,ky2;
      if(a[0]==b[0]&&b[0]==c[0])  return 1;//三点横坐标都相等，共线
      else
        {
          kx1=b[0]-a[0];
          kx2=b[0]-c[0];
          ky1=b[1]-a[1];
          ky2=b[1]-a[1];
          k1=ky1/kx1;
          k2=ky2/kx2;
          if(k1==k2) return 1;//AB与BC斜率相等，共线
           else  return 0;//不共线
         }
}
double curvature(double a[2],double b[2],double c[2])//double为数据类型，
{                                                    //数组a[2]为点a的坐标信息，a[0]为a的x坐标，a[1]为a的y坐标
       double cur;//求得的曲率
       if(collinear(a,b,c)==1)//判断三点是否共线
       {
        cur=0.0;//三点共线时将曲率设为某个值，0
        }
       else
      {
       double radius;//曲率半径
       double dis,dis1,dis2,dis3;//距离
       double cosA;//ab确定的边所对应的角A的cos值
       dis1=distance(a,b);
       dis2=distance(a,c);
       dis3=distance(b,c);
       dis=dis2*dis2+dis3*dis3-dis1*dis1;
       cosA=dis/(2*dis2*dis3);//余弦定理
       radius=0.5*dis1/cosA;
       cur=1/radius;
      }
       return cur;
}
