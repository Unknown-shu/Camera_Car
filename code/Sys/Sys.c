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


// ���ʼ��㺯��
/***********************************************
* @brief : ���ʼ��㺯��
* @param : array ��Ҫ���������
*          start ������ʼ�� �Ƽ���3��ʼ�������ֲ���Ҫ��bug
*          size ��Ҫ����ĸ���
* @return: double �����������ֵ
* @date  :2024��11��4��10:06:55
* @author:SJX
* @exp   :Calculate_Curvature(left, i, 10);
************************************************/
float Calculate_Curvature(int array[], int start, int size)
{
    float total_curvature = 0.0;
    int i = 0;
    for (i = start; i < start + size; i++)
    {
        // ��ǰ���ǰ��������
        float x1 = i - 1, y1 = array[i - 1];
        float x2 = i, y2 = array[i];
        float x3 = i + 1, y3 = array[i + 1];

        // ���Ӳ���
        float numerator = (x3 - x2) * (y1 - y2) - (y3 - y2) * (x1 - x2);

        // ��ĸ����
        float denominator = pow(pow(x3 - x2, 2) + pow(y3 - y2, 2), 1.5);

        // ���������
        if (denominator != 0)
        {
            total_curvature += fabs(numerator / denominator);
        }
    }

    // ����ƽ������
    return total_curvature / (size);
}


double distance(double a[2],double b[2])//���������룬����a[2]Ϊ��a��������Ϣ��a[0]Ϊa��x���꣬a[1]Ϊa��y����
{
     double dis;//��������
     double x,y,x2,y2;
     x=a[0]-b[0];
     y=a[1]-b[1];
     x2=x*x;
     y2=y*y;
     dis=sqrt((x2+y2));//double sqrt(double x)Ϊ��ƽ��������
     return dis;
}
int collinear(double a[2],double b[2],double c[2])//�ж������Ƿ��ߣ����߷���1
{
      double k1,k2;
      double kx1,ky1,kx2,ky2;
      if(a[0]==b[0]&&b[0]==c[0])  return 1;//��������궼��ȣ�����
      else
        {
          kx1=b[0]-a[0];
          kx2=b[0]-c[0];
          ky1=b[1]-a[1];
          ky2=b[1]-a[1];
          k1=ky1/kx1;
          k2=ky2/kx2;
          if(k1==k2) return 1;//AB��BCб����ȣ�����
           else  return 0;//������
         }
}
double curvature(double a[2],double b[2],double c[2])//doubleΪ�������ͣ�
{                                                    //����a[2]Ϊ��a��������Ϣ��a[0]Ϊa��x���꣬a[1]Ϊa��y����
       double cur;//��õ�����
       if(collinear(a,b,c)==1)//�ж������Ƿ���
       {
        cur=0.0;//���㹲��ʱ��������Ϊĳ��ֵ��0
        }
       else
      {
       double radius;//���ʰ뾶
       double dis,dis1,dis2,dis3;//����
       double cosA;//abȷ���ı�����Ӧ�Ľ�A��cosֵ
       dis1=distance(a,b);
       dis2=distance(a,c);
       dis3=distance(b,c);
       dis=dis2*dis2+dis3*dis3-dis1*dis1;
       cosA=dis/(2*dis2*dis3);//���Ҷ���
       radius=0.5*dis1/cosA;
       cur=1/radius;
      }
       return cur;
}
