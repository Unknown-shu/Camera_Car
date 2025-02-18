/*
 * body_contral.c
 *
 *  Created on: 2025年1月14日
 *      Author: 15955
 */

#include "zf_common_headfile.h"
#include "math.h"

IKparam IKParam;
float alpha1,alpha2,beta1,beta2;
float servoLeftFront,servoLeftRear,servoRightFront,servoRightRear;
float alphaLeftToAngle,betaLeftToAngle,alphaRightToAngle,betaRightToAngle;

float Y_demand=60;//足端期望y坐标
float Kp_Y=0.1;//足端运动p控制
float X,Y;
float targetSpeed=10;
float stab_roll = 0;
uint8_t lowest = 70;
uint8_t highest = 130;
/***********************************************
* @brief :姿态控制
* @param :void
* @return:X:横坐标 L:左
*         y:纵坐标 R:右
* @date  :2024.1.14
* @author:yuu.
************************************************/
void body_contral()
{
    float speedAvg = (motor_speed_l + motor_speed_r)/2;
    X = Positional_PID(&Speedcirle_PID,targetSpeed, speedAvg, 10);// 根据速度调整X坐标PID
    Y = Y + Kp_Y * (Y_demand - Y);

    icm20602_get_gyro();
    float Gyroscope_Roll = icm20602_gyro_transition(icm20602_gyro_x);
    uint16_t Remoter_Input = Y;
    //float E_H = (L/2) * sin(Phi*(PI/180));
    stab_roll = Positional_PID(&Angleroll_PID,0, Gyroscope_Roll, 10); // 复杂地形适应控制PID
    float L_Height = Remoter_Input + stab_roll;
    float R_Height = Remoter_Input - stab_roll;


    IKParam.XRight = X;
    IKParam.YRight = R_Height;
    IKParam.XLeft = X;
    IKParam.YLeft = L_Height;

    inverse_Kinematics();
    Set_Servo(Servo_LeftFront,servoLeftFront);
    Set_Servo(Servo_LeftRear,servoLeftRear);
    Set_Servo(Servo_RightFront,servoRightFront);
    Set_Servo(Servo_RightRear,servoRightRear);
}
/***********************************************
* @brief :逆运动学解算
* @param :void
* @return:
* @date  :2024.1.14
* @author:yuu.
************************************************/
void inverse_Kinematics(void)
{
    float aLeft = 2 * IKParam.XLeft * L1;
    float bLeft = 2 * IKParam.YLeft * L1;
    float cLeft = IKParam.XLeft * IKParam.XLeft + IKParam.YLeft * IKParam.YLeft + L1 * L1 - L2 * L2;
    float dLeft = 2 * L4 * (IKParam.XLeft - L5);
    float eLeft = 2 * L4 * IKParam.YLeft;
    float fLeft = ((IKParam.XLeft - L5) * (IKParam.XLeft - L5) + L4 * L4 + IKParam.YLeft * IKParam.YLeft - L3 * L3);
    //printf("%f,%f,%f,%f,%f,%f\n",aLeft,bLeft,cLeft,dLeft,eLeft,fLeft);
    alpha1 = 2 * atan((bLeft + sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));
    alpha2 = 2 * atan((bLeft - sqrt((aLeft * aLeft) + (bLeft * bLeft) - (cLeft * cLeft))) / (aLeft + cLeft));

    beta1 = 2 * atan((eLeft + sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));
    beta2 = 2 * atan((eLeft - sqrt((dLeft * dLeft) + eLeft * eLeft - (fLeft * fLeft))) / (dLeft + fLeft));

    alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

    if(alpha1 >= PI/4) IKParam.alphaLeft = alpha1;
    else IKParam.alphaLeft = alpha2;
    if(beta1 >= 0 && beta1 <= PI/4) IKParam.betaLeft = beta1;
    else IKParam.betaLeft = beta2;

    float aRight = 2 * IKParam.XRight * L1;
    float bRight = 2 * IKParam.YRight * L1;
    float cRight = IKParam.XRight * IKParam.XRight + IKParam.YRight * IKParam.YRight + L1 * L1 - L2 * L2;
    float dRight = 2 * L4 * (IKParam.XRight - L5);
    float eRight = 2 * L4 * IKParam.YRight;
    float fRight = ((IKParam.XRight - L5) * (IKParam.XRight - L5) + L4 * L4 + IKParam.YRight * IKParam.YRight - L3 * L3);

    IKParam.alphaRight = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    IKParam.betaRight = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

    alpha1 = 2 * atan((bRight + sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    alpha2 = 2 * atan((bRight - sqrt((aRight * aRight) + (bRight * bRight) - (cRight * cRight))) / (aRight + cRight));
    beta1 = 2 * atan((eRight + sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));
    beta2 = 2 * atan((eRight - sqrt((dRight * dRight) + eRight * eRight - (fRight * fRight))) / (dRight + fRight));

    alpha1 = (alpha1 >= 0)?alpha1:(alpha1 + 2 * PI);
    alpha2 = (alpha2 >= 0)?alpha2:(alpha2 + 2 * PI);

    if(alpha1 >= PI/4) IKParam.alphaRight = alpha1;
    else IKParam.alphaRight = alpha2;
    if(beta1 >= 0 && beta1 <= PI/4) IKParam.betaRight = beta1;
    else IKParam.betaRight = beta2;

    alphaLeftToAngle = ((IKParam.alphaLeft / 6.28) * 360);//弧度转角度
    betaLeftToAngle = ((IKParam.betaLeft / 6.28) * 360);

    alphaRightToAngle = ((IKParam.alphaRight / 6.28) * 360);
    betaRightToAngle = ((IKParam.betaRight / 6.28) * 360);

    servoLeftFront = 90 + betaLeftToAngle;
    servoLeftRear =  alphaLeftToAngle-90;
    servoRightFront = 90 - betaRightToAngle;
    servoRightRear = 270 - alphaRightToAngle;
}
/***********************************************
* @brief :正运动学解算（已知xy坐标求舵机角度）
* @param :float
* @return:
* @date  :2024.2.15
* @author:yuu.
************************************************/
void ForwardKinematics(float angle_alpha, float angle_beta)
{
    //pointA_x,pointA_y,pointB_x,pointB_y,pointC_x,pointC_y,pointD_x,pointD_y;
    IKParam.pointD_x = L5;
    IKParam.pointD_y = 0.0;

    IKParam.pointA_x = -L1*sinf(angle_alpha);
    IKParam.pointA_y = L1*cosf(angle_alpha);

    IKParam.pointC_x = L5+L4*cosf(angle_beta);
    IKParam.pointC_y = L4*sinf(angle_beta);

    float A  = 2 * L2 * (IKParam.pointC_x - IKParam.pointA_x);
    float B  = 2 * L2 * (IKParam.pointC_y - IKParam.pointA_y);
    float C  = L2 * L2 - L3 * L3 + (IKParam.pointC_x - IKParam.pointA_x) * (IKParam.pointC_x - IKParam.pointA_x) + (IKParam.pointC_y - IKParam.pointA_y) * (IKParam.pointC_y - IKParam.pointA_y);
    float angle2 = 2 * atan2f((B + sqrtf(A * A + B * B - C * C)), (A + C));

    IKParam.pointB_x = IKParam.pointA_x-L2*cosf(angle2);
    IKParam.pointB_y = IKParam.pointA_y+L2*sinf(angle2);
}
/***********************************************
* @brief :直线运动
* @param :void
* @return:
* @date  :2024.2.15
* @author:yuu.
************************************************/
