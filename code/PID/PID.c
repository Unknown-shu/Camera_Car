/*
 * pid.c
 *
 * Created on: 2024年1月24日
 *     Author: 潘申奇
 */
#include"PID.h"
#define my_abs(num) (((num) > 0) ? (num) : -(num))

PID_IncTypeDef Motor_Speed_PI;


/*
函数名称：PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd)
函数功能：初始化PID参数
函数变量：*sptr：pid参数结构体
例子：
*/
void PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd)
{
    sptr->Ek1 = 0; // 上次偏差值初始化
    sptr->Ek2 = 0; // 上上次偏差值初始化
    sptr->Kp = kp; // 比例常数
    sptr->Ki = ki; // 积分常数
    sptr->Kd = kd; // 微分常数
    sptr->OUT = 0;
}

/*
函数名称：Positional_PID(PID_IncTypeDef *PID, float SetValue, float ActualValue, float Max_I)
函数功能：位置式pid
函数变量：*sptr：pid参数结构体
          SetValue：设定值
          ActualValue：实际值
          Max_I：最大误差积分
例子：
*/
float Positional_PID(PID_IncTypeDef *PID, float SetValue, float ActualValue, float Max_I)
{
    float PIDInc;
    PID->Ek = SetValue - ActualValue;
    PID->Ek2 += PID->Ek;

    if (PID->Ek2 > Max_I)
        PID->Ek2 = Max_I;
    if (PID->Ek2 < -Max_I)
        PID->Ek2 = -Max_I;

    PIDInc = (PID->Kp * PID->Ek) +
             (PID->Ki * PID->Ek2) +
             (PID->Kd * (PID->Ek - PID->Ek1));
    PID->Ek1 = PID->Ek;
    return PIDInc;
}

/*
函数名称：PID_clear(PID_IncTypeDef *sptr)
函数功能：清空误差
函数变量：*sptr：pid参数结构体
例子：
*/
void PID_clear(PID_IncTypeDef *sptr)
{
    sptr->Ek1 = 0; // 上次偏差值初始化
    sptr->Ek2 = 0; // 上上次偏差值初始化
    //    sptr->OUT = 0;
}

void PID_param_init(void)
{

}
/*
函数名称：set_pid_target(PID_IncTypeDef *pid, float temp_val)
函数功能：设置目标值
函数变量：*sptr：pid参数结构体
函数返回： 无
例子：
*/
void set_pid_target(PID_IncTypeDef *pid, float temp_val)
{
    pid->target_val = temp_val; // 设置当前的目标值
}

/*
函数名称：get_pid_target(PID_IncTypeDef *pid)
函数功能：设置pid参数
函数变量：*sptr：pid参数结构体
函数返回： pid->target_val 当前目标值
例子：
*/
float get_pid_target(PID_IncTypeDef *pid)
{
    return pid->target_val; // 获取当前的目标值
}

/*
函数名称：set_p_i_d(PID_IncTypeDef *pid, float p, float i, float d)
函数功能：设置pid参数
函数变量：*sptr：pid参数结构体
例子：
*/
void set_p_i_d(PID_IncTypeDef *pid, float p, float i, float d)
{
    pid->Kp = p; // 设置比例系数 P
    pid->Ki = i; // 设置积分系数 I
    pid->Kd = d; // 设置微分系数 D
}


