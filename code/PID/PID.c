/*
 * pid.c
 *
 * Created on: 2024年1月24日
 *     Author: 潘申奇
 */
#include"PID.h"
#define MY_ABS(num) (((num) > 0) ? (num) : -(num))
#define LIMIT_VAL(a,min,max) ((a)<(min)?(min):((a)>(max)?(max):(a)))

PID_IncTypeDef Motor_Speed_PID_Left;
PID_IncTypeDef Motor_Speed_PID_Right;
PID_IncTypeDef Turn_Speed_PID;
PID_IncTypeDef Straight_Speed_PID;
PID_IncTypeDef Speed_PID;
PID_IncTypeDef Angle_PID;
PID_IncTypeDef Angle_AccPID;

PID_IncTypeDef Speedcirle_PID;
PID_IncTypeDef Angleroll_PID;

void PID_param_init(void)
{
    PID_Inc_Init(&Motor_Speed_PID_Left, motor_speed_Left_Kp, motor_speed_Left_Ki, 0);
    PID_Inc_Init(&Motor_Speed_PID_Right, motor_speed_Right_Kp, motor_speed_Right_Ki, 0);
    PID_Inc_Init(&Turn_Speed_PID,turn_speed_Kp, turn_speed_Ki, turn_speed_Kd);
    PID_Inc_Init(&Straight_Speed_PID,straight_speed_Kp, straight_speed_Ki, straight_speed_Kd);
    PID_Inc_Init(&Speed_PID, 0, 0, 0 );
    PID_Inc_Init(&Angle_PID, 30, 0, 0 );
    PID_Inc_Init(&Angle_AccPID, 30, 1, 0 );

    PID_Inc_Init(&Speedcirle_PID, 10, 0, 0 );
    PID_Inc_Init(&Angleroll_PID, 10, 0, 0 );

}

/*
函数名称：PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd)
函数功能：初始化PID参数
函数变量：*sptr：pid参数结构体
例子：
*/
void PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd)
{
    sptr->last_Ek = 0; // 上次偏差值初始化
    sptr->Ek_sum = 0; // 上上次偏差值初始化
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
    PID->Ek_sum += PID->Ek;

    if (PID->Ek_sum > Max_I)
        PID->Ek_sum = Max_I;
    if (PID->Ek_sum < -Max_I)
        PID->Ek_sum = -Max_I;

    PIDInc = (PID->Kp * PID->Ek) +
             (PID->Ki * PID->Ek_sum) +
             (PID->Kd * (PID->Ek - PID->last_Ek));
    PID->last_Ek = PID->Ek;
    return PIDInc;
}

/*
函数名称：Incremental_PID
函数功能：增量式PID
函数变量：
    *PID: PID参数结构体
    SetValue: 设定值
    ActualValue: 实际值
例子：
*/
/***********************************************
* @brief : 增量式PID
* @param : *PID: PID参数结构体
            SetValue：设定值
            ActualValue：实际值
* @return: 返回值
* @date  : 修改日期
* @author: 作者
************************************************/
int Incremental_PID(PID_IncTypeDef *PID, float SetValue, float ActualValue)
{


    // 计算当前偏差
    PID->Ek = SetValue - ActualValue;

    // 计算PID增量公式
    PID->OUT += ((PID->Kp * (PID->Ek - PID->last_Ek)) +                   // P部分：当前偏差与上次偏差之差
                 (PID->Ki * PID->Ek)) +                                    // I部分：当前偏差积分
                 (PID->Kd * (PID->Ek - 2 * PID->last_Ek + PID->Ek_sum));  // D部分：当前偏差、上次偏差与上上次偏差之组合

    // 更新偏差值
    PID->Ek_sum = PID->last_Ek;  // 保存上上次偏差
    PID->last_Ek = PID->Ek;   // 保存上次偏差

    if(PID->OUT >= 6000)
        PID->OUT = 6000;

    // 返回增量值
    return PID->OUT;
}


/*
函数名称：PID_clear(PID_IncTypeDef *sptr)
函数功能：清空误差
函数变量：*sptr：pid参数结构体
例子：
*/
void PID_clear(PID_IncTypeDef *sptr)
{
    sptr->last_Ek = 0; // 上次偏差值初始化
    sptr->Ek_sum = 0; // 上上次偏差值初始化
    sptr->OUT = 0;
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


