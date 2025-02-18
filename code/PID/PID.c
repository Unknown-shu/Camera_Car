/*
 * pid.c
 *
 * Created on: 2024��1��24��
 *     Author: ������
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
�������ƣ�PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd)
�������ܣ���ʼ��PID����
����������*sptr��pid�����ṹ��
���ӣ�
*/
void PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd)
{
    sptr->last_Ek = 0; // �ϴ�ƫ��ֵ��ʼ��
    sptr->Ek_sum = 0; // ���ϴ�ƫ��ֵ��ʼ��
    sptr->Kp = kp; // ��������
    sptr->Ki = ki; // ���ֳ���
    sptr->Kd = kd; // ΢�ֳ���
    sptr->OUT = 0;
}

/*
�������ƣ�Positional_PID(PID_IncTypeDef *PID, float SetValue, float ActualValue, float Max_I)
�������ܣ�λ��ʽpid
����������*sptr��pid�����ṹ��
          SetValue���趨ֵ
          ActualValue��ʵ��ֵ
          Max_I�����������
���ӣ�
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
�������ƣ�Incremental_PID
�������ܣ�����ʽPID
����������
    *PID: PID�����ṹ��
    SetValue: �趨ֵ
    ActualValue: ʵ��ֵ
���ӣ�
*/
/***********************************************
* @brief : ����ʽPID
* @param : *PID: PID�����ṹ��
            SetValue���趨ֵ
            ActualValue��ʵ��ֵ
* @return: ����ֵ
* @date  : �޸�����
* @author: ����
************************************************/
int Incremental_PID(PID_IncTypeDef *PID, float SetValue, float ActualValue)
{


    // ���㵱ǰƫ��
    PID->Ek = SetValue - ActualValue;

    // ����PID������ʽ
    PID->OUT += ((PID->Kp * (PID->Ek - PID->last_Ek)) +                   // P���֣���ǰƫ�����ϴ�ƫ��֮��
                 (PID->Ki * PID->Ek)) +                                    // I���֣���ǰƫ�����
                 (PID->Kd * (PID->Ek - 2 * PID->last_Ek + PID->Ek_sum));  // D���֣���ǰƫ��ϴ�ƫ�������ϴ�ƫ��֮���

    // ����ƫ��ֵ
    PID->Ek_sum = PID->last_Ek;  // �������ϴ�ƫ��
    PID->last_Ek = PID->Ek;   // �����ϴ�ƫ��

    if(PID->OUT >= 6000)
        PID->OUT = 6000;

    // ��������ֵ
    return PID->OUT;
}


/*
�������ƣ�PID_clear(PID_IncTypeDef *sptr)
�������ܣ�������
����������*sptr��pid�����ṹ��
���ӣ�
*/
void PID_clear(PID_IncTypeDef *sptr)
{
    sptr->last_Ek = 0; // �ϴ�ƫ��ֵ��ʼ��
    sptr->Ek_sum = 0; // ���ϴ�ƫ��ֵ��ʼ��
    sptr->OUT = 0;
}

/*
�������ƣ�set_pid_target(PID_IncTypeDef *pid, float temp_val)
�������ܣ�����Ŀ��ֵ
����������*sptr��pid�����ṹ��
�������أ� ��
���ӣ�
*/
void set_pid_target(PID_IncTypeDef *pid, float temp_val)
{
    pid->target_val = temp_val; // ���õ�ǰ��Ŀ��ֵ
}

/*
�������ƣ�get_pid_target(PID_IncTypeDef *pid)
�������ܣ�����pid����
����������*sptr��pid�����ṹ��
�������أ� pid->target_val ��ǰĿ��ֵ
���ӣ�
*/
float get_pid_target(PID_IncTypeDef *pid)
{
    return pid->target_val; // ��ȡ��ǰ��Ŀ��ֵ
}

/*
�������ƣ�set_p_i_d(PID_IncTypeDef *pid, float p, float i, float d)
�������ܣ�����pid����
����������*sptr��pid�����ṹ��
���ӣ�
*/
void set_p_i_d(PID_IncTypeDef *pid, float p, float i, float d)
{
    pid->Kp = p; // ���ñ���ϵ�� P
    pid->Ki = i; // ���û���ϵ�� I
    pid->Kd = d; // ����΢��ϵ�� D
}


