/*
 * pid.c
 *
 * Created on: 2024��1��24��
 *     Author: ������
 */
#include"PID.h"
#define my_abs(num) (((num) > 0) ? (num) : -(num))

PID_IncTypeDef Motor_Speed_PI;


/*
�������ƣ�PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd)
�������ܣ���ʼ��PID����
����������*sptr��pid�����ṹ��
���ӣ�
*/
void PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd)
{
    sptr->Ek1 = 0; // �ϴ�ƫ��ֵ��ʼ��
    sptr->Ek2 = 0; // ���ϴ�ƫ��ֵ��ʼ��
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
�������ƣ�PID_clear(PID_IncTypeDef *sptr)
�������ܣ�������
����������*sptr��pid�����ṹ��
���ӣ�
*/
void PID_clear(PID_IncTypeDef *sptr)
{
    sptr->Ek1 = 0; // �ϴ�ƫ��ֵ��ʼ��
    sptr->Ek2 = 0; // ���ϴ�ƫ��ֵ��ʼ��
    //    sptr->OUT = 0;
}

void PID_param_init(void)
{

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


