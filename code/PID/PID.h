#ifndef PID_H
#define PID_H


typedef struct {
    float target_val;             //Ŀ��ֵ���൱��SetValue
    float Kp;                       //����ϵ��Proportional
    float Ki;                       //����ϵ��Integral
    float Kd;                       //΢��ϵ��Derivative
    float Ek;                       //��ǰ���
    float Ek1;                      //ǰһ����� e(k-1)
    float Ek2;                      //������
    float OUT;                      //PID���
    float OUT1;
} PID_IncTypeDef;



/*****************************************************函数部分**************************************************************/
void PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd);
float Positional_PID(PID_IncTypeDef *PID, float SetValue, float ActualValue, float Max_I);
void PID_clear(PID_IncTypeDef *sptr);
void PID_param_init(void);
void set_pid_target(PID_IncTypeDef *pid, float temp_val);
float get_pid_target(PID_IncTypeDef *pid);
void set_p_i_d(PID_IncTypeDef *pid, float p, float i, float d);

#endif
