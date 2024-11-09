#ifndef PID_H
#define PID_H

#define motor_speed_Left_Kp  8
#define motor_speed_Left_Ki  4
#define motor_speed_Left_Kd  5

#define motor_speed_Right_Kp  8
#define motor_speed_Right_Ki  4
#define motor_speed_Right_Kd  5

#define turn_speed_Kp   0.2
#define turn_speed_Ki   0.00
#define turn_speed_Kd   11

#define straight_speed_Kp   0.2
#define straight_speed_Ki   0.00
#define straight_speed_Kd   11

typedef struct {
    float target_val;             //Ŀ��ֵ���൱��SetValue
    float Kp;                       //����ϵ��Proportional
    float Ki;                       //����ϵ��Integral
    float Kd;                       //΢��ϵ��Derivative
    float Ek;                       //��ǰ���
    float last_Ek;                      //ǰһ����� e(k-1)
    float Ek_sum;                      //������
    float OUT;                      //PID���
    float OUT1;
} PID_IncTypeDef;


extern PID_IncTypeDef Motor_Speed_PID_Left;
extern PID_IncTypeDef Motor_Speed_PID_Right;
extern PID_IncTypeDef Turn_Speed_PID;
extern PID_IncTypeDef Straight_Speed_PID;

/*****************************************************函数部分**************************************************************/
void PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd);
float Positional_PID(PID_IncTypeDef *PID, float SetValue, float ActualValue, float Max_I);
void PID_clear(PID_IncTypeDef *sptr);
void PID_param_init(void);
void set_pid_target(PID_IncTypeDef *pid, float temp_val);
float get_pid_target(PID_IncTypeDef *pid);
void set_p_i_d(PID_IncTypeDef *pid, float p, float i, float d);
int Incremental_PID(PID_IncTypeDef *PID, float SetValue, float ActualValue);

#endif
