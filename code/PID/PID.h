#ifndef PID_H
#define PID_H


typedef struct {
    float target_val;             //目标值，相当于SetValue
    float Kp;                       //比例系数Proportional
    float Ki;                       //积分系数Integral
    float Kd;                       //微分系数Derivative
    float Ek;                       //当前误差
    float Ek1;                      //前一次误差 e(k-1)
    float Ek2;                      //误差积分
    float OUT;                      //PID输出
    float OUT1;
} PID_IncTypeDef;



/*****************************************************鍑芥暟閮ㄥ垎**************************************************************/
void PID_Inc_Init(PID_IncTypeDef *sptr, float kp, float ki, float kd);
float Positional_PID(PID_IncTypeDef *PID, float SetValue, float ActualValue, float Max_I);
void PID_clear(PID_IncTypeDef *sptr);
void PID_param_init(void);
void set_pid_target(PID_IncTypeDef *pid, float temp_val);
float get_pid_target(PID_IncTypeDef *pid);
void set_p_i_d(PID_IncTypeDef *pid, float p, float i, float d);

#endif
