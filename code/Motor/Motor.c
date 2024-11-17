#include "MOTOR.H"



//Limit函数
#define LIMIT_VAL(a,min,max) ((a)<(min)?(min):((a)>(max)?(max):(a)))

int target_left = 0,target_right = 0;          //左右轮的目标速度的值

float V0 = 80;
//float turn_V0 = 70;
//float straight_V0 = 100;
float basic_V0 = 120;
int pwm_left = 0,pwm_right = 0;

#if MOTOR_MODE == 0

void Motor_Init(void)
{

    pwm_init(MOTOR_LEFT_1,12500,0);                   //初始化左电机
    gpio_init(MOTOR_LEFT_2, GPO, 1, GPO_PUSH_PULL);
    pwm_init(MOTOR_RIGHT_1,12500,0);                  //初始化右电机
    gpio_init(MOTOR_RIGHT_2, GPO, 0, GPO_PUSH_PULL);
}

/***********************************************
* @brief : 控制左右两个电机的转速和正反转,采用H桥的单极模式
* @param : pwm_left:给左电机的pwm大小，控制左电机转速
*          pwm_right:给右电机的pwm大小，控制右电机转速
* @return: void
* @date  : 2024.10.13
* @author: S
************************************************/
void MotorSetPWM(int pwm_left,int pwm_right)
{
    //对输入电机的pwm进行限幅

    if(pwm_left>MOTOR_PWM_MAX)
        pwm_left = MOTOR_PWM_MAX;
    else if(pwm_left<-MOTOR_PWM_MAX)
        pwm_left = -MOTOR_PWM_MAX;
    if(pwm_right>MOTOR_PWM_MAX)
        pwm_right = MOTOR_PWM_MAX;
    else if(pwm_right<-MOTOR_PWM_MAX)
        pwm_right = -MOTOR_PWM_MAX;
    //控制电机正反转和转速
    //左电机

//    pwm_left = 1000;
//    pwm_right = 1000;
    if(pwm_left >= 0)
    {
        gpio_set_level(MOTOR_LEFT_2, 0);
    }
    else
    {
        pwm_left = -pwm_left;
        gpio_set_level(MOTOR_LEFT_2, 1);
    }
//    pwm_set_duty(MOTOR_LEFT_1,5000+(int)(pwm_left/2));
      pwm_set_duty(MOTOR_LEFT_1,pwm_left);
    //右电机
    if(pwm_right >= 0)
    {
        gpio_set_level(MOTOR_RIGHT_2, 1);
    }
    else
    {
        pwm_right = -pwm_right;
        gpio_set_level(MOTOR_RIGHT_2, 0);
    }

//    pwm_set_duty(MOTOR_RIGHT_1,5000-(int)(pwm_right/2));
    pwm_set_duty(MOTOR_RIGHT_1,pwm_right);
}
#endif
#if MOTOR_MODE == 1

/***********************************************
* @brief : 电机初始化
* @param : void
* @return: void
* @date  : 2024年10月3日19:54:24
* @author: 钱文杰
************************************************/
void Motor_Init(void)
{
    pwm_init(MOTOR_LEFT_1,12500,0);                   //初始化左电机
    pwm_init(MOTOR_LEFT_2,12500,0);
    pwm_init(MOTOR_RIGHT_1,12500,0);                  //初始化右电机
    pwm_init(MOTOR_RIGHT_2,12500,0);
}
/***********************************************
* @brief : PWM输出
* @param : void
* @return: void
* @date  : 2024年10月3日19:58:54
* @author: 钱文杰
************************************************/
void MotorSetPWM(int pwm_left,int pwm_right)
{
    //对输入电机的pwm进行限幅

    if(pwm_left>MOTOR_PWM_MAX)
        pwm_left = MOTOR_PWM_MAX;
    else if(pwm_left<-MOTOR_PWM_MAX)
        pwm_left = -MOTOR_PWM_MAX;
    if(pwm_right>MOTOR_PWM_MAX)
        pwm_right = MOTOR_PWM_MAX;
    else if(pwm_right<-MOTOR_PWM_MAX)
        pwm_right = -MOTOR_PWM_MAX;
    //控制电机正反转和转速
    //左电机

//    pwm_left = 1000;
//    pwm_right = 1000;
    if(pwm_left >= 0)
    {
        pwm_set_duty(MOTOR_LEFT_1,pwm_left);
        pwm_set_duty(MOTOR_LEFT_2,0);
    }
    else
    {
        pwm_set_duty(MOTOR_LEFT_1,0);
        pwm_set_duty(MOTOR_LEFT_2,-pwm_left);
    }
    //右电机
    if(pwm_right >= 0)
    {
        pwm_set_duty(MOTOR_RIGHT_1,pwm_right);
        pwm_set_duty(MOTOR_RIGHT_2,0);
    }
    else
    {
        pwm_set_duty(MOTOR_RIGHT_1,0);
        pwm_set_duty(MOTOR_RIGHT_2,-pwm_right);
    }
}
#endif
/***********************************************
* @brief : 增量式PI控制电机转速
* @param : void
* @return: void
* @date  : 2024.10.13
* @author: S
************************************************/
void MotorCtrl(void)
{

//    pwm_left = Incremental_PID(&Motor_Speed_PID_Left, target_left,Encoder_speed_l);
//    pwm_right = Incremental_PID(&Motor_Speed_PID_Right, target_right,Encoder_speed_r);
    pwm_left    = Positional_PID(&Motor_Speed_PID_Left, target_left,Encoder_speed_l, 2000);
    pwm_right   = Positional_PID(&Motor_Speed_PID_Right, target_right,Encoder_speed_r, 2000);
    MotorSetPWM(pwm_left, pwm_right);
}

void Turn_Ctrl(void)
{
//    static int mid_err;
////    int middle_sum,middle_avg,speed_delta;
    int speed_delta, basic_V0_delta;
//    mid_err = Camera_Get_MidErr();

//    func_limit_ab(g_camera_mid_err,94,-94);

//    if(mid_err >= STRAIGHT_MAX_ERR)
//    {
//        speed_delta = Positional_PID(&Straight_Speed_PID,Target_Column,Target_Column - mid_err,100);
//    }
//    else
//        speed_delta = Positional_PID(&Turn_Speed_PID,Target_Column,Target_Column - mid_err,100);
//    if(bend_straight_flag == 1)
//    {
//        speed_delta = Positional_PID(&Turn_Speed_PID, Target_Column, Target_Column - g_camera_mid_err, 100);
//        target_left = turn_V0 - speed_delta ;
//        target_right = turn_V0 + speed_delta ;
//    }
//    else
//    {
//        speed_delta = Positional_PID(&Straight_Speed_PID, Target_Column, Target_Column - g_camera_mid_err, 100);
//        target_left = straight_V0 - speed_delta ;
//        target_right = straight_V0 + speed_delta ;
//    }
#if(MIDDLE_LINE_MODE == 1)
    if(circle_flag == 1)
    {
        camera_horizon = 25;
    }
    else
    {
        camera_horizon = 10 ;
//              + 0.5 * (abs(g_camera_mid_err))
    }
    g_camera_mid_err = Camera_Get_MidErr();
    if(g_camera_mid_err == 0)
    {
        g_camera_mid_err = 94;
    }
    g_camera_mid_err = Target_Column - g_camera_mid_err;
#endif
#if(MIDDLE_LINE_MODE == 2)
    g_camera_mid_err = Camera_Get_MidErr();

//    speed_delta = Positional_PID(&Turn_Speed_PID, Target_Column, Target_Column - g_camera_mid_err, 20);
#endif

    if(abs(Target_Column - g_camera_mid_err) < 3 )
    {
        g_camera_mid_err = 0;
    }
    speed_delta = Positional_PID(&Turn_Speed_PID, Target_Column, Target_Column - g_camera_mid_err, 10);
    V0 = basic_V0 - 1.1 * (abs(g_camera_mid_err));

    if(circle_flag == 1)
    {
//        V0 = 137 - (abs(g_camera_mid_err));
    }

    target_left = V0 - speed_delta ;
    target_right = V0 + speed_delta ;
    // 对 target_left 进行限幅
    func_limit_ab(target_left, 1800, -1800);

    // 对 target_right 进行限幅
    func_limit_ab(target_right, 1800, -1800);
////    ips200_show_int(0, 200, middle_avg, 3);
//    printf("%d,%d,%d\r\n",
//            g_camera_mid_err, target_left, target_right);
//    printf("%d\r\n",g_camera_mid_err);

}

void Motor_Stop(void)
{
    MotorSetPWM(0, 0);
}

