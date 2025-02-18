/*
 * gyroscope.c
 *
 *  Created on: 2023年10月28日
 *      Author: 卢劲涵
 */
#include "Gyroscope.h"

int32 Sum_icm20602_gyro_z = 0,Sum_icm20602_gyro_y = 0,Sum_icm20602_gyro_x = 0,Sum_icm20602_acc_z = 0,Sum_icm20602_acc_y = 0,Sum_icm20602_acc_x = 0;
int16 zero_icm20602_gyro_z = 0,zero_icm20602_gyro_y = 0,zero_icm20602_gyro_x = 0,zero_icm20602_acc_z = 0,zero_icm20602_acc_y = 0,zero_icm20602_acc_x = 0;
float gyro_z = 0,gyro_y = 0,gyro_x = 0,acc_z,acc_y,acc_x = 0;

void gyroscope_init(void)
{
	
	int i = 0;
	
    Sum_icm20602_gyro_z=-9;
    Sum_icm20602_gyro_y=10;
    Sum_icm20602_gyro_x=-2;
	//P52 = 0;
	icm20602_init();
    for(i=0;i<500;i++)
    {
        system_delay_ms(2);
        icm20602_get_gyro();
        Sum_icm20602_gyro_z+=icm20602_gyro_z;
        Sum_icm20602_gyro_y+=icm20602_gyro_y;
        Sum_icm20602_gyro_x+=icm20602_gyro_x;
    }
    zero_icm20602_gyro_z=Sum_icm20602_gyro_z/500;
    zero_icm20602_gyro_y=Sum_icm20602_gyro_y/500;
    zero_icm20602_gyro_x=Sum_icm20602_gyro_x/500;
	
	//P52 = 1;
}

void get_gyro(void)
{
	int16 new_gyro_x = 0;
    int16 new_gyro_z = 0;
	int16 new_gyro_y = 0;
    
	new_gyro_z=icm20602_gyro_z-zero_icm20602_gyro_z;
    gyro_z=icm20602_gyro_transition(new_gyro_z);

    acc_z = icm20602_acc_z * 9.8 / 4120;

    new_gyro_y=icm20602_gyro_y-zero_icm20602_gyro_y;
    gyro_y=icm20602_gyro_transition(new_gyro_y);


    acc_y = icm20602_acc_y * 9.8 / 4120;

    
    new_gyro_x=icm20602_gyro_x-zero_icm20602_gyro_x;
    gyro_x=icm20602_gyro_transition(new_gyro_x);


    acc_x = icm20602_acc_x * 9.8 / 4120;

}
float gyro_z_res = 0,gyro_x_res = 0,gyro_y_res = 0;

void get_gyorscope_data(void)
{
    //陀螺仪数据
    icm20602_get_gyro();
    icm20602_get_acc();
    get_gyro();
    gyro_x_res = LPF2_T2(gyro_x);
	gyro_y_res = LPF2_T2(gyro_y);
    gyro_z_res = LPF2_T2(gyro_z);
	
    IMUupdate(gyro_x/57.3,gyro_y/57.3,gyro_z/57.3,acc_x,acc_y,acc_z);
}
