#ifndef __MyCamera_H
#define __MyCamera_H

#define white_pixel 255     //ͼ���
#define black_pixel 0       //ͼ���

#define image_h 120        //ͼ��߶�      ����
#define image_w 188         //ͼ����      ����
#define zip_num     1       //ͼ��ת�ز����� 1 ��ѹ����2 ѹ��һ��

#define Border_Max  image_w-2
#define Border_Min  1
#define Target_Column 94
#define STRAIGHT_MAX_ERR  30              //����ָ��������Ϊ�����

extern float rd_calculate;
extern uint8 bend_straight_flag;
extern uint8 Camera_process_finish_flag;
extern uint8 Camera_Wifi_Image_Send_Flag ;
extern uint8 camera_process_cnt;
extern uint16 camera_process_FPS ;
extern uint16 camera_process_cnt_show ;

extern int g_camera_mid_err;
extern int  middle[120];
extern int left[120];
extern int right[120];
extern uint8 image[120][188];
extern uint16 pro_time;
extern uint8 circle_flag;
extern uint8 left_copy[120];
extern uint8 right_copy[120];
extern uint8 middle_copy[120];

float absolute(float z);
void MyCamera_Init(void);                                       //����ͷ��ʼ��
void MyCamera_Show(void);                                       //ͼ����ʾ
void Image_Process(void);                                       //ͼ����
void Get_Image(uint8(*mt9v03x_image)[image_w]);                 //ͼ��ת��
uint8 OtsuThreshold(uint8 *image, uint16 col, uint16 row);      //���ȡ��ֵ
void Binaryzation(void);                                        //��ֵ��
void image_filter(uint8(*imag)[image_w]);
void image_draw_rectan(uint8(*image)[image_w]);
uint8 get_start_point(uint8 start_row);                         //Ѱ�ұ�����ʼ��
void Search_Line_BLY(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, int*Endline);
void get_left(uint16 total_L);
void get_right(uint16 total_R);
void middle_line(void);
int Camera_Get_MidErr(void);
void Lower_left(void);
void Lower_right(void);
void Upper_right(void);
void Upper_left(void);
void Inflection_Point(void);
void Find_Circle(void);
void Left_Roundabout(void);
void Right_Roundabout(void);
void Slope_Adding_Line( uint8 choice, uint8 startX, uint8 startY);
void Appoint_Adding_Line( uint8 choice, uint8 startX, uint8 startY, uint8 endX, uint8 endY);
void Right_Straight(void);
void Left_Straight(void);
void Lost_Right(void) ;
void Lost_Left(void) ;
void Cross_Road(void);
void Bend_Straight_Opinion(void);
void Cross_Inflection_Point(void);
void Zebra_Crossing(void);
void Growth_Direction(void);
void Circle_Upper_right(void);
void Middle_Empty(void);
void Middle_Empty_Set(uint8 dir, uint8 start_line , uint8 end_line);

#endif
