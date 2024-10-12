#ifndef __MyCamera_H
#define __MyCamera_H

#define white_pixel 255     //ͼ���
#define black_pixel 0       //ͼ���

#define image_h 120         //ͼ��߶�      ����
#define image_w 188         //ͼ����      ����
#define zip_num     1       //ͼ��ת�ز����� 1 ��ѹ����2 ѹ��һ��

#define Border_Max  image_w-2
#define Border_Min  1




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

#endif
