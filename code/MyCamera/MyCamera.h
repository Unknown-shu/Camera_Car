#ifndef __MyCamera_H
#define __MyCamera_H

#define white_pixel 255     //图像黑
#define black_pixel 0       //图像白

#define image_h 120         //图像高度      行数
#define image_w 188         //图像宽度      列数
#define zip_num     1       //图像转载参数， 1 不压缩，2 压缩一倍

#define Border_Max  image_w-2
#define Border_Min  1




float absolute(float z);
void MyCamera_Init(void);                                       //摄像头初始化
void MyCamera_Show(void);                                       //图像显示
void Image_Process(void);                                       //图像处理
void Get_Image(uint8(*mt9v03x_image)[image_w]);                 //图像转换
uint8 OtsuThreshold(uint8 *image, uint16 col, uint16 row);      //大津法取阈值
void Binaryzation(void);                                        //二值化
void image_filter(uint8(*imag)[image_w]);
void image_draw_rectan(uint8(*image)[image_w]);
uint8 get_start_point(uint8 start_row);                         //寻找边线起始点
void Search_Line_BLY(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, int*Endline);
void get_left(uint16 total_L);
void get_right(uint16 total_R);

#endif
