#include "MyHeadfile.h"

#define USE_num image_h*3
#define ZERBA_CROSSING_ROW  70         //斑马线扫线行数
int g_camera_mid_err;

float rd_calculate = 0;

uint8 Camera_process_finish_flag = 0;
uint8 Camera_Wifi_Image_Send_Flag = 0;
uint8 camera_process_cnt = 0;
uint16 camera_process_FPS = 0;
uint16 camera_process_cnt_show = 0;

int x,y;
uint8_t original_image[image_h][image_w];                //原始图像数组
uint8_t threshold_value;                                 //动态阈值自动值
uint8 image[120][188];                                   //二值化图像数组
uint8 start_point_l[2] = { 0 };                          //左边起点的x，y值
uint8 start_point_r[2] = { 0 };                          //右边起点的x，y值
uint8 start_point_detection_flag ;
int left[120]={2};       //左边界数组   从118开始存  存储的顺序是第一行就存在left[1]中      //提取的线在爬出线的列数上+1
int right[120]={185};    //右边界数组   从118开始存  存储的顺序是第一行就存在right[1]中     //提取的线在爬出线的列数上-1
int middle[120]={93};    //中线数组
int left_tmp[120];

uint8 left_copy[120]={2};
uint8 right_copy[120]={185};
uint8 middle_copy[120]={93};

//存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线      points_l[num][0]是列坐标数组  points_l[num][1]是行坐标数组
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线      points_r[num][0]是列坐标数组  points_r[num][1]是行坐标数组
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数

uint8 lower_left_inflection_Flag = 0;
uint8 lower_left_inflection_X = 0;
uint8 lower_left_inflection_Y = 0;
uint8 lower_right_inflection_Flag=0;
uint8 lower_right_inflection_X =0;
uint8 lower_right_inflection_Y =0;
uint8 upper_left_inflection_flag=0;
uint8 upper_left_inflection_X =0;
uint8 upper_left_inflection_Y =0;
uint8 upper_right_inflection_flag=0;
uint8 upper_right_inflection_X =0;
uint8 upper_right_inflection_Y =0;
uint8 circle_upper_right_inflection_flag=0;
uint8 circle_upper_right_inflection_X =0;
uint8 circle_upper_right_inflection_Y =0;
uint8 left_straight_flag;
uint8 right_straight_flag;
uint8 ten_inflexion_down_l=0;    //十字左下拐点行坐标
uint8 ten_inflexion_down_r=0;    //十字右下拐点行坐标
uint8 ten_inflexion_up_l=0;      //十字左上拐点行坐标
uint8 ten_inflexion_up_r=0;      //十字右上拐点行坐标
uint8 ten_inflexion_down_l_flag=0;    //十字左下拐点
uint8 ten_inflexion_down_r_flag=0;    //十字右下拐点
uint8 ten_inflexion_up_l_flag=0;      //十字左上拐点
uint8 ten_inflexion_up_r_flag=0;      //十字右上拐点
uint8 cross_road_flag=0;       //十字标志
uint8 cross_road_status=0;     //十字状态
uint8 bend_straight_flag = 0;
uint8 zebra_crossing_flag = 0;

//生长方向
uint8 left_2_growth_direction=0;    //左边生长方向2的个数
uint8 left_5_growth_direction=0;    //左边生长方向5的个数
uint8 left_6_growth_direction=0;    //左边生长方向6的个数
uint8 right_2_growth_direction=0;   //右边生长方向2的个数
uint8 right_5_growth_direction=0;   //右边生长方向5的个数
uint8 left_3_growth_direction=0;    //左边生长方向5的个数
uint8 right_6_growth_direction=0;    //左边生长方向6的个数
uint8 right_3_growth_direction=0;   //右边生长方向2的个数
uint8 l_growth_direction_flag=0;    //左边生长方向2标志位
uint8 r_growth_direction_flag=0;    //右边生长方向2标志位
uint8 l_growth_direction_flag35=0;    //左边生长方向35标志位
uint8 r_growth_direction_flag35=0;    //右边生长方向35标志位
uint8 l_growth_direction_flag6=0;    //左边生长方向35标志位
uint8 r_growth_direction_flag6=0;    //右边生长方向35标志位

uint8 right_lost_num=0;        //右边丢线数
uint8 left_lost_num=0;         //左边丢线数
uint8 right_lost_num2=0;        //右边丢线数
uint8 left_lost_num2=0;         //左边丢线数
uint8 right_lost_num3=0;        //右边丢线数
uint8 left_lost_num3=0;         //左边丢线数

uint8 Lost_left_Flag=0;         //左边丢线标志
uint8 Lost_right_Flag=0;        //右边丢线标志
uint8 Lost_left_Flag2=0;         //左边丢线标志
uint8 Lost_right_Flag2=0;        //右边丢线标志
uint8 Lost_left_Flag3=0;         //左边丢线标志
uint8 Lost_right_Flag3=0;        //右边丢线标志

uint8 lost_point_L_scan_line;
uint8 lost_point_R_scan_line;

uint8 circle_flag;

uint8 Circle_Begin_Flag = 0;        //看见第一个拐点
uint8 Circle_Enter_Flag = 0;        //看见环岛，找右上拐点准备进环
uint8 Circle_Static_Flag = 0;
uint8 roundabout_X=0;
uint8 roundabout_Y=0;
uint8 roundabout_Flag=0;

uint8 middle_empty_dir = 1;
uint8 middle_empty_start_line = 0;
uint8 middle_empty_end_line = 0;

int Endline = 1 ;           //截止行

uint16 pro_time;            //处理时间

int weight[120]=
{


0,0,0,0,0,0,0,0,0,0,//11-20
2,2,2,2,2,2,2,2,2,2,//101-110
2,2,2,2,2,2,2,2,2,2,//111-120
2,2,2,2,2,2,2,2,2,2,//30-40
7,7,7,7,7,7,7,7,7,7,//61-70
8,8,8,8,8,8,8,8,8,8,//81-90
25,25,25,25,25,25,25,25,25,25,//71-80
6,6,6,6,6,6,12,12,12,12,//51-60
4,4,4,4,4,4,4,4,4,4,//91-100
4,4,4,4,4,4,4,4,4,4,//40-50
0,0,0,0,0,0,0,0,0,0,//21-30
0,0,0,0,0,0,0,0,0,0,//0-10



};//11120
int camera_miderr;              //摄像头中线偏差

void MyCamera_Init(void)
{
    ips200_show_string(0, 0, "mt9v03x init.");
    while(1)
    {
        if(mt9v03x_init())
            ips200_show_string(0, 80, "mt9v03x reinit.");
//        if ( key_get_state(KEY_4) == KEY_SHORT_PRESS )
//            break;
        else
            break;
        system_delay_ms(500);                                                   // 短延时快速闪灯表示异常
    }
    ips200_show_string(0, 16, "init success.");
}

uint16_t page_cnt;
void MyCamera_Show(void)
{

//    if(mt9v03x_finish_flag)
//    {
//        Image_Process();

//    page_cnt++;
//    if( page_cnt > 0)
//    {
//        page_cnt = 0;
//
//    }
    if(Camera_process_finish_flag == 1)
    {
        Camera_process_finish_flag = 0;
        ips200_show_int(0, 300, threshold_value, 3);  //显示阈值
        ips200_displayimage03x((const uint8 *)image, MT9V03X_W , MT9V03X_H  );
        int i;
        for (i = Endline; i < image_h-1; i++)
        {
          //  middle[i] = (left[i] + right[i]) >> 1;//求中线
          //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
          //当然也有多组边线的找法，但是个人感觉很繁琐，不建议
            ips200_draw_point((uint16)middle_copy[i], (uint16)i,  RGB565_GREEN);
            ips200_draw_point((uint16)left_copy[i], (uint16)i, RGB565_RED);
            ips200_draw_point((uint16)right_copy[i],(uint16) i, RGB565_BLUE);
        }
//        seekfree_assistant_camera_send();
    }

                              // 显示原始图像
//        ips200_displayimage03x((const uint8 *)mt9v03x_image, 240, 153);
//        ips200_show_gray_image(0, 0, (const uint8 *)image, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 128);     // 显示二值化图像
//        ips200_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 128);     // 显示二值化图像


//       if( get_start_point(image_h - 1) )  //如果找到边线
//       {
//           ips200_draw_point(start_point_l[0] , start_point_l[1] , RGB565_RED);
//           ips200_draw_point(start_point_r[0] , start_point_r[1] , RGB565_RED);
//           ips200_show_string(0, 200, "Find_Point      ");
//       }
//       else
//       {
//           ips200_show_string(0, 200, "UNFind_Point    ");
//       }

//        mt9v03x_finish_flag = 0;
////    }
}

//  使用示例      Get_image(mt9v03x_image);
/*********************图像转载*********************/
void Get_Image(uint8(*mt9v03x_image)[image_w])
{

//    uint8 i = 0, j = 0, row = 0, col = 0;
//    for (i = 0; i < image_h; i += zip_num)          //
//    {
//        for (j = 0; j <image_w; j += zip_num)     //
//        {
//            original_image[row][col] = mt9v03x_image[i][j];//这里的参数填写你的摄像头采集到的图像
//            col++;
//        }
//        col = 0;
//        row++;
//    }
    memcpy(original_image[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
}


/*********************大津法取阈值*********************/
uint8 OtsuThreshold(uint8 *image, uint16 col, uint16 row)
{

    #define GrayScale 256
    uint16 Image_Width  = col; // 定义图像的宽度
    uint16 Image_Height = row; // 定义图像的高度
    int X; uint16 Y;
    uint8* data = image; // 指向图像数据的指针
    int HistGram[GrayScale] = {0}; // 用于统计灰度直方图的数组

    uint32 Amount = 0; // 总像素数量
    uint32 PixelBack = 0; // 前景像素数量
    uint32 PixelIntegralBack = 0; // 前景像素灰度值总和
    uint32 PixelIntegral = 0; // 像素灰度值总和
    int32 PixelIntegralFore = 0; // 背景像素灰度值总和
    int32 PixelFore = 0; // 背景像素数量
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 用于类间方差计算的变量
    uint8 MinValue=0, MaxValue=0; // 图像的最小和最大灰度值
    uint8 Threshold = 0; // 计算得到的阈值

    // 遍历图像，统计每个灰度级出现的频率
    for (Y = 0; Y <Image_Height; Y++) // 遍历图像的每一行
    {
        for (X = 0; X < Image_Width; X++) // 遍历图像的每一列
        {
            HistGram[(int)data[Y*Image_Width + X]]++; // 统计灰度直方图
        }
    }

    // 获取图像的最小灰度值
    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;

    // 获取图像的最大灰度值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MaxValue] == 0; MaxValue--) ;

    // 如果最大值和最小值相同，说明图像中只有一个颜色，直接返回最大值作为阈值
    if (MaxValue == MinValue)
    {
        return MaxValue;
    }

    // 如果图像只有两个灰度值，返回较小的灰度值作为阈值
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;
    }

    // 计算像素的总数
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];
    }

    // 计算像素灰度值的总和
    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;
    }

    // 使用Otsu算法计算最佳阈值
    SigmaB = -1; // 初始化最大类间方差为-1
    for (Y = MinValue; Y < MaxValue; Y++) // 遍历所有可能的阈值
    {
          PixelBack = PixelBack + HistGram[Y];    // 前景像素点数累加
          PixelFore = Amount - PixelBack;         // 背景像素点数
          OmegaBack = (double)PixelBack / Amount; // 前景像素所占比例
          OmegaFore = (double)PixelFore / Amount; // 背景像素所占比例
          PixelIntegralBack += HistGram[Y] * Y;  // 前景像素灰度值总和
          PixelIntegralFore = PixelIntegral - PixelIntegralBack; // 背景像素灰度值总和
          MicroBack = (double)PixelIntegralBack / PixelBack; // 前景灰度平均值
          MicroFore = (double)PixelIntegralFore / PixelFore; // 背景灰度平均值
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore); // 计算类间方差

          // 如果当前类间方差大于之前的最大类间方差，更新最大类间方差和阈值
          if (Sigma > SigmaB)
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }

   return Threshold; // 返回计算得到的阈值
}

/*********************二值化*********************/
void Binaryzation(void)
{
  uint8 i,j;
  if(threshold_open_or_close == 0)
      threshold_value = OtsuThreshold(original_image[0], image_w, image_h);
  else
 threshold_value = OtsuThreshold(original_image[0], image_w, image_h)+threshold_value_add;
  for(i = 0;i<image_h;i++)
  {

      for(j = 0;j<image_w;j++)
      {
          if(original_image[i][j]>threshold_value)//175
              image[i][j] = white_pixel;
          else
              image[i][j] = black_pixel;
//              image[i][j] = white_pixel;
      }
  }
}

/*===========================================进行膨胀和腐蚀====================================================*/
#define threshold_max   255*5
#define threshold_min   255*2
void image_filter(uint8(*imag)[image_w])//形态学滤波，膨胀和腐蚀的思想
{
    uint16 i, j;
    uint32 num = 0;


    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            //统计八个方向的像素值
            num =
                imag[i - 1][j - 1] + imag[i - 1][j] + imag[i - 1][j + 1]
                + imag[i][j - 1] + imag[i][j + 1]
                + imag[i + 1][j - 1] + imag[i + 1][j] + imag[i + 1][j + 1];


            if (num >= threshold_max && imag[i][j] == 0)
            {

                imag[i][j] = 255;//白  可以搞成宏定义，方便更改

            }
            if (num <= threshold_min && imag[i][j] == 255)
            {

                imag[i][j] = 0;//黑

            }

        }
    }

}
/*===========================================进行画黑框====================================================*/
void image_draw_rectan(uint8(*image)[image_w])
{

    uint8 i = 0;
    for (i = 0; i < image_h; i++)     //给左边0、1列和右边186、187列画黑框
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][image_w - 1] = 0;
        image[i][image_w - 2] = 0;

    }
    for (i = 0; i < image_w; i++)     //给上方0、1行画黑框
    {
        image[0][i] = 0;
        image[1][i] = 0;
        //image[image_h-1][i] = 0;

    }
}

/*********************寻找边界起点*********************/
uint8 get_start_point(uint8 start_row)
{
    uint8 First_start_row;
    First_start_row = start_row;
    uint8 i = 0,l_found = 0,r_found = 0;
    //清零
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

        //从中间往左边，先找起点
    for (i = image_w / 2; i > Border_Min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if (image[start_row][i] == 255 && image[start_row][i - 1] == 0&&
            image[start_row][i+1] == 255 && image[start_row][i - 2] == 0)
        {
//            printf("找到左边起点image[%d][%d]\n", start_row,i);
            l_found = 1;
            break;
        }
    }

    for (i = image_w / 2; i < Border_Max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if (image[start_row][i-1] == 255 && image[start_row][i] == 255 &&
            image[start_row][i + 1] == 0 && image[start_row][i + 2] == 0)
        {
//            printf("找到右边起点image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }

    if(l_found&&r_found)return 1;
    else {
//        printf("未找到起点\n");
        return 0;
    }
}

/*********************起始点差比和计算*********************/
//bool Point_CBH(uint8 row, uint8 col)
//{
//    int8 point_cbh_sum;
//    if(row == image_h - 1 || row == 0)
//        return 0;
//
//
//}



// 函数简介     八领域巡线
// 参数说明     Berak_Flag      巡线次数，防止死循环
// 参数说明     image_w         图像宽度
// 返回参数     void
// 使用示例
// 备注信息
void Search_Line_BLY(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, int*Endline)
{
//    uint8 i = 0, j = 0;
//    /***********左变量top************/
//
//
//    //{-1,-1},{0,-1},{+1,-1},
//    //{-1, 0},       {+1, 0},
//    //{-1,+1},{0,+1},{+1,+1},
//    //这个是逆时针
//    static int8 seeds_l[8][2] = { {0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}, };
//    /***********左变量end************/
//
//
//    /***********右变量top************/
//    uint8 search_filds_r[8][2] = { {  0  } };
//
//    //定义八个邻域
//    //{-1,-1},{0,-1},{+1,-1},
//    //{-1, 0},       {+1, 0},
//    //{-1,+1},{0,+1},{+1,+1},
//    //这个是顺时针
//    static int8 seeds_r[8][2] = { {0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1} };
//
//    /***********右变量end************/
        data_stastics_l = 0;
        data_stastics_r = 0;

        uint8 i = 0, j = 0;
        //左边变量
        uint8 search_filds_l[8][2] = { { 0 } };
        uint8 index_l = 0;
        uint8 temp_l[8][2] = { {  0 } };
        uint8 center_point_l[2] = {  0 };
        uint16 l_data_statics;//统计左边
        //定义八个邻域
        static int8 seeds_l[8][2] = { {0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1} };
//        static int8 seeds_l[8][2] = { {-1,0}, {-1,-1}, {0,-1}, {1,-1}, {+1,0}, {1,1}, {0,1}, {-1,1} };
        //{-1,-1},{0,-1},{+1,-1},
        //{-1, 0},       {+1, 0},
        //{-1,+1},{0,+1},{+1,+1},
        //这个是顺时针

        //右边变量
        uint8 search_filds_r[8][2] = { {  0 } };
        uint8 center_point_r[2] = { 0 };//中心坐标点
        uint8 index_r = 0;//索引下标
        uint8 temp_r[8][2] = { {  0 } };
        uint16 r_data_statics;//统计右边
        //定义八个邻域
        static int8 seeds_r[8][2] = { {0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}, };
//        static int8 seeds_r[8][2] = { {1,0}, {1,-1}, {0,-1}, {-1,-1}, {-1,0}, {-1,1}, {0,1}, {1,1} };
//        static int8 seeds_l[8][2] = { {1,0}, {1,-1}, {0,-1}, {-1,-1}, {-1,0}, {-1,1}, {0,1}, {1,1} };
//        static int8 seeds_r[8][2] = { {-1,0}, {-1,-1}, {0,-1}, {1,-1}, {+1,0}, {1,1}, {0,1}, {-1,1} };
        //{-1,-1},{0,-1},{+1,-1},
        //{-1, 0},       {+1, 0},
        //{-1,+1},{0,+1},{+1,+1},
        //这个是逆时针

        l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
        r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

        //第一次更新坐标点  将找到的起点值传进来
        center_point_l[0] = l_start_x;//x
        center_point_l[1] = l_start_y;//y
        center_point_r[0] = r_start_x;//x
        center_point_r[1] = r_start_y;//y

            //开启邻域循环
        while (break_flag--)
        {

            //左边
            for (i = 0; i < 8; i++)//传递8F坐标
            {
                search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
                search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
            }
            //中心坐标点填充到已经找到的点内
            points_l[l_data_statics][0] = center_point_l[0];//x
            points_l[l_data_statics][1] = center_point_l[1];//y
            l_data_statics++;//索引加一

            //右边
            for (i = 0; i < 8; i++)//传递8F坐标
            {
                search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
                search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
            }
            //中心坐标点填充到已经找到的点内
            points_r[r_data_statics][0] = center_point_r[0];//x
            points_r[r_data_statics][1] = center_point_r[1];//y

            index_l = 0;//先清零，后使用
            for (i = 0; i < 8; i++)
            {
                temp_l[i][0] = 0;//先清零，后使用
                temp_l[i][1] = 0;//先清零，后使用
            }

            //左边判断
            for (i = 0; i < 8; i++)
            {
                if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                    && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
                {
                    temp_l[index_l][0] = search_filds_l[(i)][0];
                    temp_l[index_l][1] = search_filds_l[(i)][1];
                    index_l++;
                    dir_l[l_data_statics - 1] = (i);//记录生长方向
                }

                if (index_l)
                {
                    //更新坐标点
                    center_point_l[0] = temp_l[0][0];//x
                    center_point_l[1] = temp_l[0][1];//y
                    for (j = 0; j < index_l; j++)
                    {
                        if (center_point_l[1] > temp_l[j][1])
                        {
                            center_point_l[0] = temp_l[j][0];//x
                            center_point_l[1] = temp_l[j][1];//y
                        }
                    }
                }

            }
            if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
                && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
                ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                    && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
            {
//                printf("三次进入同一个点，退出\n");
                break;
            }
            if (absolute(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
                && absolute(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
                )
            {
//                printf("\n左右相遇退出\n");
                *Endline = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
//                printf("\n在y=%d处退出\n",*Endline);
                break;
            }
            if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
            {
//                printf("\n如果左边比右边高了，左边等待右边\n");
                continue;//如果左边比右边高了，左边等待右边
            }
            if (dir_l[l_data_statics - 1] == 7
                && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
            {
//                printf("\n左边开始向下了，等待右边，等待中... \n");
                center_point_l[0] = (uint8)points_l[l_data_statics - 1][0];//x
                center_point_l[1] = (uint8)points_l[l_data_statics - 1][1];//y
                l_data_statics--;
            }
            r_data_statics++;//索引加一

            index_r = 0;//先清零，后使用
            for (i = 0; i < 8; i++)
            {
                temp_r[i][0] = 0;//先清零，后使用
                temp_r[i][1] = 0;//先清零，后使用
            }

            //右边判断
            for (i = 0; i < 8; i++)
            {
                if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                    && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
                {
                    temp_r[index_r][0] = search_filds_r[(i)][0];
                    temp_r[index_r][1] = search_filds_r[(i)][1];
                    index_r++;//索引加一
                    dir_r[r_data_statics - 1] = (i);//记录生长方向
//                    printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
                }
                if (index_r)
                {

                    //更新坐标点
                    center_point_r[0] = temp_r[0][0];//x
                    center_point_r[1] = temp_r[0][1];//y
                    for (j = 0; j < index_r; j++)
                    {
                        if (center_point_r[1] > temp_r[j][1])
                        {
                            center_point_r[0] = temp_r[j][0];//x
                            center_point_r[1] = temp_r[j][1];//y
                        }
                    }

                }
            }
        }

        //取出循环次数
        *l_stastic = l_data_statics;
        *r_stastic = r_data_statics;
}
/*=======================================进行生长方个数提取========================================================*/
uint8 r1=0,r2=0,r3=0,r4=0,r5=0,r6=0,r7=0,r8=0;
uint8 l1=0,l2=0,l3=0,l4=0,l5=0,l6=0,l7=0,l8=0;
void Growth_Direction(void)
{
    uint16 i;
    r1=0;
    r2=0;
    r3=0;
    r4=0;
    r5=0;
    r6=0;
    r7=0;
    r8=0;

    l1=0;
    l2=0;
    l3=0;
    l4=0;
    l5=0;
    l6=0;
    l7=0;
    l8=0;

    /*===清零===*/
    left_2_growth_direction=0;
    right_2_growth_direction=0;
    left_5_growth_direction=0;
    right_5_growth_direction=0;
    left_3_growth_direction=0;
    right_6_growth_direction=0;
    left_6_growth_direction=0;
    right_3_growth_direction=0;
    /*===左边提取生长方向个数===*/
    for(i=0;i<=data_stastics_l;i++)
    {
        if(dir_l[i]==2)
            left_2_growth_direction++;
        if(dir_l[i]==5)
            left_5_growth_direction++;
        if(dir_l[i]==3)
            left_3_growth_direction++;
        if(dir_l[i]==1)
            l1++;
        if(dir_l[i]==2)
            l2++;
        if(dir_l[i]==3)
            l3++;
        if(dir_l[i]==4)
            l4++;
        if(dir_l[i]==5)
            l5++;
        if(dir_l[i]==6)
        {
            l6++;
            left_6_growth_direction++;
        }
        if(dir_l[i]==7)
            l7++;
        if(dir_l[i]==0)
            l8++;
    }
    /*===右边提取生长方向个数===*/
    for(i=0;i<=data_stastics_r;i++)
    {
        if(dir_r[i]==2)
          right_2_growth_direction++;
        if(dir_r[i]==5)
          right_5_growth_direction++;
        if(dir_r[i]==3)
          right_3_growth_direction++;
        if(dir_r[i]==1)
            r1++;
        if(dir_r[i]==2)
            r2++;
        if(dir_r[i]==3)
            r3++;
        if(dir_r[i]==4)
            r4++;
        if(dir_r[i]==5)
            r5++;
        if(dir_r[i]==6)
        {
            r6++;
            right_6_growth_direction++;
        }
        if(dir_r[i]==7)
            r7++;
        if(dir_r[i]==0)
            r8++;
    }


    /*左边判断*/
    if(left_2_growth_direction>20&&left_5_growth_direction>20)
    {
        l_growth_direction_flag=1;
    }
    else
    {
        l_growth_direction_flag=0;
    }
    if(left_3_growth_direction>20&&left_5_growth_direction>20)
    {
        l_growth_direction_flag35=1;
    }
    else
    {
        l_growth_direction_flag35=0;
    }
    if(left_6_growth_direction>25)
    {
        l_growth_direction_flag6=1;
    }
    else
    {
        l_growth_direction_flag6=0;
    }


    /*右边判断*/
    if(right_2_growth_direction>20&&right_5_growth_direction>20)
    {
        r_growth_direction_flag=1;
    }
    else
    {
        r_growth_direction_flag=0;
    }
    if(right_3_growth_direction>20&&right_5_growth_direction>20)
    {
        r_growth_direction_flag35=1;
    }
    else
    {
        r_growth_direction_flag35=0;
    }
    if(right_6_growth_direction>25)
    {
        r_growth_direction_flag6=1;
    }
    else
    {
        r_growth_direction_flag6=0;
    }
}

float absolute(float z)
{
    z = z< 0 ? (-z) : z;
    return z;
}

void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;

    //初始化
    for (i = 0;i<image_h;i++)
    {
        left[i] = Border_Min;
        left_copy[i] = Border_Min;

    }
    h = image_h - 2;               //从118行开始提取
    //左边
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            left[h] = points_l[j][0]+1;   //提取的线在边界线的内侧
            left_copy[h] = points_l[j][0]+1;
        }
        else continue; //每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)
        {
            break;//到最后一行退出
        }
    }
}
void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;

    for (i = 0; i < image_h; i++)
    {
        right[i] = Border_Max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
        right_copy[i] = Border_Max;
    }
    h = image_h - 2;
    //右边
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            right[h] = points_r[j][0] - 1;
            right_copy[h] = points_r[j][0] - 1;
        }
        else continue;//每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)break;//到最后一行退出
    }
}

void middle_line(void)
{
    int i;
    memset(middle,0,sizeof(middle));
    for(i=119;i>Endline;i--)           //有效行全部提取中线
    {
        middle[i]=(right[i]+left[i])/2;
        middle_copy[i]=(right_copy[i]+left_copy[i])/2;
    }
}

void Lost_Right(void)
{
    uint8 i = 0;  // 定义循环变量 i
    right_lost_num = 0;  // 初始化右侧丢线计数器
    Lost_right_Flag = 0;  // 初始化右侧丢线标志
    right_lost_num2 = 0;  // 初始化第二个右侧丢线计数器
    Lost_right_Flag2 = 0;  // 初始化第二个右侧丢线标志

    // 第一部分：检测右侧丢线情况
    for (i = 90; i > 10; i--)
    {
        if (right[i] >= 185)
        {  // 判断右边第 i 行的值是否大于等于 185（表示白色）
            right_lost_num++;  // 统计丢线次数
            lost_point_R_scan_line = i + 5;  // 记录丢线的扫描行
        }
        if (right_lost_num > 15)
        {  // 如果丢线次数超过 15
            Lost_right_Flag = 1;  // 设置丢线标志
            break;  // 退出循环
        }
    }

    // 第二部分：进一步检测右侧丢线情况
    for (i = 100; i > 40; i--) {
        if (right[i] >= 185) {  // 判断右边第 i 行的值是否大于等于 185（表示白色）
            right_lost_num2++;  // 统计丢线次数
        }
        if (right_lost_num2 > 15) {  // 如果丢线次数超过 15
            Lost_right_Flag2 = 1;  // 设置第二个丢线标志
            break;  // 退出循环
        }
    }

    // 第三部分：检测右侧丢线情况
    for (i = 90; i > 40; i--) {
        if (right[i] >= 185) {  // 判断右边第 i 行的值是否大于等于 185（表示白色）
            right_lost_num3++;  // 统计丢线次数
        }
        if (right_lost_num3 > 15) {  // 如果丢线次数超过 15
            Lost_right_Flag3 = 1;  // 设置第三个丢线标志
            break;  // 退出循环
        }
    }
}
void Lost_Left(void)
{
    uint8 i=0;
    left_lost_num=0;
    left_lost_num2=0;
    left_lost_num3=0;
    Lost_left_Flag=0;
    Lost_left_Flag2=0;
    Lost_left_Flag3=0;
    //1
    for(i=90;i>10;i--)
    {
        if(left[i]<=3)     //左边第2列等于白就丢线
        {
            left_lost_num++;
            lost_point_L_scan_line=i+5;
        }
        if(left_lost_num>15)
        {
            Lost_left_Flag=1; //判断左边下方是否丢线
            break;
//            return;
        }
    }
    ///2
    for(i=100;i>40;i--)
    {
        if(left[i]<=3)     //右边第185列等于白就丢线
        {
            left_lost_num2++;
        }
        if(left_lost_num2>10)
        {
            Lost_left_Flag2=1; //判断左边下方是否丢线
            break;
//            return;
        }
    }
    ///3
    for(i=90;i>40;i--)
    {
        if(left[i]<=3)     //右边第185列等于白就丢线
        {
            left_lost_num3++;
        }
        if(left_lost_num3>10)
        {
            Lost_left_Flag3=1; //判断左边下方是否丢线
            break;
//            return;
        }
    }

}


/***********************************************
* @brief : 获取摄像头中线误差,使用权重
* @param : void
* @return: int
* @date  : 2024年10月25日12:28:00
* @author: SJX
************************************************/
int Camera_Get_MidErr(void)
{

#if(MIDDLE_LINE_MODE == 1)
    int i;
    int err_sum = 0, err = 0;
    for(i=40;i < 80; i++)
    {
        err_sum += middle_copy[i] ;
    }
    err = err_sum / 40;
    return err;
#endif
#if(MIDDLE_LINE_MODE == 2)
    int i;
    int weight_sum=0;
    for(i=119;i >= 0;i--)
    {
        if(middle_copy[i]!=0)
        {
            camera_miderr += weight[i] * (Target_Column-middle[i]);
            weight_sum += weight[i];
        }
    }
    camera_miderr /= weight_sum;
    return camera_miderr;
#endif
}
/***********************************************
* @brief : 左下找拐点
* @param : void
* @return: void
* @date  : 2024年10月27日22:32:05
* @author: 严建晨
************************************************/
void Lower_left(void)
{
    lower_left_inflection_Flag=0;
    lower_left_inflection_X =3;
    lower_left_inflection_Y =105;

        for(y=image_h-3;y>Endline+10;y--)
        {
            if(y>30)
            {
                if((left[y]-left[y-4])>5&&left[y-4]==2&&(left[y]-left[y+2])<5&&left[y]>10)
                {
                    lower_left_inflection_Flag=1;
                    lower_left_inflection_X =(uint8)left[y];
                    lower_left_inflection_Y =(uint8)y;
//                    Slope_Adding_Line(1, lower_left_inflection_X, lower_left_inflection_Y);
//                    uart_write_string(UART_2, "Find_Lower_Left\r\n");
                    return;
                }
            }

         }
}
/***********************************************
* @brief : 右下找拐点
* @param : void
* @return: void
* @date  : 2024年10月27日20:15:12
* @author: 严建晨
************************************************/
void Lower_right(void)
{
    lower_right_inflection_Flag=0;
    lower_right_inflection_X = 186;
    lower_right_inflection_Y = 105;
    for(y=image_h-10;y>(Endline+10);y--)
    {
        if(y>15)
        {
            if((right[y-4]-right[y])>5&&right[y-4]==185&&(right[y+2]-right[y])<5&&right[y]<177)
            {
                lower_right_inflection_Flag=1;
                lower_right_inflection_X =(uint8)right[y];
                lower_right_inflection_Y =(uint8)y;
//                ips200_draw_point(lower_right_inflection_X, lower_right_inflection_Y, RGB565_SandBeige);
//                uart_write_string(UART_2, "Find_Lower_Right\r\n");
//                Slope_Adding_Line(2, lower_right_inflection_X, lower_right_inflection_Y);
                return;
            }
        }

     }
}
/***********************************************
* @brief : 左上找拐点
* @param : void
* @return: void
* @date  : 2024年10月29日22:27:55
* @author: SJX
************************************************/
void Upper_left(void)
{
    uint8 h=image_h-3;
    upper_left_inflection_flag=0;
    upper_left_inflection_X =0;
    upper_left_inflection_Y =0;
//    if(Lost_left_Flag==1)
//    {
        for(h=4;h<50;h++)
        {
            if((left[h-2] - left[h] < 6 ) && (left[h] - left[h+2]) < 6 && (left[h] - left[h+10] > 20 ))
            {
            //                        uart_write_string(UART_2, "Find_Upper_Left\r\n");
                   upper_left_inflection_flag=1;
                   upper_left_inflection_X =(uint8)left[h+5];
                   upper_left_inflection_Y =h+5;
                   return;
            }
        }
//    }

}
/***********************************************
* @brief : 右上找拐点
* @param : void
* @return: void
* @date  : 2024年10月29日22:32:44
* @author: SJX
************************************************/
void Upper_right(void)
{
    uint8 h=image_h-3;
    upper_right_inflection_flag=0;
    upper_right_inflection_X =0;
    upper_right_inflection_Y =0;
    if(Lost_right_Flag==1)
    {
        for(h=4;h<50;h++)
        {
            if((right[h-2] - right[h] < 6 ) && (right[h] - right[h+2]) < 6 && (right[h] - right[h+10] > 20 ))
            {
            //                        uart_write_string(UART_2, "Find_Upper_Left\r\n");
                   upper_right_inflection_flag=1;
                   upper_right_inflection_X =(uint8)right[h+5];
                   upper_right_inflection_Y =h+5;
                   return;
            }
        }
    }
}

/***********************************************
* @brief : 圆环右上找拐点
* @param : void
* @return: void
* @date  : 2024年11月7日18:44:38
* @author: SJX
************************************************/
void Circle_Upper_right(void)
{
    uint8 h=image_h-3;
    circle_upper_right_inflection_flag=0;
    circle_upper_right_inflection_X =0;
    circle_upper_right_inflection_Y =0;
//    if(Lost_right_Flag==1)
//    {
        for(h=8;h<90;h++)
        {
//            if((right[h-2] - right[h] < 6 ) && (right[h] - right[h+2]) < 6 && (right[h] - right[h+10] > 20 ))
            if((right[h] - right[h-3])<0 && (right[h-3] - right[h-6])<0
                    && (right[h] - right[h+3])<0 && (right[h+3] - right[h+6])<0)
            {
            //                        uart_write_string(UART_2, "Find_Upper_Left\r\n");
                circle_upper_right_inflection_flag=1;
                circle_upper_right_inflection_X =(uint8)right[h+5];
                circle_upper_right_inflection_Y =h+5;
                   return;
            }
        }
//    }
}
/***********************************************
* @brief : 拐点总断
* @param : void
* @return: void
* @date  : 2024年10月27日20:21:54
* @author: SJX
************************************************/
void Inflection_Point(void)
{
    Lower_left();
    Lower_right();
    Upper_left();
    Upper_right();
}
/***********************************************
* @brief : 中线清空
* @param : void
* @return: void
* @date  : 2024年11月7日19:03:27
* @author: SJX
* @note  : 起始行在终止行的下方，从下往上刷，故起始行坐标数值比终止行大
************************************************/
void Middle_Empty(void)
{
    if(middle_empty_start_line < middle_empty_end_line)
    {
        uint8 tmp;
        tmp = middle_empty_start_line;
        middle_empty_start_line = middle_empty_end_line;
        middle_empty_end_line = tmp;
    }
    uint8 i;
    if(middle_empty_dir == 0)
    {
        return;
    }
    else if(middle_empty_dir == 1)
    {
        for(i = middle_empty_start_line; i > middle_empty_end_line; i-- )
        {
            middle_copy[i] = 2;
        }
    }
    else
    {
        for(i = middle_empty_start_line; i > middle_empty_end_line; i-- )
        {
            middle_copy[i] = 185;
        }
    }
    Middle_Empty_Set(0, 1, 1);
}
/***********************************************
* @brief : 中线清空设置
* @param : void
* @return: dir 方向  0停止  1左  2右
*          start_line 起始行
*          end_line 终止行
* @date  : 2024年11月7日19:03:27
* @author: SJX
* @note  : 起始行在终止行的下方，从下往上刷，故起始行坐标数值比终止行大
************************************************/
void Middle_Empty_Set(uint8 dir, uint8 start_line , uint8 end_line)
{
    middle_empty_dir = dir;
    middle_empty_start_line = start_line;
    middle_empty_end_line = end_line;
}

/***********************************************
* @brief : 找圆环
* @param : void
* @return: void
* @date  : 2024年10月27日23:00:12
* @author: SJX
************************************************/
void Find_Circle(void)
{
    static uint16 circle_lost_cnt = 0;
    static uint16 circle_process_cnt = 0;
    static uint16 circle_out_cnt = 0;
    if((left_straight_flag == 1 && right_straight_flag == 1 && Lost_left_Flag == 0 && Lost_right_Flag == 0 && Circle_Static_Flag < 3)
                || cross_road_flag == 2 )
    //        && (Circle_Static_Flag <= 2 || Circle_Static_Flag >= 5))
        {
            circle_lost_cnt++;
            if(circle_lost_cnt >= 10)
            {
                circle_lost_cnt = 0;
                circle_process_cnt = 0;
//                uart_write_string(UART_0, "Circle_OUT\r\n");
                Circle_Static_Flag = 0;
                circle_flag = 0;
                Middle_Empty_Set(0, 40, 2);
                return ;
            }

        }
    if(circle_process_cnt > 30 || circle_out_cnt > 40 )
    {
        circle_out_cnt = 0;
        circle_process_cnt = 0;
//        uart_write_string(UART_0, "Circle_OUT\r\n");
        Circle_Static_Flag = 0;
        circle_flag = 0;
        Middle_Empty_Set(0, 40, 2);
        return ;
    }
//    memcpy(left_tmp[0], left[0], 120);
//    printf("%d, %d\r\n",lower_right_inflection_X, lower_right_inflection_Y);
//    //右圆环
//    if(Circle_Static_Flag == 1 || (left_straight_flag == 1 && lower_right_inflection_Flag == 1 && roundabout_Flag == 0 && Circle_Static_Flag == 0))
////        if(Circle_Static_Flag == 1 || (lower_left_inflection_Flag == 0 && lower_right_inflection_Flag == 1 && Circle_Static_Flag == 0 && roundabout_Flag == 0))
//    {
//        Slope_Adding_Line(2, lower_right_inflection_X, lower_right_inflection_Y);
//        uart_write_string(UART_2, "Find_Right_Circle\r\n");
//        Circle_Static_Flag = 1;
//        Right_Roundabout();
//    }
//    //左圆环

    if(Circle_Static_Flag == 1 || (upper_right_inflection_flag == 0 && lower_left_inflection_Flag == 1 && lower_right_inflection_Flag == 0 && Circle_Static_Flag == 0 ))
    {

//        Slope_Adding_Line(1, lower_left_inflection_X, lower_left_inflection_Y);
//        Appoint_Adding_Line(1, , 95,79, 5);
        Appoint_Adding_Line(1, 79, 5,5, 95);
//        uart_write_string(UART_0, "Find_Left_Circle\r\n");
        circle_flag = 0;
        Left_Roundabout();
        Circle_Static_Flag = 1;
        circle_process_cnt++;
        if((Lost_right_Flag == 0 && roundabout_Y > 25 && roundabout_Flag == 1 && Circle_Static_Flag == 1)|| Circle_Static_Flag == 2)
        {
            circle_process_cnt = 0;
            circle_flag = 0;
            Circle_Static_Flag = 2;
        }
    }
    //右环岛
//    if((roundabout_Flag == 1 && Circle_Static_Flag == 1)|| Circle_Static_Flag == 2)
//    {
//        Right_Roundabout();
//        Circle_Static_Flag = 2;
//        uart_write_string(UART_2, "Find_Right_RD\r\n");
////        Beep_ShortRing();
//        Appoint_Adding_Line(2, 3, 117,roundabout_X, roundabout_Y);
//    }
    //左环岛
    if(Circle_Static_Flag == 2)
        {
            circle_process_cnt++;
            Left_Roundabout();
//            uart_write_string(UART_0, "Find_Left_RD\r\n");
    //        Beep_ShortRing();
//            Appoint_Adding_Line(1, 187, 120,roundabout_X, roundabout_Y);
            Appoint_Adding_Line(1, 79, 5,5, 95);
            if(upper_left_inflection_flag == 1 && roundabout_Y > 42 && (roundabout_Y - upper_left_inflection_Y  ) > 20)
            {
                circle_process_cnt = 0;
                Circle_Static_Flag = 3;
                circle_flag = 1;
            }
        }
    //拉线入环
    if(Circle_Static_Flag == 3)
    {
//        circle_process_cnt++;
//        Appoint_Adding_Line(2, upper_left_inflection_X, upper_left_inflection_Y, right[32], 68);
        Appoint_Adding_Line(2, 27, 19, right[80], 80);
        Middle_Empty_Set(1, 24, 2);
        Left_Roundabout();
//        uart_write_string(UART_0, "Find_Left_Upper\r\n");
        if(roundabout_Flag == 0 && upper_left_inflection_flag == 0 && left_straight_flag == 1 && right_straight_flag == 0)
        {
            circle_process_cnt = 0;
            Circle_Static_Flag = 4;
        }
    }
    //已经入环准备出环
    if(Circle_Static_Flag == 4)
    {
//        uart_write_string(UART_0, "Find_IN\r\n");
        Circle_Upper_right();
        if(circle_upper_right_inflection_flag == 1)
        {
            Circle_Static_Flag = 5;
            circle_process_cnt = 0;
            circle_out_cnt = 0;
        }
    }
    if(Circle_Static_Flag == 5)
    {
        Circle_Upper_right();
        circle_out_cnt++;
        if(circle_upper_right_inflection_flag == 1)
        {
            Appoint_Adding_Line(2, 2, 24, circle_upper_right_inflection_X, circle_upper_right_inflection_Y);
//            middle_empty_start_line = circle_upper_right_inflection_Y;
//            middle_empty_end_line = 2;
            Middle_Empty_Set(1, circle_upper_right_inflection_Y, 2);
//            Middle_Empty(1, circle_upper_right_inflection_Y, 2);
        }
        else
        {
            Appoint_Adding_Line(2, 2, 24, right[90], 90);
            Middle_Empty_Set(1, 40, 2);
//            Middle_Empty(1, 40, 2);
        }
    }


//    printf("%d, %d\r\n", Circle_Static_Flag, circle_flag);

//    if(Lost_left_Flag == 0 && )
//    double a[2], b[2], c[2];
//    a[0] = left[5];
//    a[1] = 5;
//    b[0] = left[15];
//    b[1] = 15;
//    c[0] = left[20];
//    c[1] = 20;
//    rd_calculate = curvature(a, b, c);

//    ips200_show_float(50, 18*5, tmp, 4, 6);
//    printf("%f\r\n", tmp);
//    Right_Roundabout();
//    printf("%d, %d, %d, %d\r\n",left_straight_flag, right_straight_flag, Lost_left_Flag, Lost_right_Flag);
//    printf("%d, %d, %d, %d\r\n",roundabout_X, roundabout_Y, upper_left_inflection_Y, lower_left_inflection_Y);
//    printf("%d, %d, %d\r\n",circle_upper_right_inflection_flag, circle_upper_right_inflection_X, circle_upper_right_inflection_Y);
//    printf("%d, %d, %d, %d, %d \r\n",left_straight_flag, right_straight_flag, lower_left_inflection_Flag,lower_right_inflection_Flag,roundabout_Flag);
//    Right_Roundabout();

//    printf("%d, %d, %d, %d, %d, %d, %d\r\n",Circle_Static_Flag, cross_road_flag, Circle_Static_Flag, cross_road_status, circle_process_cnt, circle_lost_cnt, circle_out_cnt);
//    printf("%d, %d, %d, %d\r\n",roundabout_Flag, Circle_Static_Flag, lower_right_inflection_Flag,Circle_Enter_Flag);
//    uart_write_byte(UART_2, roundabout_Flag);

//    if(Circle_Enter_Flag == 1 && lower_right_inflection_Flag == 0 && )
}
/***********************************************
* @brief : 左环岛
* @param : void
* @return: void
* @date  : 2024年10月27日23:28:25
* @author: SJX
************************************************/
void Left_Roundabout(void)
{

    roundabout_X=0;
    roundabout_Y=0;
    roundabout_Flag=0;
//    for(y=0+10;y<lower_right_inflection_Y-6;y++)
    for(y=50;y>15;y--)
    {
        if((left[y]-left[y-8]) > 2 && (left[y]-left[y+2])<10 && right_straight_flag == 1 && Lost_left_Flag == 1 && (lower_left_inflection_Y - y) > 20)
        {
            y+=4;
            roundabout_Flag=1;
            roundabout_X =(uint8)left[y];
            roundabout_Y =(uint8)y;
            return;
        }
//        if((left_tmp[y]-left_tmp[y-8]) > 2 && (left_tmp[y]-left_tmp[y+2])<10 && right_straight_flag == 1 && Lost_left_Flag == 1)
//        {
//            y+=4;
//            roundabout_Flag=1;
//            roundabout_X =(uint8)left_tmp[y];
//            roundabout_Y =(uint8)y;
//            return;
//        }
     }
}
/***********************************************
* @brief : 右环岛
* @param : void
* @return: void
* @date  : 2024年10月27日23:38:19
* @author: SJX
************************************************/
void Right_Roundabout(void)
{
    roundabout_X=0;
    roundabout_Y=0;
    roundabout_Flag=0;
    for(y=0 + 3 ;y < lower_right_inflection_Y;y++)
//        for(y=image_h-3;y>10;y--)
    {
        if((right[y-8]-right[y])>2&& (right[y+2]-right[y])<10 && left_straight_flag == 1 && right_straight_flag == 0 && Lost_right_Flag == 1)
        {
            y+=4;
            roundabout_Flag=1;
            roundabout_X =(uint8)right[y];
            roundabout_Y =(uint8)y;
            return;
        }
     }
}
/***********************************************
* @brief : 左直线
* @param : void
* @return: void
* @date  : 2024年10月28日19:08:58
* @author: SJX Copy
************************************************/
float l_k = 0;
void Left_Straight(void)
{
    float k1,k2;
    left_straight_flag = 0;
    float l_slope2=0,l_slope3=0,l_distance2=0,l_distance3=0;
    caculate_distance(25,80,left,&l_slope3,&l_distance3);
    k2=l_slope3;
//    caculate_distance(30,40,left,&l_slope2,&l_distance2);
//    k1=l_slope2;
//    if(absolute(k1-k2)<0.8)
////        left_straight_flag = 1;
//    l_k=absolute(k1-k2);
    if(k2 <= 0.95)
    {
        left_straight_flag = 1;
    }
//    printf("%f\r\n",k2);
//        float k1,k2=0;
//        left_straight_flag = 0;
//        float l_slope2=0,l_slope3=0,l_distance2=0,l_distance3=0;
//        caculate_distance(4,80,left,&l_slope3,&l_distance3);
//        k2=l_slope3;
//        printf("%d\r\n",k2);
}
/***********************************************
* @brief : 右直线
* @param : void
* @return: void
* @date  : 2024年10月28日19:10:17
* @author: SJX Copy
************************************************/
float r_k = 0;
void Right_Straight(void)
{
        float k1,k2;
        right_straight_flag=0;
        float l_slope2=0,l_slope3=0,l_distance2=0,l_distance3=0;
        caculate_distance(25,80,right,&l_slope3,&l_distance3);
        k2=l_slope3;
//        caculate_distance(5,50,right,&l_slope2,&l_distance2);
//        k1=l_slope2;
//        if(absolute(k1-k2)<0.3)
//            right_straight_flag=1;
//        r_k=absolute(k1-k2);
        if(k2 <= 0.96)
        {
            right_straight_flag = 1;
        }

}


/***********************************************
* @brief : 斜率补线
* @param : void
* @return: choice 左补线还是右补线，1左，2右
*          startX 起始X
*          startY 起始Y

* @date  : 2024年10月27日20:21:54
* @author: SJX
************************************************/
void Slope_Adding_Line( uint8 choice, uint8 startX, uint8 startY)    //看到拐点延斜率向上延长
{

    // 直线 x = ky + b
    float k = 0;
    float b = 0;
    int temp=0;
    switch(choice)
    {
      case 1://左补线
        {
            k = (float)(((float)left[lower_left_inflection_Y+1] - (float)left[lower_left_inflection_Y+5]) /(-4));
            b = (float)((float)left[lower_left_inflection_Y+5]- (float)(lower_left_inflection_Y+5) * k);

            for(y = startY; y >(Endline); y--)
            {
                temp = (int)(k* y + b);
                if(temp<180&&temp>10)
                {
//                    left[y]=temp;
                    left_copy[y]=temp;
                }
            }
            break;
        }
      case 2://右补线  待测试
      {

           k = (float)(((float)right[lower_right_inflection_Y+1] - (float)right[lower_right_inflection_Y+5]) /(-4));
           b = (float)((float)right[lower_right_inflection_Y+5]- (float)(lower_right_inflection_Y+5) * k);

           for(y = startY; y >(Endline); y--)
           {
                temp = (int)(k* y + b);
                if(temp<180&&temp>10)
                {
//                    right[y]=temp;
                    right_copy[y]=temp;
                }
           }
           break;
       }
    }
}
/***********************************************
* @brief : 指定终点和起点补线
* @param : void
* @return: choice 左补线还是右补线，1左，2右
*          startX 起始X
*          startY 起始Y
           endX 终止X
           endY 终止Y
* @date  : 2024年10月27日20:21:54
* @author: SJX
************************************************/
void Appoint_Adding_Line( uint8 choice, uint8 startX, uint8 startY, uint8 endX, uint8 endY)
{
    y = 0;
    // 直线 x = ky + b
    float k = 0;
    float b = 0;
    switch(choice)
    {
      case 1://左补线
        {
            k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
            b = (float)startX - (float)startY * k;
            for(y = startY; y < endY; y++)
            {
                if( (uint8)(k * y + b)>185)
                {
//                    left[y] = 185;
                    left_copy[y] = 185;
                }

                else if( (uint8)(k * y + b)<2)
                {
//                    left[y] = 2;
                    left_copy[y] = 2;
                }

                else
                {
//                    left[y] = (k * y + b);
                    left_copy[y] = (k * y + b);
                }
            }
            break;
        }
      case 2://右补线
        {
            k = (float)((float)endX - (float)startX) / (float)((float)endY - (float)startY);
            b = (float)startX - (float)startY * k;
            for(y = startY; y < endY; y++)
            {
                if( (uint8)(k * y + b)>185)
                {
//                    right[y]=185;
                    right_copy[y]=185;
                }

                else if ( (uint8)(k * y + b)<2)
                {
//                    right[y]=2;
                    right_copy[y]=2;
                }

                else
                {
//                    right[y]= (k * y + b);
                    right_copy[y]= (k * y + b);
                }
            }
            break;
        }
    }
}
/***********************************************
* @brief : 十字
* @param : void
* @return: void
* @date  : 2024年10月31日18:36:40
* @author: SJX
* @notes : 没有加入补,不补线比补线好，出去经常有东西看不到
************************************************/
void Cross_Road(void)
{
    Cross_Inflection_Point();
//    cross_road_flag = 0;
//    if(Lost_left_Flag == 1 && Lost_right_Flag == 1 && lower_left_inflection_Flag == 1 &&
//       lower_right_inflection_Flag == 1 && upper_left_inflection_flag == 1 && upper_right_inflection_flag == 1)
//    {
//        cross_road_flag = 1;
//        printf("%d\r\n",cross_road_flag);
//    }
//    if(cross_road_flag == 1)
//    {
//        Appoint_Adding_Line(1,lower_left_inflection_X, lower_left_inflection_Y, upper_left_inflection_X, upper_left_inflection_Y);
//        Appoint_Adding_Line(2,lower_right_inflection_X, lower_right_inflection_Y, upper_right_inflection_X, upper_right_inflection_Y);
//    }
    /*-----------------------第二版十字补线--------------------------------*/
        int l;
        uint8 start,end;           //存放拟合直线的点个列坐标
        float slope=0,distance=0;  //存放斜率和截距
//        if(bend_straight_flag==0 && Lost_left_Flag == 1 && Lost_right_Flag == 1 && Lost_left_Flag3 == 1 && Lost_right_Flag3 == 1 &&
//           cross_road_flag == 0 && cross_road_status == 0 && ten_inflexion_down_l_flag == 1 && ten_inflexion_down_r_flag == 1)
        if(Lost_left_Flag == 1 && Lost_right_Flag == 1 && Lost_left_Flag3 == 1 && Lost_right_Flag3 == 1 &&
           cross_road_flag == 0 && cross_road_status == 0 && ten_inflexion_down_l_flag == 1 && ten_inflexion_down_r_flag == 1)

        {
           cross_road_flag=1;
           cross_road_status=1;
//           uart_write_string(UART_0, "Find_Cross\r\n");
//           Beep_Start();
        }
        if(cross_road_flag==1)
        {
            /*==================进十字路口前进行补线(调用斜率截距函数和最小二乘法函数)=======================*/
            if(cross_road_status==1)
            {
//                Appoint_Adding_Line(1, left[lost_point_L_scan_line], lost_point_L_scan_line,78, 3);
//                Appoint_Adding_Line(2, right[lost_point_R_scan_line], lost_point_R_scan_line,117, 3);
                Appoint_Adding_Line(1, 78, 3,left[lost_point_L_scan_line], lost_point_L_scan_line);
                Appoint_Adding_Line(2, 117, 3,right[lost_point_R_scan_line], lost_point_R_scan_line);
//                /*===左边补线===*/
//                start=ten_inflexion_down_l+3;
//                end=ten_inflexion_down_l+15;
//                caculate_distance(start,end,left,&slope,&distance);
//                for(l=ten_inflexion_down_l;l>2;l--)
//                {
//                    if((slope*(l)+distance)>185)
//                        left[l]=185;
//                    else if((slope*(l)+distance)<2)
//                        left[l]=2;
//                    else
//                        left[l]=slope*(l)+distance;
//                }
//                /*===右边补线===*/
//                start=ten_inflexion_down_r+3;
//                end=ten_inflexion_down_r+15;
//                caculate_distance(start,end,right,&slope,&distance);
//                for(l=ten_inflexion_down_r;l>2;l--)
//                {
//                    if((slope*(l)+distance)>185)
//                        right[l]=185;
//                    else if((slope*(l)+distance)<2)
//                        right[l]=2;
//                    else
//                        right[l]=slope*(l)+distance;
//                }


                /*====判断是否进行状态二上拐点补线===*/
                if(ten_inflexion_up_l_flag==1&&ten_inflexion_up_r_flag==1)
                {
                    cross_road_status=2;
                }
                if(Lost_right_Flag3==0||Lost_left_Flag3==0)
                {
                    cross_road_flag=0;
                    cross_road_status=0;
                    Beep_Stop();
                }
            }

            /*============进十字路口后进行补线(调用斜率截距函数和最小二乘法函数)===============*/
            if(cross_road_status==2)
            {
//                Appoint_Adding_Line(1, 5, 115,left[ten_inflexion_up_l], ten_inflexion_up_l);
//                Appoint_Adding_Line(2, 185, 115,right[ten_inflexion_up_l], ten_inflexion_up_r);
                Appoint_Adding_Line(1,left[ten_inflexion_up_l] , ten_inflexion_up_l,5, 115);
                Appoint_Adding_Line(2, right[ten_inflexion_up_l], ten_inflexion_up_r,185, 115);
                /*===左边补线===*/
//                start=ten_inflexion_up_l-13;
//                end=ten_inflexion_up_l-3;
//                caculate_distance(start,end,left,&slope,&distance);
//                for(l=ten_inflexion_up_l;l<90;l++)
//                {
//                    if((slope*(l)+distance)>185)
//                    {
//                        left[l]=185;
//                        left_copy[l]=185;
//                    }
//
//                    else if((slope*(l)+distance)<2)
//                    {
//                        left[l]=2;
//                        left_copy[l]=2;
//                    }
//
//                    else
//                    {
//                        left[l]=slope*(l)+distance;
//                        left_copy[l]=slope*(l)+distance;
//                    }
//
//                }
//                /*===右边补线===*/
//                start=ten_inflexion_up_r-13;
//                end=ten_inflexion_up_r-3;
//                caculate_distance(start,end,right,&slope,&distance);
//                for(l=ten_inflexion_up_r;l<90;l++)
//                {
//                    if((slope*(l)+distance)>185)
//                    {
//                        right[l]=185;
//                        right_copy[l]=185;
//                    }
//
//                    else if((slope*(l)+distance)<2)
//                    {
//                        right[l]=2;
//                        right_copy[l]=2;
//                    }
//                    else
//                    {
//                        right[l]=slope*(l)+distance;
//                        right_copy[l]=slope*(l)+distance;
//                    }
//                }
                /*====判断是否结束十字状态==*/
                if(ten_inflexion_up_l_flag==0||ten_inflexion_up_r_flag==0||Lost_right_Flag3==0||Lost_left_Flag3==0)
                {
                    cross_road_flag=0;
                    cross_road_status=0;
//                    Beep_Stop();
//                    uart_write_string(UART_0, "Cross_OUT\r\n");
                }
            }

        }
//        printf("%d, %d, %d, %d, %d, %d, %d, %d, %d, %d\r\n ",bend_straight_flag, Lost_left_Flag, Lost_right_Flag,
//                        Lost_left_Flag3 , Lost_right_Flag3,ten_inflexion_down_l_flag ,ten_inflexion_down_r_flag,
//                        ten_inflexion_up_l_flag,ten_inflexion_up_r_flag,cross_road_status );
//        printf("%d, %d, %d \r\n",ten_inflexion_up_l, ten_inflexion_up_r, cross_road_status);

}
/***********************************************
* @brief : 十字拐点判断
* @param : void
* @return: void
* @date  : 2024年11月1日19:19:24
* @author: 严建程
************************************************/
void Cross_Inflection_Point(void)
{
    uint8 i;
    /*====清零====*/
    //十字拐点
    ten_inflexion_down_l=0;    //十字左下拐点列坐标
    ten_inflexion_down_r=0;    //十字右下拐点列坐标
    ten_inflexion_up_l=0;      //十字左上拐点列坐标
    ten_inflexion_up_r=0;      //十字右上拐点列坐标
    //十字拐点标志
    ten_inflexion_down_l_flag=0;    //十字左下拐点列坐标
    ten_inflexion_down_r_flag=0;    //十字右下拐点列坐标
    ten_inflexion_up_l_flag=0;      //十字左上拐点列坐标
    ten_inflexion_up_r_flag=0;      //十字右上拐点列坐标
    /*====清零====*/


    /*=====十字下拐点判断=====*/
    /*左下拐点判断*/
    for(i=image_h-20;i>40;i--)
    {
        if((left[i]-left[i-1]<3)&&(left[i-1]-left[i-2]<3)&&(left[i-3]-left[i-4]>3))
        {
          ten_inflexion_down_l=i;
          ten_inflexion_down_l_flag=1;
          break;
        }

    }
    /*右下拐点判断*/
    for(i=image_h-20;i>40;i--)
    {
        if((right[i]-right[i-1]<3)&&(right[i-2]-right[i-1]<3)&&(right[i-4]-right[i-3]>3))
        {
           ten_inflexion_down_r=i;
           ten_inflexion_down_r_flag=1;
           break;
        }

    }
    /*=====十字上拐点判断=====*/
    /*左上拐点判断*/
    for(i=image_h-40;i>15;i--)
    {
        if((left[i]-left[i+1]<3)&&(left[i+1]-left[i+2]<3)&&(left[i+3]-left[i+4]>3))
        {
            ten_inflexion_up_l=i;
            ten_inflexion_up_l_flag=1;
            break;
        }

    }
    /*右上拐点判断*/
    for(i=image_h-40;i>15;i--)
    {
        if((right[i+1]-right[i]<3)&&(right[i+2]-right[i+1]<3)&&(right[i+4]-right[i+3]>3))
        {
           ten_inflexion_up_r=i;
           ten_inflexion_up_r_flag=1;
           break;
        }

    }
}
/***********************************************
* @brief : 直线弯道判断
* @param : void
* @return: void
* @date  : 2024年11月1日20:18:14
* @author: 严建程
************************************************/
float straight_k_err=0;
float straight_k_1=0;
float straight_k_2=0;
void Bend_Straight_Opinion(void)
{
    float k1,k2;
    float l_slope2=0,l_slope3=0,l_distance2=0,l_distance3=0;
    bend_straight_flag=0;
    caculate_distance(15,35,middle,&l_slope3,&l_distance3);
    k2=l_slope3;

    caculate_distance(45,80,middle,&l_slope2,&l_distance2);
    k1=l_slope2;

    if((absolute(k1-k2)<0.08))//&&(Lost_left_Flag==0)&&(Lost_right_Flag==0)
    bend_straight_flag=1;
    else
    bend_straight_flag=0;
    straight_k_err=absolute(k1-k2);
//    printf("%f\r\n", straight_k_err);
}
/***********************************************
* @brief : 斑马线
* @param : void
* @return: void
* @date  : 2024年11月1日21:03:49
* @author: SJX
************************************************/
void Zebra_Crossing(void)
{
    uint8 zebra_row;
    int i;
    int edge_left_num = 0;
    int edge_right_num = 0;
    int edge_sum = 0;
    zebra_crossing_flag = 0;
    for(zebra_row = ZERBA_CROSSING_ROW; zebra_row < ZERBA_CROSSING_ROW+3; zebra_row++)
    {
        for(i=left[zebra_row];i<94;i++)
        {
            if((image[zebra_row][i] == white_pixel && image[zebra_row][i+1] == white_pixel) &&
               (image[zebra_row][i+2] == black_pixel && image[zebra_row][i+3] == black_pixel))
            {
                edge_left_num++;
            }
//            printf("%d,%d\r\n",image[zebra_row][i]);
        }
        for(i=right[zebra_row];i>94;i--)
        {
            if((image[zebra_row][i] == white_pixel && image[zebra_row][i-1] == white_pixel) &&
               (image[zebra_row][i-2] == black_pixel && image[zebra_row][i-3] == black_pixel))
            {
                edge_right_num++;
            }
        }
    }
    edge_sum = edge_left_num + edge_right_num;
    if(edge_sum > 200)
        edge_sum = 0;
    if(edge_left_num > 200)
        edge_left_num = 0;
    if(edge_sum >= 16 && edge_left_num > 5 && edge_right_num > 5)                      //停车
    {
        system_delay_ms(210);
        zebra_crossing_flag = 1;
        Car_Stop();
        Beep_ShortRing();
//        system_delay_ms(1000);

    }
    else
        zebra_crossing_flag = 0;
//    printf("%d, %d, %d ,%d\r\n",edge_left_num, edge_right_num, left[zebra_row], right[zebra_row]);
//    printf("%d, %d, %d\r\n",edge_left_num, edge_right_num, edge_sum);
}

void Image_Process(void)
{

    if(mt9v03x_finish_flag == 1)              //判断一幅图像是否接收完成
    {

        Get_Image(mt9v03x_image);               //图像转载 37ns一帧
        Binaryzation();                 //二值化       5.7ms一帧
//        image_filter(image);            //膨胀腐蚀滤波      10ms一帧？？？？
        image_draw_rectan(image);       //画黑框           20ns一帧
        start_point_detection_flag = get_start_point(image_h - 2) ;         //找边界起点，25ns一帧
        if(start_point_detection_flag)
        {
            //八领域   圆环3ms一帧  直线1.8ms一帧
            Search_Line_BLY((uint16)USE_num, image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &Endline);
            Growth_Direction();
            get_left(data_stastics_l);    //左边界提取
            get_right(data_stastics_r);   //右边界提取
            Zebra_Crossing();
            Lost_Left();                  //左下方丢线判断
            Lost_Right();                 //右下方丢线判断

            Lower_left();                   //左下断点
            Lower_right();                  //右下断点
            Upper_left();                   //左上断点
            Upper_right();                  //右上断点

            Cross_Road();                   //十字
            Left_Straight();              //左直线判断
            Right_Straight();             //右直线判断

//            Inflection_Point();           //断点总断
            Find_Circle();                  //找圆环

            middle_line();                  //提取中线
            Middle_Empty();
            Bend_Straight_Opinion();        //判断是否是直线
//            g_camera_mid_err = Camera_Get_MidErr();
//            printf("%d\r\n",g_camera_mid_err);
//            printf("%d ,%d ,%d\r\n ",target_left,target_right,g_camera_mid_err);
            camera_process_cnt++;
            camera_process_cnt_show++;

        }
//        pro_time = stop_time - start_time ;
        mt9v03x_finish_flag = 0;
        Camera_process_finish_flag = 1;
        Camera_Wifi_Image_Send_Flag = 1;

    }
}

