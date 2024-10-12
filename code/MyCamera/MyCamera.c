#include "MyHeadfile.h"

#define USE_num image_h*3

uint8_t original_image[image_h][image_w];                //原始图像数组
uint8_t threshold_value;                                 //动态阈值自动值
uint8 image[120][188];                                   //二值化图像数组
uint8 start_point_l[2] = { 0 };                          //左边起点的x，y值
uint8 start_point_r[2] = { 0 };                          //右边起点的x，y值
uint8 start_point_detection_flag ;
int left[120] = {2};
int right[120]={185};

//存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线      points_l[num][0]是列坐标数组  points_l[num][1]是行坐标数组
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线      points_r[num][0]是列坐标数组  points_r[num][1]是行坐标数组
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数

int Endline = 20 ;           //截止行



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
    int i;
//    if(mt9v03x_finish_flag)
//    {
//        Image_Process();
    page_cnt++;
    if( page_cnt > 0)
    {
        page_cnt = 0;
        ips200_displayimage03x((const uint8 *)image, MT9V03X_W , MT9V03X_H  );
    }
    ips200_show_int(0, 300, threshold_value, 3);  //显示阈值
    for (i = Endline; i < image_h-1; i++)
    {
      //  middle[i] = (left[i] + right[i]) >> 1;//求中线

        //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
        //当然也有多组边线的找法，但是个人感觉很繁琐，不建议
//        ips200_draw_point((uint16)middle[i], (uint16)i,  RGB565_GREEN);
        ips200_draw_point((uint16)left[i], (uint16)i, RGB565_RED);
        ips200_draw_point((uint16)right[i],(uint16) i, RGB565_BLUE);

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

void Image_Process(void)
{

    if(mt9v03x_finish_flag)              //判断一幅图像是否接收完成
    {
        Get_Image(mt9v03x_image);
        Binaryzation();                 //二值化
        image_filter(image);            //膨胀腐蚀滤波
        image_draw_rectan(image);       //画黑框

        start_point_detection_flag = get_start_point(image_h - 1) ;
        if(start_point_detection_flag)
        {
            printf("正在开始八领域\n");
            Search_Line_BLY((uint16)USE_num, image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &Endline);
            printf("八邻域已结束\n");
            get_left(data_stastics_l);    //左边界提取
            get_right(data_stastics_r);   //右边界提取

        }





        mt9v03x_finish_flag = 0;
    }
}

//  使用示例      Get_image(mt9v03x_image);
/*********************图像转载*********************/
void Get_Image(uint8(*mt9v03x_image)[image_w])
{

    uint8 i = 0, j = 0, row = 0, col = 0;
    for (i = 0; i < image_h; i += zip_num)          //
    {
        for (j = 0; j <image_w; j += zip_num)     //
        {
            original_image[row][col] = mt9v03x_image[i][j];//这里的参数填写你的摄像头采集到的图像
            col++;
        }
        col = 0;
        row++;
    }
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
  if(threshold_open_or_close==1)
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
                //printf("三次进入同一个点，退出\n");
                break;
            }
            if (absolute(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
                && absolute(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
                )
            {
                //printf("\n左右相遇退出\n");
                *Endline = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
                //printf("\n在y=%d处退出\n",*Endline);
                break;
            }
            if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
            {
               // printf("\n如果左边比右边高了，左边等待右边\n");
                continue;//如果左边比右边高了，左边等待右边
            }
            if (dir_l[l_data_statics - 1] == 7
                && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
            {
                //printf("\n左边开始向下了，等待右边，等待中... \n");
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
                    //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
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

    }
    h = image_h - 2;               //从118行开始提取
    //左边
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            left[h] = points_l[j][0]+1;   //提取的线在边界线的内侧

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

    }
    h = image_h - 2;
    //右边
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            right[h] = points_r[j][0] - 1;

        }
        else continue;//每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0)break;//到最后一行退出
    }
}
