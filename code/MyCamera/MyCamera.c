#include "MyHeadfile.h"

#define USE_num image_h*3
#define ZERBA_CROSSING_ROW  70         //������ɨ������
int g_camera_mid_err;

float rd_calculate = 0;

uint8 Camera_process_finish_flag = 0;
uint8 Camera_Wifi_Image_Send_Flag = 0;
uint8 camera_process_cnt = 0;
uint16 camera_process_FPS = 0;
uint16 camera_process_cnt_show = 0;

int x,y;
uint8_t original_image[image_h][image_w];                //ԭʼͼ������
uint8_t threshold_value;                                 //��̬��ֵ�Զ�ֵ
uint8 image[120][188];                                   //��ֵ��ͼ������
uint8 start_point_l[2] = { 0 };                          //�������x��yֵ
uint8 start_point_r[2] = { 0 };                          //�ұ�����x��yֵ
uint8 start_point_detection_flag ;
int left[120]={2};       //��߽�����   ��118��ʼ��  �洢��˳���ǵ�һ�оʹ���left[1]��      //��ȡ�����������ߵ�������+1
int right[120]={185};    //�ұ߽�����   ��118��ʼ��  �洢��˳���ǵ�һ�оʹ���right[1]��     //��ȡ�����������ߵ�������-1
int middle[120]={93};    //��������
int left_tmp[120];

uint8 left_copy[120]={2};
uint8 right_copy[120]={185};
uint8 middle_copy[120]={93};

//��ŵ��x��y����
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//����      points_l[num][0]������������  points_l[num][1]������������
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//����      points_r[num][0]������������  points_r[num][1]������������
uint16 dir_r[(uint16)USE_num] = { 0 };//�����洢�ұ���������
uint16 dir_l[(uint16)USE_num] = { 0 };//�����洢�����������
uint16 data_stastics_l = 0;//ͳ������ҵ���ĸ���
uint16 data_stastics_r = 0;//ͳ���ұ��ҵ���ĸ���

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
uint8 ten_inflexion_down_l=0;    //ʮ�����¹յ�������
uint8 ten_inflexion_down_r=0;    //ʮ�����¹յ�������
uint8 ten_inflexion_up_l=0;      //ʮ�����Ϲյ�������
uint8 ten_inflexion_up_r=0;      //ʮ�����Ϲյ�������
uint8 ten_inflexion_down_l_flag=0;    //ʮ�����¹յ�
uint8 ten_inflexion_down_r_flag=0;    //ʮ�����¹յ�
uint8 ten_inflexion_up_l_flag=0;      //ʮ�����Ϲյ�
uint8 ten_inflexion_up_r_flag=0;      //ʮ�����Ϲյ�
uint8 cross_road_flag=0;       //ʮ�ֱ�־
uint8 cross_road_status=0;     //ʮ��״̬
uint8 bend_straight_flag = 0;
uint8 zebra_crossing_flag = 0;

//��������
uint8 left_2_growth_direction=0;    //�����������2�ĸ���
uint8 left_5_growth_direction=0;    //�����������5�ĸ���
uint8 left_6_growth_direction=0;    //�����������6�ĸ���
uint8 right_2_growth_direction=0;   //�ұ���������2�ĸ���
uint8 right_5_growth_direction=0;   //�ұ���������5�ĸ���
uint8 left_3_growth_direction=0;    //�����������5�ĸ���
uint8 right_6_growth_direction=0;    //�����������6�ĸ���
uint8 right_3_growth_direction=0;   //�ұ���������2�ĸ���
uint8 l_growth_direction_flag=0;    //�����������2��־λ
uint8 r_growth_direction_flag=0;    //�ұ���������2��־λ
uint8 l_growth_direction_flag35=0;    //�����������35��־λ
uint8 r_growth_direction_flag35=0;    //�ұ���������35��־λ
uint8 l_growth_direction_flag6=0;    //�����������35��־λ
uint8 r_growth_direction_flag6=0;    //�ұ���������35��־λ

uint8 right_lost_num=0;        //�ұ߶�����
uint8 left_lost_num=0;         //��߶�����
uint8 right_lost_num2=0;        //�ұ߶�����
uint8 left_lost_num2=0;         //��߶�����
uint8 right_lost_num3=0;        //�ұ߶�����
uint8 left_lost_num3=0;         //��߶�����

uint8 Lost_left_Flag=0;         //��߶��߱�־
uint8 Lost_right_Flag=0;        //�ұ߶��߱�־
uint8 Lost_left_Flag2=0;         //��߶��߱�־
uint8 Lost_right_Flag2=0;        //�ұ߶��߱�־
uint8 Lost_left_Flag3=0;         //��߶��߱�־
uint8 Lost_right_Flag3=0;        //�ұ߶��߱�־

uint8 lost_point_L_scan_line;
uint8 lost_point_R_scan_line;

uint8 circle_flag;

uint8 Circle_Begin_Flag = 0;        //������һ���յ�
uint8 Circle_Enter_Flag = 0;        //���������������Ϲյ�׼������
uint8 Circle_Static_Flag = 0;
uint8 roundabout_X=0;
uint8 roundabout_Y=0;
uint8 roundabout_Flag=0;

uint8 middle_empty_dir = 1;
uint8 middle_empty_start_line = 0;
uint8 middle_empty_end_line = 0;

int Endline = 1 ;           //��ֹ��

uint16 pro_time;            //����ʱ��

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
int camera_miderr;              //����ͷ����ƫ��

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
        system_delay_ms(500);                                                   // ����ʱ�������Ʊ�ʾ�쳣
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
        ips200_show_int(0, 300, threshold_value, 3);  //��ʾ��ֵ
        ips200_displayimage03x((const uint8 *)image, MT9V03X_W , MT9V03X_H  );
        int i;
        for (i = Endline; i < image_h-1; i++)
        {
          //  middle[i] = (left[i] + right[i]) >> 1;//������
          //�������������󣬲����ǲ��߻�����״̬����ȫ�����ʹ��һ����ߣ����������������ܸ����������
          //��ȻҲ�ж�����ߵ��ҷ������Ǹ��˸о��ܷ�����������
            ips200_draw_point((uint16)middle_copy[i], (uint16)i,  RGB565_GREEN);
            ips200_draw_point((uint16)left_copy[i], (uint16)i, RGB565_RED);
            ips200_draw_point((uint16)right_copy[i],(uint16) i, RGB565_BLUE);
        }
//        seekfree_assistant_camera_send();
    }

                              // ��ʾԭʼͼ��
//        ips200_displayimage03x((const uint8 *)mt9v03x_image, 240, 153);
//        ips200_show_gray_image(0, 0, (const uint8 *)image, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 128);     // ��ʾ��ֵ��ͼ��
//        ips200_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 128);     // ��ʾ��ֵ��ͼ��


//       if( get_start_point(image_h - 1) )  //����ҵ�����
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

//  ʹ��ʾ��      Get_image(mt9v03x_image);
/*********************ͼ��ת��*********************/
void Get_Image(uint8(*mt9v03x_image)[image_w])
{

//    uint8 i = 0, j = 0, row = 0, col = 0;
//    for (i = 0; i < image_h; i += zip_num)          //
//    {
//        for (j = 0; j <image_w; j += zip_num)     //
//        {
//            original_image[row][col] = mt9v03x_image[i][j];//����Ĳ�����д�������ͷ�ɼ�����ͼ��
//            col++;
//        }
//        col = 0;
//        row++;
//    }
    memcpy(original_image[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
}


/*********************���ȡ��ֵ*********************/
uint8 OtsuThreshold(uint8 *image, uint16 col, uint16 row)
{

    #define GrayScale 256
    uint16 Image_Width  = col; // ����ͼ��Ŀ��
    uint16 Image_Height = row; // ����ͼ��ĸ߶�
    int X; uint16 Y;
    uint8* data = image; // ָ��ͼ�����ݵ�ָ��
    int HistGram[GrayScale] = {0}; // ����ͳ�ƻҶ�ֱ��ͼ������

    uint32 Amount = 0; // ����������
    uint32 PixelBack = 0; // ǰ����������
    uint32 PixelIntegralBack = 0; // ǰ�����ػҶ�ֵ�ܺ�
    uint32 PixelIntegral = 0; // ���ػҶ�ֵ�ܺ�
    int32 PixelIntegralFore = 0; // �������ػҶ�ֵ�ܺ�
    int32 PixelFore = 0; // ������������
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // ������䷽�����ı���
    uint8 MinValue=0, MaxValue=0; // ͼ�����С�����Ҷ�ֵ
    uint8 Threshold = 0; // ����õ�����ֵ

    // ����ͼ��ͳ��ÿ���Ҷȼ����ֵ�Ƶ��
    for (Y = 0; Y <Image_Height; Y++) // ����ͼ���ÿһ��
    {
        for (X = 0; X < Image_Width; X++) // ����ͼ���ÿһ��
        {
            HistGram[(int)data[Y*Image_Width + X]]++; // ͳ�ƻҶ�ֱ��ͼ
        }
    }

    // ��ȡͼ�����С�Ҷ�ֵ
    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;

    // ��ȡͼ������Ҷ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MaxValue] == 0; MaxValue--) ;

    // ������ֵ����Сֵ��ͬ��˵��ͼ����ֻ��һ����ɫ��ֱ�ӷ������ֵ��Ϊ��ֵ
    if (MaxValue == MinValue)
    {
        return MaxValue;
    }

    // ���ͼ��ֻ�������Ҷ�ֵ�����ؽ�С�ĻҶ�ֵ��Ϊ��ֵ
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;
    }

    // �������ص�����
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];
    }

    // �������ػҶ�ֵ���ܺ�
    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;
    }

    // ʹ��Otsu�㷨���������ֵ
    SigmaB = -1; // ��ʼ�������䷽��Ϊ-1
    for (Y = MinValue; Y < MaxValue; Y++) // �������п��ܵ���ֵ
    {
          PixelBack = PixelBack + HistGram[Y];    // ǰ�����ص����ۼ�
          PixelFore = Amount - PixelBack;         // �������ص���
          OmegaBack = (double)PixelBack / Amount; // ǰ��������ռ����
          OmegaFore = (double)PixelFore / Amount; // ����������ռ����
          PixelIntegralBack += HistGram[Y] * Y;  // ǰ�����ػҶ�ֵ�ܺ�
          PixelIntegralFore = PixelIntegral - PixelIntegralBack; // �������ػҶ�ֵ�ܺ�
          MicroBack = (double)PixelIntegralBack / PixelBack; // ǰ���Ҷ�ƽ��ֵ
          MicroFore = (double)PixelIntegralFore / PixelFore; // �����Ҷ�ƽ��ֵ
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore); // ������䷽��

          // �����ǰ��䷽�����֮ǰ�������䷽����������䷽�����ֵ
          if (Sigma > SigmaB)
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }

   return Threshold; // ���ؼ���õ�����ֵ
}

/*********************��ֵ��*********************/
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

/*===========================================�������ͺ͸�ʴ====================================================*/
#define threshold_max   255*5
#define threshold_min   255*2
void image_filter(uint8(*imag)[image_w])//��̬ѧ�˲������ͺ͸�ʴ��˼��
{
    uint16 i, j;
    uint32 num = 0;


    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            //ͳ�ư˸����������ֵ
            num =
                imag[i - 1][j - 1] + imag[i - 1][j] + imag[i - 1][j + 1]
                + imag[i][j - 1] + imag[i][j + 1]
                + imag[i + 1][j - 1] + imag[i + 1][j] + imag[i + 1][j + 1];


            if (num >= threshold_max && imag[i][j] == 0)
            {

                imag[i][j] = 255;//��  ���Ը�ɺ궨�壬�������

            }
            if (num <= threshold_min && imag[i][j] == 255)
            {

                imag[i][j] = 0;//��

            }

        }
    }

}
/*===========================================���л��ڿ�====================================================*/
void image_draw_rectan(uint8(*image)[image_w])
{

    uint8 i = 0;
    for (i = 0; i < image_h; i++)     //�����0��1�к��ұ�186��187�л��ڿ�
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][image_w - 1] = 0;
        image[i][image_w - 2] = 0;

    }
    for (i = 0; i < image_w; i++)     //���Ϸ�0��1�л��ڿ�
    {
        image[0][i] = 0;
        image[1][i] = 0;
        //image[image_h-1][i] = 0;

    }
}

/*********************Ѱ�ұ߽����*********************/
uint8 get_start_point(uint8 start_row)
{
    uint8 First_start_row;
    First_start_row = start_row;
    uint8 i = 0,l_found = 0,r_found = 0;
    //����
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

        //���м�����ߣ��������
    for (i = image_w / 2; i > Border_Min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if (image[start_row][i] == 255 && image[start_row][i - 1] == 0&&
            image[start_row][i+1] == 255 && image[start_row][i - 2] == 0)
        {
//            printf("�ҵ�������image[%d][%d]\n", start_row,i);
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
//            printf("�ҵ��ұ����image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }

    if(l_found&&r_found)return 1;
    else {
//        printf("δ�ҵ����\n");
        return 0;
    }
}

/*********************��ʼ���Ⱥͼ���*********************/
//bool Point_CBH(uint8 row, uint8 col)
//{
//    int8 point_cbh_sum;
//    if(row == image_h - 1 || row == 0)
//        return 0;
//
//
//}



// �������     ������Ѳ��
// ����˵��     Berak_Flag      Ѳ�ߴ�������ֹ��ѭ��
// ����˵��     image_w         ͼ����
// ���ز���     void
// ʹ��ʾ��
// ��ע��Ϣ
void Search_Line_BLY(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, int*Endline)
{
//    uint8 i = 0, j = 0;
//    /***********�����top************/
//
//
//    //{-1,-1},{0,-1},{+1,-1},
//    //{-1, 0},       {+1, 0},
//    //{-1,+1},{0,+1},{+1,+1},
//    //�������ʱ��
//    static int8 seeds_l[8][2] = { {0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}, };
//    /***********�����end************/
//
//
//    /***********�ұ���top************/
//    uint8 search_filds_r[8][2] = { {  0  } };
//
//    //����˸�����
//    //{-1,-1},{0,-1},{+1,-1},
//    //{-1, 0},       {+1, 0},
//    //{-1,+1},{0,+1},{+1,+1},
//    //�����˳ʱ��
//    static int8 seeds_r[8][2] = { {0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1} };
//
//    /***********�ұ���end************/
        data_stastics_l = 0;
        data_stastics_r = 0;

        uint8 i = 0, j = 0;
        //��߱���
        uint8 search_filds_l[8][2] = { { 0 } };
        uint8 index_l = 0;
        uint8 temp_l[8][2] = { {  0 } };
        uint8 center_point_l[2] = {  0 };
        uint16 l_data_statics;//ͳ�����
        //����˸�����
        static int8 seeds_l[8][2] = { {0,1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,0},{1,1} };
//        static int8 seeds_l[8][2] = { {-1,0}, {-1,-1}, {0,-1}, {1,-1}, {+1,0}, {1,1}, {0,1}, {-1,1} };
        //{-1,-1},{0,-1},{+1,-1},
        //{-1, 0},       {+1, 0},
        //{-1,+1},{0,+1},{+1,+1},
        //�����˳ʱ��

        //�ұ߱���
        uint8 search_filds_r[8][2] = { {  0 } };
        uint8 center_point_r[2] = { 0 };//���������
        uint8 index_r = 0;//�����±�
        uint8 temp_r[8][2] = { {  0 } };
        uint16 r_data_statics;//ͳ���ұ�
        //����˸�����
        static int8 seeds_r[8][2] = { {0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}, };
//        static int8 seeds_r[8][2] = { {1,0}, {1,-1}, {0,-1}, {-1,-1}, {-1,0}, {-1,1}, {0,1}, {1,1} };
//        static int8 seeds_l[8][2] = { {1,0}, {1,-1}, {0,-1}, {-1,-1}, {-1,0}, {-1,1}, {0,1}, {1,1} };
//        static int8 seeds_r[8][2] = { {-1,0}, {-1,-1}, {0,-1}, {1,-1}, {+1,0}, {1,1}, {0,1}, {-1,1} };
        //{-1,-1},{0,-1},{+1,-1},
        //{-1, 0},       {+1, 0},
        //{-1,+1},{0,+1},{+1,+1},
        //�������ʱ��

        l_data_statics = *l_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������
        r_data_statics = *r_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������

        //��һ�θ��������  ���ҵ������ֵ������
        center_point_l[0] = l_start_x;//x
        center_point_l[1] = l_start_y;//y
        center_point_r[0] = r_start_x;//x
        center_point_r[1] = r_start_y;//y

            //��������ѭ��
        while (break_flag--)
        {

            //���
            for (i = 0; i < 8; i++)//����8F����
            {
                search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
                search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
            }
            //�����������䵽�Ѿ��ҵ��ĵ���
            points_l[l_data_statics][0] = center_point_l[0];//x
            points_l[l_data_statics][1] = center_point_l[1];//y
            l_data_statics++;//������һ

            //�ұ�
            for (i = 0; i < 8; i++)//����8F����
            {
                search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
                search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
            }
            //�����������䵽�Ѿ��ҵ��ĵ���
            points_r[r_data_statics][0] = center_point_r[0];//x
            points_r[r_data_statics][1] = center_point_r[1];//y

            index_l = 0;//�����㣬��ʹ��
            for (i = 0; i < 8; i++)
            {
                temp_l[i][0] = 0;//�����㣬��ʹ��
                temp_l[i][1] = 0;//�����㣬��ʹ��
            }

            //����ж�
            for (i = 0; i < 8; i++)
            {
                if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                    && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
                {
                    temp_l[index_l][0] = search_filds_l[(i)][0];
                    temp_l[index_l][1] = search_filds_l[(i)][1];
                    index_l++;
                    dir_l[l_data_statics - 1] = (i);//��¼��������
                }

                if (index_l)
                {
                    //���������
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
//                printf("���ν���ͬһ���㣬�˳�\n");
                break;
            }
            if (absolute(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
                && absolute(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
                )
            {
//                printf("\n���������˳�\n");
                *Endline = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//ȡ����ߵ�
//                printf("\n��y=%d���˳�\n",*Endline);
                break;
            }
            if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
            {
//                printf("\n�����߱��ұ߸��ˣ���ߵȴ��ұ�\n");
                continue;//�����߱��ұ߸��ˣ���ߵȴ��ұ�
            }
            if (dir_l[l_data_statics - 1] == 7
                && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//��߱��ұ߸����Ѿ�����������
            {
//                printf("\n��߿�ʼ�����ˣ��ȴ��ұߣ��ȴ���... \n");
                center_point_l[0] = (uint8)points_l[l_data_statics - 1][0];//x
                center_point_l[1] = (uint8)points_l[l_data_statics - 1][1];//y
                l_data_statics--;
            }
            r_data_statics++;//������һ

            index_r = 0;//�����㣬��ʹ��
            for (i = 0; i < 8; i++)
            {
                temp_r[i][0] = 0;//�����㣬��ʹ��
                temp_r[i][1] = 0;//�����㣬��ʹ��
            }

            //�ұ��ж�
            for (i = 0; i < 8; i++)
            {
                if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                    && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
                {
                    temp_r[index_r][0] = search_filds_r[(i)][0];
                    temp_r[index_r][1] = search_filds_r[(i)][1];
                    index_r++;//������һ
                    dir_r[r_data_statics - 1] = (i);//��¼��������
//                    printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
                }
                if (index_r)
                {

                    //���������
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

        //ȡ��ѭ������
        *l_stastic = l_data_statics;
        *r_stastic = r_data_statics;
}
/*=======================================����������������ȡ========================================================*/
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

    /*===����===*/
    left_2_growth_direction=0;
    right_2_growth_direction=0;
    left_5_growth_direction=0;
    right_5_growth_direction=0;
    left_3_growth_direction=0;
    right_6_growth_direction=0;
    left_6_growth_direction=0;
    right_3_growth_direction=0;
    /*===�����ȡ�����������===*/
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
    /*===�ұ���ȡ�����������===*/
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


    /*����ж�*/
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


    /*�ұ��ж�*/
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

    //��ʼ��
    for (i = 0;i<image_h;i++)
    {
        left[i] = Border_Min;
        left_copy[i] = Border_Min;

    }
    h = image_h - 2;               //��118�п�ʼ��ȡ
    //���
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            left[h] = points_l[j][0]+1;   //��ȡ�����ڱ߽��ߵ��ڲ�
            left_copy[h] = points_l[j][0]+1;
        }
        else continue; //ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
        if (h == 0)
        {
            break;//�����һ���˳�
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
        right[i] = Border_Max;//�ұ��߳�ʼ���ŵ����ұߣ�����߷ŵ�����ߣ�����������պ�����������߾ͻ����м䣬������ŵõ�������
        right_copy[i] = Border_Max;
    }
    h = image_h - 2;
    //�ұ�
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            right[h] = points_r[j][0] - 1;
            right_copy[h] = points_r[j][0] - 1;
        }
        else continue;//ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
        if (h == 0)break;//�����һ���˳�
    }
}

void middle_line(void)
{
    int i;
    memset(middle,0,sizeof(middle));
    for(i=119;i>Endline;i--)           //��Ч��ȫ����ȡ����
    {
        middle[i]=(right[i]+left[i])/2;
        middle_copy[i]=(right_copy[i]+left_copy[i])/2;
    }
}

void Lost_Right(void)
{
    uint8 i = 0;  // ����ѭ������ i
    right_lost_num = 0;  // ��ʼ���Ҳඪ�߼�����
    Lost_right_Flag = 0;  // ��ʼ���Ҳඪ�߱�־
    right_lost_num2 = 0;  // ��ʼ���ڶ����Ҳඪ�߼�����
    Lost_right_Flag2 = 0;  // ��ʼ���ڶ����Ҳඪ�߱�־

    // ��һ���֣�����Ҳඪ�����
    for (i = 90; i > 10; i--)
    {
        if (right[i] >= 185)
        {  // �ж��ұߵ� i �е�ֵ�Ƿ���ڵ��� 185����ʾ��ɫ��
            right_lost_num++;  // ͳ�ƶ��ߴ���
            lost_point_R_scan_line = i + 5;  // ��¼���ߵ�ɨ����
        }
        if (right_lost_num > 15)
        {  // ������ߴ������� 15
            Lost_right_Flag = 1;  // ���ö��߱�־
            break;  // �˳�ѭ��
        }
    }

    // �ڶ����֣���һ������Ҳඪ�����
    for (i = 100; i > 40; i--) {
        if (right[i] >= 185) {  // �ж��ұߵ� i �е�ֵ�Ƿ���ڵ��� 185����ʾ��ɫ��
            right_lost_num2++;  // ͳ�ƶ��ߴ���
        }
        if (right_lost_num2 > 15) {  // ������ߴ������� 15
            Lost_right_Flag2 = 1;  // ���õڶ������߱�־
            break;  // �˳�ѭ��
        }
    }

    // �������֣�����Ҳඪ�����
    for (i = 90; i > 40; i--) {
        if (right[i] >= 185) {  // �ж��ұߵ� i �е�ֵ�Ƿ���ڵ��� 185����ʾ��ɫ��
            right_lost_num3++;  // ͳ�ƶ��ߴ���
        }
        if (right_lost_num3 > 15) {  // ������ߴ������� 15
            Lost_right_Flag3 = 1;  // ���õ��������߱�־
            break;  // �˳�ѭ��
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
        if(left[i]<=3)     //��ߵ�2�е��ڰ׾Ͷ���
        {
            left_lost_num++;
            lost_point_L_scan_line=i+5;
        }
        if(left_lost_num>15)
        {
            Lost_left_Flag=1; //�ж�����·��Ƿ���
            break;
//            return;
        }
    }
    ///2
    for(i=100;i>40;i--)
    {
        if(left[i]<=3)     //�ұߵ�185�е��ڰ׾Ͷ���
        {
            left_lost_num2++;
        }
        if(left_lost_num2>10)
        {
            Lost_left_Flag2=1; //�ж�����·��Ƿ���
            break;
//            return;
        }
    }
    ///3
    for(i=90;i>40;i--)
    {
        if(left[i]<=3)     //�ұߵ�185�е��ڰ׾Ͷ���
        {
            left_lost_num3++;
        }
        if(left_lost_num3>10)
        {
            Lost_left_Flag3=1; //�ж�����·��Ƿ���
            break;
//            return;
        }
    }

}


/***********************************************
* @brief : ��ȡ����ͷ�������,ʹ��Ȩ��
* @param : void
* @return: int
* @date  : 2024��10��25��12:28:00
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
* @brief : �����ҹյ�
* @param : void
* @return: void
* @date  : 2024��10��27��22:32:05
* @author: �Ͻ���
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
* @brief : �����ҹյ�
* @param : void
* @return: void
* @date  : 2024��10��27��20:15:12
* @author: �Ͻ���
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
* @brief : �����ҹյ�
* @param : void
* @return: void
* @date  : 2024��10��29��22:27:55
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
* @brief : �����ҹյ�
* @param : void
* @return: void
* @date  : 2024��10��29��22:32:44
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
* @brief : Բ�������ҹյ�
* @param : void
* @return: void
* @date  : 2024��11��7��18:44:38
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
* @brief : �յ��ܶ�
* @param : void
* @return: void
* @date  : 2024��10��27��20:21:54
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
* @brief : �������
* @param : void
* @return: void
* @date  : 2024��11��7��19:03:27
* @author: SJX
* @note  : ��ʼ������ֹ�е��·�����������ˢ������ʼ��������ֵ����ֹ�д�
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
* @brief : �����������
* @param : void
* @return: dir ����  0ֹͣ  1��  2��
*          start_line ��ʼ��
*          end_line ��ֹ��
* @date  : 2024��11��7��19:03:27
* @author: SJX
* @note  : ��ʼ������ֹ�е��·�����������ˢ������ʼ��������ֵ����ֹ�д�
************************************************/
void Middle_Empty_Set(uint8 dir, uint8 start_line , uint8 end_line)
{
    middle_empty_dir = dir;
    middle_empty_start_line = start_line;
    middle_empty_end_line = end_line;
}

/***********************************************
* @brief : ��Բ��
* @param : void
* @return: void
* @date  : 2024��10��27��23:00:12
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
//    //��Բ��
//    if(Circle_Static_Flag == 1 || (left_straight_flag == 1 && lower_right_inflection_Flag == 1 && roundabout_Flag == 0 && Circle_Static_Flag == 0))
////        if(Circle_Static_Flag == 1 || (lower_left_inflection_Flag == 0 && lower_right_inflection_Flag == 1 && Circle_Static_Flag == 0 && roundabout_Flag == 0))
//    {
//        Slope_Adding_Line(2, lower_right_inflection_X, lower_right_inflection_Y);
//        uart_write_string(UART_2, "Find_Right_Circle\r\n");
//        Circle_Static_Flag = 1;
//        Right_Roundabout();
//    }
//    //��Բ��

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
    //�һ���
//    if((roundabout_Flag == 1 && Circle_Static_Flag == 1)|| Circle_Static_Flag == 2)
//    {
//        Right_Roundabout();
//        Circle_Static_Flag = 2;
//        uart_write_string(UART_2, "Find_Right_RD\r\n");
////        Beep_ShortRing();
//        Appoint_Adding_Line(2, 3, 117,roundabout_X, roundabout_Y);
//    }
    //�󻷵�
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
    //�����뻷
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
    //�Ѿ��뻷׼������
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
* @brief : �󻷵�
* @param : void
* @return: void
* @date  : 2024��10��27��23:28:25
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
* @brief : �һ���
* @param : void
* @return: void
* @date  : 2024��10��27��23:38:19
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
* @brief : ��ֱ��
* @param : void
* @return: void
* @date  : 2024��10��28��19:08:58
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
* @brief : ��ֱ��
* @param : void
* @return: void
* @date  : 2024��10��28��19:10:17
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
* @brief : б�ʲ���
* @param : void
* @return: choice ���߻����Ҳ��ߣ�1��2��
*          startX ��ʼX
*          startY ��ʼY

* @date  : 2024��10��27��20:21:54
* @author: SJX
************************************************/
void Slope_Adding_Line( uint8 choice, uint8 startX, uint8 startY)    //�����յ���б�������ӳ�
{

    // ֱ�� x = ky + b
    float k = 0;
    float b = 0;
    int temp=0;
    switch(choice)
    {
      case 1://����
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
      case 2://�Ҳ���  ������
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
* @brief : ָ���յ����㲹��
* @param : void
* @return: choice ���߻����Ҳ��ߣ�1��2��
*          startX ��ʼX
*          startY ��ʼY
           endX ��ֹX
           endY ��ֹY
* @date  : 2024��10��27��20:21:54
* @author: SJX
************************************************/
void Appoint_Adding_Line( uint8 choice, uint8 startX, uint8 startY, uint8 endX, uint8 endY)
{
    y = 0;
    // ֱ�� x = ky + b
    float k = 0;
    float b = 0;
    switch(choice)
    {
      case 1://����
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
      case 2://�Ҳ���
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
* @brief : ʮ��
* @param : void
* @return: void
* @date  : 2024��10��31��18:36:40
* @author: SJX
* @notes : û�м��벹,�����߱Ȳ��ߺã���ȥ�����ж���������
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
    /*-----------------------�ڶ���ʮ�ֲ���--------------------------------*/
        int l;
        uint8 start,end;           //������ֱ�ߵĵ��������
        float slope=0,distance=0;  //���б�ʺͽؾ�
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
            /*==================��ʮ��·��ǰ���в���(����б�ʽؾຯ������С���˷�����)=======================*/
            if(cross_road_status==1)
            {
//                Appoint_Adding_Line(1, left[lost_point_L_scan_line], lost_point_L_scan_line,78, 3);
//                Appoint_Adding_Line(2, right[lost_point_R_scan_line], lost_point_R_scan_line,117, 3);
                Appoint_Adding_Line(1, 78, 3,left[lost_point_L_scan_line], lost_point_L_scan_line);
                Appoint_Adding_Line(2, 117, 3,right[lost_point_R_scan_line], lost_point_R_scan_line);
//                /*===��߲���===*/
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
//                /*===�ұ߲���===*/
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


                /*====�ж��Ƿ����״̬���Ϲյ㲹��===*/
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

            /*============��ʮ��·�ں���в���(����б�ʽؾຯ������С���˷�����)===============*/
            if(cross_road_status==2)
            {
//                Appoint_Adding_Line(1, 5, 115,left[ten_inflexion_up_l], ten_inflexion_up_l);
//                Appoint_Adding_Line(2, 185, 115,right[ten_inflexion_up_l], ten_inflexion_up_r);
                Appoint_Adding_Line(1,left[ten_inflexion_up_l] , ten_inflexion_up_l,5, 115);
                Appoint_Adding_Line(2, right[ten_inflexion_up_l], ten_inflexion_up_r,185, 115);
                /*===��߲���===*/
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
//                /*===�ұ߲���===*/
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
                /*====�ж��Ƿ����ʮ��״̬==*/
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
* @brief : ʮ�ֹյ��ж�
* @param : void
* @return: void
* @date  : 2024��11��1��19:19:24
* @author: �Ͻ���
************************************************/
void Cross_Inflection_Point(void)
{
    uint8 i;
    /*====����====*/
    //ʮ�ֹյ�
    ten_inflexion_down_l=0;    //ʮ�����¹յ�������
    ten_inflexion_down_r=0;    //ʮ�����¹յ�������
    ten_inflexion_up_l=0;      //ʮ�����Ϲյ�������
    ten_inflexion_up_r=0;      //ʮ�����Ϲյ�������
    //ʮ�ֹյ��־
    ten_inflexion_down_l_flag=0;    //ʮ�����¹յ�������
    ten_inflexion_down_r_flag=0;    //ʮ�����¹յ�������
    ten_inflexion_up_l_flag=0;      //ʮ�����Ϲյ�������
    ten_inflexion_up_r_flag=0;      //ʮ�����Ϲյ�������
    /*====����====*/


    /*=====ʮ���¹յ��ж�=====*/
    /*���¹յ��ж�*/
    for(i=image_h-20;i>40;i--)
    {
        if((left[i]-left[i-1]<3)&&(left[i-1]-left[i-2]<3)&&(left[i-3]-left[i-4]>3))
        {
          ten_inflexion_down_l=i;
          ten_inflexion_down_l_flag=1;
          break;
        }

    }
    /*���¹յ��ж�*/
    for(i=image_h-20;i>40;i--)
    {
        if((right[i]-right[i-1]<3)&&(right[i-2]-right[i-1]<3)&&(right[i-4]-right[i-3]>3))
        {
           ten_inflexion_down_r=i;
           ten_inflexion_down_r_flag=1;
           break;
        }

    }
    /*=====ʮ���Ϲյ��ж�=====*/
    /*���Ϲյ��ж�*/
    for(i=image_h-40;i>15;i--)
    {
        if((left[i]-left[i+1]<3)&&(left[i+1]-left[i+2]<3)&&(left[i+3]-left[i+4]>3))
        {
            ten_inflexion_up_l=i;
            ten_inflexion_up_l_flag=1;
            break;
        }

    }
    /*���Ϲյ��ж�*/
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
* @brief : ֱ������ж�
* @param : void
* @return: void
* @date  : 2024��11��1��20:18:14
* @author: �Ͻ���
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
* @brief : ������
* @param : void
* @return: void
* @date  : 2024��11��1��21:03:49
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
    if(edge_sum >= 16 && edge_left_num > 5 && edge_right_num > 5)                      //ͣ��
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

    if(mt9v03x_finish_flag == 1)              //�ж�һ��ͼ���Ƿ�������
    {

        Get_Image(mt9v03x_image);               //ͼ��ת�� 37nsһ֡
        Binaryzation();                 //��ֵ��       5.7msһ֡
//        image_filter(image);            //���͸�ʴ�˲�      10msһ֡��������
        image_draw_rectan(image);       //���ڿ�           20nsһ֡
        start_point_detection_flag = get_start_point(image_h - 2) ;         //�ұ߽���㣬25nsһ֡
        if(start_point_detection_flag)
        {
            //������   Բ��3msһ֡  ֱ��1.8msһ֡
            Search_Line_BLY((uint16)USE_num, image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &Endline);
            Growth_Direction();
            get_left(data_stastics_l);    //��߽���ȡ
            get_right(data_stastics_r);   //�ұ߽���ȡ
            Zebra_Crossing();
            Lost_Left();                  //���·������ж�
            Lost_Right();                 //���·������ж�

            Lower_left();                   //���¶ϵ�
            Lower_right();                  //���¶ϵ�
            Upper_left();                   //���϶ϵ�
            Upper_right();                  //���϶ϵ�

            Cross_Road();                   //ʮ��
            Left_Straight();              //��ֱ���ж�
            Right_Straight();             //��ֱ���ж�

//            Inflection_Point();           //�ϵ��ܶ�
            Find_Circle();                  //��Բ��

            middle_line();                  //��ȡ����
            Middle_Empty();
            Bend_Straight_Opinion();        //�ж��Ƿ���ֱ��
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

