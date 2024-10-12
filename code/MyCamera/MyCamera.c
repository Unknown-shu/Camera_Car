#include "MyHeadfile.h"

#define USE_num image_h*3

uint8_t original_image[image_h][image_w];                //ԭʼͼ������
uint8_t threshold_value;                                 //��̬��ֵ�Զ�ֵ
uint8 image[120][188];                                   //��ֵ��ͼ������
uint8 start_point_l[2] = { 0 };                          //�������x��yֵ
uint8 start_point_r[2] = { 0 };                          //�ұ�����x��yֵ
uint8 start_point_detection_flag ;
int left[120] = {2};
int right[120]={185};

//��ŵ��x��y����
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//����      points_l[num][0]������������  points_l[num][1]������������
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//����      points_r[num][0]������������  points_r[num][1]������������
uint16 dir_r[(uint16)USE_num] = { 0 };//�����洢�ұ���������
uint16 dir_l[(uint16)USE_num] = { 0 };//�����洢�����������
uint16 data_stastics_l = 0;//ͳ������ҵ���ĸ���
uint16 data_stastics_r = 0;//ͳ���ұ��ҵ���ĸ���

int Endline = 20 ;           //��ֹ��



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
    ips200_show_int(0, 300, threshold_value, 3);  //��ʾ��ֵ
    for (i = Endline; i < image_h-1; i++)
    {
      //  middle[i] = (left[i] + right[i]) >> 1;//������

        //�������������󣬲����ǲ��߻�����״̬����ȫ�����ʹ��һ����ߣ����������������ܸ����������
        //��ȻҲ�ж�����ߵ��ҷ������Ǹ��˸о��ܷ�����������
//        ips200_draw_point((uint16)middle[i], (uint16)i,  RGB565_GREEN);
        ips200_draw_point((uint16)left[i], (uint16)i, RGB565_RED);
        ips200_draw_point((uint16)right[i],(uint16) i, RGB565_BLUE);

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

void Image_Process(void)
{

    if(mt9v03x_finish_flag)              //�ж�һ��ͼ���Ƿ�������
    {
        Get_Image(mt9v03x_image);
        Binaryzation();                 //��ֵ��
        image_filter(image);            //���͸�ʴ�˲�
        image_draw_rectan(image);       //���ڿ�

        start_point_detection_flag = get_start_point(image_h - 1) ;
        if(start_point_detection_flag)
        {
            printf("���ڿ�ʼ������\n");
            Search_Line_BLY((uint16)USE_num, image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &Endline);
            printf("�������ѽ���\n");
            get_left(data_stastics_l);    //��߽���ȡ
            get_right(data_stastics_r);   //�ұ߽���ȡ

        }





        mt9v03x_finish_flag = 0;
    }
}

//  ʹ��ʾ��      Get_image(mt9v03x_image);
/*********************ͼ��ת��*********************/
void Get_Image(uint8(*mt9v03x_image)[image_w])
{

    uint8 i = 0, j = 0, row = 0, col = 0;
    for (i = 0; i < image_h; i += zip_num)          //
    {
        for (j = 0; j <image_w; j += zip_num)     //
        {
            original_image[row][col] = mt9v03x_image[i][j];//����Ĳ�����д�������ͷ�ɼ�����ͼ��
            col++;
        }
        col = 0;
        row++;
    }
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
                //printf("���ν���ͬһ���㣬�˳�\n");
                break;
            }
            if (absolute(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
                && absolute(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
                )
            {
                //printf("\n���������˳�\n");
                *Endline = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//ȡ����ߵ�
                //printf("\n��y=%d���˳�\n",*Endline);
                break;
            }
            if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
            {
               // printf("\n�����߱��ұ߸��ˣ���ߵȴ��ұ�\n");
                continue;//�����߱��ұ߸��ˣ���ߵȴ��ұ�
            }
            if (dir_l[l_data_statics - 1] == 7
                && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//��߱��ұ߸����Ѿ�����������
            {
                //printf("\n��߿�ʼ�����ˣ��ȴ��ұߣ��ȴ���... \n");
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
                    //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
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

    }
    h = image_h - 2;               //��118�п�ʼ��ȡ
    //���
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            left[h] = points_l[j][0]+1;   //��ȡ�����ڱ߽��ߵ��ڲ�

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

    }
    h = image_h - 2;
    //�ұ�
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            right[h] = points_r[j][0] - 1;

        }
        else continue;//ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
        if (h == 0)break;//�����һ���˳�
    }
}
