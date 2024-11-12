/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC264 ��Դ���һ����
*
* TC264 ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          cpu0_main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.9.4
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "MYHEADFILE.h"

#pragma section all "cpu0_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��
// *************************** ����Ӳ������˵�� ***************************
// ���������Ҷ���������ͷ ��Ӧ��������ͷ�ӿ� ��ע������
//      ģ��ܽ�            ��Ƭ���ܽ�
//      TXD                 �鿴 zf_device_mt9v03x.h �� MT9V03X_COF_UART_TX �궨��
//      RXD                 �鿴 zf_device_mt9v03x.h �� MT9V03X_COF_UART_RX �궨��
//      PCLK                �鿴 zf_device_mt9v03x.h �� MT9V03X_PCLK_PIN �궨��
//      VSY                 �鿴 zf_device_mt9v03x.h �� MT9V03X_VSYNC_PIN �궨��
//      D0-D7               �鿴 zf_device_mt9v03x.h �� MT9V03X_DATA_PIN �궨�� �Ӹö��忪ʼ�������˸�����
//      GND                 ���İ��Դ�� GND
//      3V3                 ���İ� 3V3 ��Դ
// ����2��IPSģ��
// *************************** ����Ӳ������˵�� ***************************
//      ģ��ܽ�            ��Ƭ���ܽ�
//      BL                  �鿴 zf_device_ips200_parallel8.h �� IPS200_BL_PIN �궨��  Ĭ�� P15_3
//      CS                  �鿴 zf_device_ips200_parallel8.h �� IPS200_CS_PIN �궨��  Ĭ�� P15_5
//      RST                 �鿴 zf_device_ips200_parallel8.h �� IPS200_RST_PIN �궨�� Ĭ�� P15_1
//      RS                  �鿴 zf_device_ips200_parallel8.h �� IPS200_RS_PIN �궨��  Ĭ�� P15_0
//      WR                  �鿴 zf_device_ips200_parallel8.h �� IPS200_WR_PIN �궨��  Ĭ�� P15_2
//      RD                  �鿴 zf_device_ips200_parallel8.h �� IPS200_RD_PIN �궨��  Ĭ�� P15_4
//      D0-D7               �鿴 zf_device_ips200_parallel8.h �� IPS200_Dx_PIN �궨��  Ĭ�� P11_9/P11_10/P11_11/P11_12/P13_0/P13_1/P13_2/P13_3
//      GND                 ���İ��Դ�� GND
//      3V3                 ���İ� 3V3 ��Դ



// *************************** ���̲���˵�� ***************************
// 1.���İ���¼��ɱ����� �����İ���������� �嵽��
// 2.����ͷ�������������ͷ�ӿ� ע������2��IPSģ�����������Ļ�ӿ�
// 3.�����ϵ� ���ߺ��İ�������Ϻ��ϵ� ���İ尴�¸�λ����
// 4.��Ļ����ʾ��ʼ����ϢȻ����ʾ����ͷͼ��
// �������������˵�����ز��� ����ձ��ļ����·� ���̳�������˵�� �����Ų�
// **************************** �������� ****************************

#define IPS200_TYPE     (IPS200_TYPE_SPI)                                       // ˫������ ���������� ����궨����д IPS200_TYPE_PARALLEL8

                                                                          // �������� SPI ������ ����궨����д IPS200_TYPE_SPI
uint8_t Init_End_Flag;                  //��ʼ��������־��
//uint8 button1,button2,button3,button4;  //�ĸ�����



int core0_main(void)
{

    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������

    uint8 FPS;
    Encoder_Distance_Typedef Distance_Test_Structure;

    system_delay_init();            //�ӳٳ�ʼ��
    Menu_init();                    //�˵���ʼ��
    Beep_Init();                    //��������ʼ��
    ips200_init(IPS200_TYPE);       //��Ļ��ʼ��
    MyCamera_Init();                //����ͷ��ʼ��
    MyEncoder_Init();               //��������ʼ��
    Motor_Init();                   //�����ʼ��
    PID_param_init();               //PID������ʼ��
    key_init(20);                   //������ʼ��
    UART_Init();                    //���ڳ�ʼ��



    pit_ms_init(CCU60_CH0, 20);     //����ɨ���жϳ�ʼ��
    pit_ms_init(CCU60_CH1, 1000);     //����ɨ���жϳ�ʼ��
    pit_ms_init(CCU61_CH0, 5);     //pid�ж�


    Flash_Init();
    // �˴���д�û����� ���������ʼ�������
	cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����


	ips200_set_color(RGB565_WHITE, RGB565_BLACK);
	ips200_clear();

//	Wifi_Image_Init();

	Beep_MediumRing(); //�ϵ����
	Init_End_Flag = 1;


//	Gyroscope_Run();

    while (TRUE)
    {

        // �˴���д��Ҫѭ��ִ�еĴ���
        if(g_Car_Status == 0 || g_started_debug == 1)
            Menu_Handler(&menu);
        else if (Key_IfEnter())
        {
            Car_Stop();
        }
        if(key_get_state(KEY_6) == KEY_SHORT_PRESS)
        {
//            Beep_LongRing();
            Beep_ShortRing();
            Car_Start();
//            key_clear_all_state();
        }
//        Encoder_Distance_Start(&Distance_Test_Structure);
//        if(Encoder_Distance_MaxLimit(&Distance_Test_Structure, 20))
//        {
//            Beep_MediumRing();
//        }
//        Get_Encoder_Distance(&Distance_Test_Structure);
//        printf("%f\r\n", Distance_Test_Structure.AVG_distance);


//      FPS = 1000 / g_past_time ;
//        ips200_show_int(204,0 , FPS, 3);
        ips200_show_int(188, 0, camera_process_FPS, 5);
//        ips200_show_float(50, 18*5, rd_calculate, 4, 6);
//        ips200_show_int(188, 18, g_past_time, 5);
//        seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, middle[0], MT9V03X_W, MT9V03X_H);
//        seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, left[0], MT9V03X_W, MT9V03X_H);
//        seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, right[0], MT9V03X_W, MT9V03X_H);
//	    ips200_show_int(204,0 , enter_flag, 3);
//	    printf("%d ,%d ,%d ,%d, %d, %d\r\n ",target_left,target_right,Encoder_speed_l,Encoder_speed_r,pwm_left, pwm_right);
//
//	    printf("%d\r\n",V0);
        // �˴���д��Ҫѭ��ִ�еĴ���

//	    printf("%d\r\n",switch_encoder_num);
//	    uart_write_string(UART_2, "Test\r\n");
//        printf("%d\r\n",switch_encoder_change_num);
	}
}

#pragma section all restore
// **************************** �������� ****************************
// *************************** ���̳�������˵�� ***************************
// ��������ʱ�밴�������������б���
// ����1����Ļ����ʾ
//      ���ʹ��������ԣ��������Ҫ�õ�ع��� �����Ļ�������ŵ�ѹ
//      �����Ļ�ǲ��ǲ��λ���� ������Ŷ�Ӧ��ϵ
//      �����Ӧ���Ŷ���ȷ ���һ���Ƿ������Ų��β��� ��Ҫ��ʾ����
//      �޷���ɲ��β�������һ��GPIO���̽���Ļ����IO��ʼ��ΪGPIO��ת��ƽ �����Ƿ��ܿ�
// ����2����ʾ reinit ����
//      �������Ƿ�����
//      ���幩���Ƿ�ʹ�õ�������ĵ�ع���
// ����2����ʾͼ������ ��λ
//      �������ͷ�ź����Ƿ����ɶ�
