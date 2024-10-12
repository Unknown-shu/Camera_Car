#include "menu.h"
//----------------------------------------------------------------------------------------------------

struct Menu menu;

void Menu_init(void){
    menu_init(&menu, MAIN_PAGE, NULL_KEY_EVENT);
//    menu_para_init();
}
//----------------------------------------------------------------------------------------------------
/*�˵���ʼ��*/
void menu_init(struct Menu *handle, uint8_t Page, int EVENT_CODE)
{
    memset(handle, 0, sizeof(struct Menu));
    handle->Current_Page = Page  ;
    handle->KeyEvent = EVENT_CODE ;
}

/*�˵���ת*/
void Set_Menu(struct Menu *handle, uint8_t Page)
{
    handle->Current_Page= Page ;
}

/*��ȡ��ǰ�˵�*/
uint8_t Get_Menu(struct Menu *handle)
{
    return handle->Current_Page ;
}

/*���õ�ǰ�������¼�ֵ*/
void Set_Event_Code(struct Menu *handle, int Event_Code)
{
    handle->KeyEvent = Event_Code ;
}

/*��ȡ��ǰ�������¼�ֵ*/
int Get_Event_Code(struct Menu *handle)
{
    return handle->KeyEvent ;
}
/*�˵�ѡ��ѡ��ѭ��*/
void pagelimit(int8 *page_num,uint8 page_num_max)
{
    if(*page_num>page_num_max)*page_num=1;
    if(*page_num<=0)*page_num=page_num_max;
}
int Judgepress(struct Menu *handle)
{

    if(handle->KeyEvent != NULL_KEY_EVENT)
        return 1;
    else
        return 0;
}
//----------------------------------------------------------------------------------------------------
/*�˵�����*/
void Menu_Handler(struct Menu *handle)
{
    /*��ǰ�ǲ˵����ĸ�ҳ��*/

    switch(handle->Current_Page)
    {
        case MAIN_PAGE :
            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
            main_page_process(handle->KeyEvent);
            break ;
        case Start_PAGE :
            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
            start_page_process(handle->KeyEvent);
            break ;

        case NAVIGATION_PAGE :
            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
            navigation_page_process(handle->KeyEvent);
            break ;
        case Camera_PAGE :
            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
            Camera_page_process(handle->KeyEvent);
            break ;
//        case SERVO_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            servo_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_TURN_P_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            servo_turn_p_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_TURN_D_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            servo_turn_d_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_DUTY_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            servo_duty_page_process(handle->KeyEvent);
//            break ;
//        case START_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            start_page_process(handle->KeyEvent);
//            break ;
//        case GPS_MAP_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            gps_map_page_process(handle->KeyEvent);
//            break ;
//        case ELEMENT_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            element_page_process(handle->KeyEvent);
//            break ;
//        case SPEED_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            speed_page_process(handle->KeyEvent);
//            break ;
//        case START_ANGLE_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            start_angle_page_process(handle->KeyEvent);
//            break ;
//        case GPS_POINTS_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            points_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_TURN_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            servo_turn_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_STRAIGHT_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            servo_straight_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_STRAIGHT_P_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            servo_straight_p_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_STRAIGHT_D_PAGE :
//            /*���ע��ļ�ֵ����Ӧ�Ĵ���*/
//            servo_straight_d_page_process(handle->KeyEvent);
//            break ;
        default:
            break ;
    }

    /*��ʱ���¼��������ֹ�ظ�����*/

}

