#include "menu.h"
//----------------------------------------------------------------------------------------------------

struct Menu menu;

void Menu_init(void){
    menu_init(&menu, MAIN_PAGE, NULL_KEY_EVENT);
//    menu_para_init();
}
//----------------------------------------------------------------------------------------------------
/*菜单初始化*/
void menu_init(struct Menu *handle, uint8_t Page, int EVENT_CODE)
{
    memset(handle, 0, sizeof(struct Menu));
    handle->Current_Page = Page  ;
    handle->KeyEvent = EVENT_CODE ;
}

/*菜单跳转*/
void Set_Menu(struct Menu *handle, uint8_t Page)
{
    handle->Current_Page= Page ;
}

/*获取当前菜单*/
uint8_t Get_Menu(struct Menu *handle)
{
    return handle->Current_Page ;
}

/*设置当前发生的事件值*/
void Set_Event_Code(struct Menu *handle, int Event_Code)
{
    handle->KeyEvent = Event_Code ;
}

/*获取当前发生的事件值*/
int Get_Event_Code(struct Menu *handle)
{
    return handle->KeyEvent ;
}
/*菜单选项选中循环*/
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
/*菜单处理*/
void Menu_Handler(struct Menu *handle)
{
    /*当前是菜单的哪个页面*/

    switch(handle->Current_Page)
    {
        case MAIN_PAGE :
            /*针对注册的键值做相应的处理*/
            main_page_process(handle->KeyEvent);
            break ;
        case Start_PAGE :
            /*针对注册的键值做相应的处理*/
            start_page_process(handle->KeyEvent);
            break ;

        case NAVIGATION_PAGE :
            /*针对注册的键值做相应的处理*/
            navigation_page_process(handle->KeyEvent);
            break ;
        case Camera_PAGE :
            /*针对注册的键值做相应的处理*/
            Camera_page_process(handle->KeyEvent);
            break ;
//        case SERVO_PAGE :
//            /*针对注册的键值做相应的处理*/
//            servo_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_TURN_P_PAGE :
//            /*针对注册的键值做相应的处理*/
//            servo_turn_p_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_TURN_D_PAGE :
//            /*针对注册的键值做相应的处理*/
//            servo_turn_d_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_DUTY_PAGE :
//            /*针对注册的键值做相应的处理*/
//            servo_duty_page_process(handle->KeyEvent);
//            break ;
//        case START_PAGE :
//            /*针对注册的键值做相应的处理*/
//            start_page_process(handle->KeyEvent);
//            break ;
//        case GPS_MAP_PAGE :
//            /*针对注册的键值做相应的处理*/
//            gps_map_page_process(handle->KeyEvent);
//            break ;
//        case ELEMENT_PAGE :
//            /*针对注册的键值做相应的处理*/
//            element_page_process(handle->KeyEvent);
//            break ;
//        case SPEED_PAGE :
//            /*针对注册的键值做相应的处理*/
//            speed_page_process(handle->KeyEvent);
//            break ;
//        case START_ANGLE_PAGE :
//            /*针对注册的键值做相应的处理*/
//            start_angle_page_process(handle->KeyEvent);
//            break ;
//        case GPS_POINTS_PAGE :
//            /*针对注册的键值做相应的处理*/
//            points_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_TURN_PAGE :
//            /*针对注册的键值做相应的处理*/
//            servo_turn_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_STRAIGHT_PAGE :
//            /*针对注册的键值做相应的处理*/
//            servo_straight_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_STRAIGHT_P_PAGE :
//            /*针对注册的键值做相应的处理*/
//            servo_straight_p_page_process(handle->KeyEvent);
//            break ;
//        case SERVO_STRAIGHT_D_PAGE :
//            /*针对注册的键值做相应的处理*/
//            servo_straight_d_page_process(handle->KeyEvent);
//            break ;
        default:
            break ;
    }

    /*及时将事件清除，防止重复触发*/

}

