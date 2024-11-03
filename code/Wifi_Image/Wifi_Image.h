/*
 * Wifi_Image.h
 *
 *  Created on: 2024Äê10ÔÂ26ÈÕ
 *      Author: sun
 */

#ifndef CODE_WIFI_IMAGE_WIFI_IMAGE_H_
#define CODE_WIFI_IMAGE_WIFI_IMAGE_H_

#include "zf_device_wifi_spi.h"
#include "zf_device_ips200.h"
#include "zf_driver_delay.h"
#include "zf_device_Key.h"
#include "stdbool.h"
#include "seekfree_assistant_interface.h"
#include "BEEP.h"
#include "MYCAMERA.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_camera.h"
#include "seekfree_assistant.h"

extern uint8 g_wifi_image_open_flag;
extern uint8 image_copy[MT9V03X_H][MT9V03X_W];

#define WIFI_SSID_TEST              "MiSaKaWIFI"
#define WIFI_PASSWORD_TEST          "123456789"

void Wifi_Image_Init(void);
void Wifi_Image_Send_Camera(void);

#endif /* CODE_WIFI_IMAGE_WIFI_IMAGE_H_ */
