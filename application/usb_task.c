/*
 * @Author: your name
 * @Date: 2021-03-03 16:45:59
 * @LastEditTime: 2021-04-08 12:41:11
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMc:\Users\42517\Documents\GitHub\RM-sentinel\application\usb_task.c
 */
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb??????????Ϣ
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "detect_task.h"
#include "voltage_task.h"

#include "stm32f4xx_it.h"



void usb_printf(const char *fmt,...);

static uint8_t usb_buf[256];
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_usb_local;


void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    error_list_usb_local = get_error_list_point();
static int i;
	i=(int)HAL_GPIO_ReadPin(GPIOI,7);

    while(1)
    {
        osDelay(1000);
//         usb_printf(
// "******************************\r\n\
// voltage percentage:%d%% \r\n\s
// DBUS:%s\r\n\
// chassis motor1:%s\r\n\
// chassis motor2:%s\r\n\
// chassis motor3:%s\r\n\
// chassis motor4:%s\r\n\
// yaw motor:%s\r\n\
// pitch motor:%s\r\n\
// trigger motor:%s\r\n\
// gyro sensor:%s\r\n\
// accel sensor:%s\r\n\
// mag sensor:%s\r\n\
// referee usart:%s\r\n\
// dis_to_right:%hu\r\n\
// dis_to_left:%hu\r\n\
// flag:%hu\r\n\
// GPIO_INPUT:%d\r\n\
// ******************************\r\n",
//             get_battery_percentage(), 
//             status[error_list_usb_local[DBUS_TOE].error_exist],
//             status[error_list_usb_local[CHASSIS_MOTOR1_TOE].error_exist],
//             status[error_list_usb_local[CHASSIS_MOTOR2_TOE].error_exist],
//             status[error_list_usb_local[CHASSIS_MOTOR3_TOE].error_exist],
//             status[error_list_usb_local[CHASSIS_MOTOR4_TOE].error_exist],
//             status[error_list_usb_local[YAW_GIMBAL_MOTOR_TOE].error_exist],
//             status[error_list_usb_local[PITCH_GIMBAL_MOTOR_TOE].error_exist],
//             status[error_list_usb_local[TRIGGER_MOTOR_TOE].error_exist],
//             status[error_list_usb_local[BOARD_GYRO_TOE].error_exist],
//             status[error_list_usb_local[BOARD_ACCEL_TOE].error_exist],
//             status[error_list_usb_local[BOARD_MAG_TOE].error_exist],
//             status[error_list_usb_local[REFEREE_TOE].error_exist],
// 						distance_to_right,
// 						distance_to_left,
// 						flag,
//             i
// 						);

    }

}
void usb_printf(const char *fmt,...)
{	
    va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);

    CDC_Transmit_FS(usb_buf, len);
}
