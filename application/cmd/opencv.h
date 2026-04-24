#pragma once

#include <stdint.h>
#include "usart.h"
#include "robot_def.h"

extern float opencv_Vision_data[12];
extern uint8_t Vison_revc_flag;
extern float Vison_Frequency;



#ifdef VISION_USE_VCP
void Vision_USB_Init(void);
#endif

#ifdef VISION_USE_UART
void Vision_Uart_Init(UART_HandleTypeDef *_handle);
#endif
