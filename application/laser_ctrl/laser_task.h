#ifndef __LASER_TASK_H
#define __LASER_TASK_H

#include <stdint.h>

void LaserCtrl_Init(void);
void LaserCtrl_Task(void);
void LaserCtrl_OnUsbRx(const uint8_t *data, uint16_t len);
void LaserCtrl_OnDmaCompleteIRQ(void);

uint32_t LaserCtrl_GetQueueDropCount(void);

#endif // __LASER_TASK_H
