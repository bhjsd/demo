#ifndef __GALVANOMETER_H
#define __GALVANOMETER_H

#include <stdint.h>

// 初始化DMA输出管脚
void Galvo_Init(void);

// 触发一次坐标输出 (支持REL浮点输入转换)
void Galvo_Send_XY_DMA(float x, float y);

// DMA 全完成中断时回调该函数，告知外界传输完成
void Galvo_DMA_Complete_Callback(void);

#endif // __GALVANOMETER_H
