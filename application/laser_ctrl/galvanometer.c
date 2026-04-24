#include <stdint.h>

#include "galvanometer.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include "tim.h"

extern DMA_HandleTypeDef hdma_tim7_up;

// XY2-100: 20 bit * 2 half-cycles + 1 tail = 41, 预留到 42
#define XY2_DMA_BUF_LEN 42u

#define PIN_SET(pin)   ((uint32_t)(pin))
#define PIN_RESET(pin) ((uint32_t)(pin) << 16u)

static uint32_t g_dma_buf[XY2_DMA_BUF_LEN];
static volatile uint8_t g_dma_busy = 0u;
static volatile uint8_t g_pending_valid = 0u;
static volatile float g_pending_x = 0.0f;
static volatile float g_pending_y = 0.0f;

static uint8_t Galvo_Parity(uint32_t v)
{
    v ^= v >> 16u;
    v ^= v >> 8u;
    v ^= v >> 4u;
    v ^= v >> 2u;
    v ^= v >> 1u;
    return (uint8_t)(v & 1u);
}

static uint16_t Galvo_CoordToRaw(float coord)
{
    if (coord < -1000.0f)
    {
        coord = -1000.0f;
    }
    if (coord > 1000.0f)
    {
        coord = 1000.0f;
    }

    // 线性映射: -1000 -> 0, +1000 -> 65535
    float normalized = (coord + 1000.0f) / 2000.0f;
    return (uint16_t)(normalized * 65535.0f + 0.5f);
}

static uint32_t Galvo_BuildFrame(uint16_t raw_data)
{
    uint32_t frame = 0u;
    frame |= (0x01u << 17u);            // 控制位 001
    frame |= ((uint32_t)raw_data << 1u);
    frame |= Galvo_Parity(frame >> 1u); // bit19..bit1 偶校验
    return frame;
}

static uint16_t Galvo_FillDmaBuf(uint32_t x_frame, uint32_t y_frame)
{
    uint16_t idx = 0u;

    for (int8_t bit = 19; bit >= 0; bit--)
    {
        uint32_t bsrr = PIN_RESET(GALVO_CLK_Pin) | PIN_RESET(GALVO_SYN_Pin);

        if ((x_frame >> bit) & 1u)
        {
            bsrr |= PIN_SET(GALVO_CHX_Pin);
        }
        else
        {
            bsrr |= PIN_RESET(GALVO_CHX_Pin);
        }

        if ((y_frame >> bit) & 1u)
        {
            bsrr |= PIN_SET(GALVO_CHY_Pin);
        }
        else
        {
            bsrr |= PIN_RESET(GALVO_CHY_Pin);
        }

        g_dma_buf[idx++] = bsrr;

        // 振镜在 CLK 上升沿采样
        bsrr &= ~PIN_RESET(GALVO_CLK_Pin);
        bsrr |= PIN_SET(GALVO_CLK_Pin);
        g_dma_buf[idx++] = bsrr;
    }

    // 帧间空闲态: SYNC=高, CLK=高, CHX/CHY=低
    g_dma_buf[idx++] = PIN_SET(GALVO_SYN_Pin) | PIN_SET(GALVO_CLK_Pin) |
                       PIN_RESET(GALVO_CHX_Pin) | PIN_RESET(GALVO_CHY_Pin);
    return idx;
}

static void Galvo_DmaCpltCallback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    Galvo_DMA_Complete_Callback();
}

static void Galvo_DmaErrorCallback(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    Galvo_DMA_Complete_Callback();
}

static HAL_StatusTypeDef Galvo_StartFrame(float x, float y)
{
    uint16_t x_raw = Galvo_CoordToRaw(x);
    uint16_t y_raw = Galvo_CoordToRaw(y);

    uint32_t x_frame = Galvo_BuildFrame(x_raw);
    uint32_t y_frame = Galvo_BuildFrame(y_raw);
    uint16_t count = Galvo_FillDmaBuf(x_frame, y_frame);

    HAL_TIM_Base_Stop(&htim7);
    __HAL_TIM_DISABLE_DMA(&htim7, TIM_DMA_UPDATE);
    (void)HAL_DMA_Abort(&hdma_tim7_up);

    hdma_tim7_up.XferCpltCallback = Galvo_DmaCpltCallback;
    hdma_tim7_up.XferErrorCallback = Galvo_DmaErrorCallback;

    if (HAL_DMA_Start_IT(&hdma_tim7_up,
                         (uint32_t)g_dma_buf,
                         (uint32_t)&(GPIOE->BSRR),
                         count) != HAL_OK)
    {
        return HAL_ERROR;
    }

    __HAL_TIM_ENABLE_DMA(&htim7, TIM_DMA_UPDATE);
    if (HAL_TIM_Base_Start(&htim7) != HAL_OK)
    {
        __HAL_TIM_DISABLE_DMA(&htim7, TIM_DMA_UPDATE);
        (void)HAL_DMA_Abort(&hdma_tim7_up);
        HAL_TIM_Base_Stop(&htim7);
        return HAL_ERROR;
    }

    return HAL_OK;
}

void Galvo_Init(void)
{
    g_dma_busy = 0u;
    g_pending_valid = 0u;

    // 空闲态: SYNC=高, CLK=高, CHX/CHY=低
    GPIOE->BSRR = PIN_SET(GALVO_SYN_Pin) | PIN_SET(GALVO_CLK_Pin) |
                  PIN_RESET(GALVO_CHX_Pin) | PIN_RESET(GALVO_CHY_Pin);
}

void Galvo_Send_XY_DMA(float x, float y)
{
    if (g_dma_busy != 0u)
    {
        // 忙时覆盖为最新点，避免持续丢失最新目标
        g_pending_x = x;
        g_pending_y = y;
        g_pending_valid = 1u;
        return;
    }

    g_dma_busy = 1u;
    if (Galvo_StartFrame(x, y) != HAL_OK)
    {
        g_dma_busy = 0u;
    }
}

void Galvo_DMA_Complete_Callback(void)
{
    HAL_TIM_Base_Stop(&htim7);
    __HAL_TIM_DISABLE_DMA(&htim7, TIM_DMA_UPDATE);
    g_dma_busy = 0u;

    if (g_pending_valid != 0u)
    {
        float pending_x = g_pending_x;
        float pending_y = g_pending_y;
        g_pending_valid = 0u;

        g_dma_busy = 1u;
        if (Galvo_StartFrame(pending_x, pending_y) != HAL_OK)
        {
            g_dma_busy = 0u;
        }
    }
}

