#include "opencv.h"
#include <stdint.h>
#include "bsp_usb.h"
#include "bsp_dwt.h"
#include <string.h>

#include "robot_def.h"

float opencv_Vision_data[12] = {0};
uint8_t Vison_revc_flag = 0;
float Vison_Frequency = 0.0f;

#ifdef VISION_USE_VCP

static uint8_t *vis_recv_buff = NULL;
static float last_vision_time = 0.0f;

#define VISION_PACKET_SIZE 51u
#define VISION_HEADER 0xA5u
#define VISION_FOOTER 0x5Au

static uint8_t rx_cache[256];
static uint16_t rx_cache_len = 0;

static void DecodeVisionVCP(uint16_t recv_len)
{
    float current_time = DWT_GetTimeline_s();

    if (vis_recv_buff == NULL || recv_len == 0u)
    {
        return;
    }

    // Keep the newest data if the cache is about to overflow.
    if (recv_len >= sizeof(rx_cache))
    {
        memcpy(rx_cache, vis_recv_buff + (recv_len - sizeof(rx_cache)), sizeof(rx_cache));
        rx_cache_len = sizeof(rx_cache);
    }
    else
    {
        uint16_t copy_len = recv_len;
        if ((uint32_t)rx_cache_len + copy_len > sizeof(rx_cache))
        {
            uint16_t shift = (uint16_t)((uint32_t)rx_cache_len + copy_len - sizeof(rx_cache));
            rx_cache_len = (uint16_t)(rx_cache_len - shift);
            memmove(rx_cache, rx_cache + shift, rx_cache_len);
        }

        memcpy(rx_cache + rx_cache_len, vis_recv_buff, copy_len);
        rx_cache_len = (uint16_t)(rx_cache_len + copy_len);
    }

    while (rx_cache_len >= VISION_PACKET_SIZE)
    {
        if (rx_cache[0] != VISION_HEADER)
        {
            rx_cache_len--;
            memmove(rx_cache, rx_cache + 1, rx_cache_len);
            continue;
        }

        if (rx_cache[VISION_PACKET_SIZE - 1u] != VISION_FOOTER)
        {
            rx_cache_len--;
            memmove(rx_cache, rx_cache + 1, rx_cache_len);
            continue;
        }

        uint8_t checksum = 0u;
        for (uint16_t i = 0; i < (VISION_PACKET_SIZE - 2u); i++)
        {
            checksum ^= rx_cache[i];
        }

        if (checksum == rx_cache[VISION_PACKET_SIZE - 2u])
        {
            memcpy(opencv_Vision_data, rx_cache + 1, sizeof(opencv_Vision_data));

            if (last_vision_time != 0.0f && (current_time - last_vision_time) > 0.0f)
            {
                Vison_Frequency = 1.0f / (current_time - last_vision_time);
            }
            last_vision_time = current_time;
            Vison_revc_flag = 1;

            rx_cache_len = (uint16_t)(rx_cache_len - VISION_PACKET_SIZE);
            memmove(rx_cache, rx_cache + VISION_PACKET_SIZE, rx_cache_len);
        }
        else
        {
            rx_cache_len--;
            memmove(rx_cache, rx_cache + 1, rx_cache_len);
        }
    }
}

void Vision_USB_Init(void)
{
    USB_Init_Config_s conf = {.rx_cbk = DecodeVisionVCP, .tx_cbk = NULL};
    vis_recv_buff = USBInit(conf);
}

#endif // VISION_USE_VCP

#ifdef VISION_USE_UART

#include "master_process.h"

void Vision_Uart_Init(UART_HandleTypeDef *_handle)
{
    (void)VisionInit(_handle);
}

#endif // VISION_USE_UART
