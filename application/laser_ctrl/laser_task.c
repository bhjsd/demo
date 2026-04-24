#include "laser_task.h"
#include "cmsis_os2.h"
#include "galvanometer.h"
#include "laser.h"
#include "laser_proto.h"

#define LASER_IDLE_RELEASE_TICKS 5u

static osMessageQueueId_t g_laser_queue = NULL;
static uint32_t g_laser_queue_drop_count = 0u;
static uint32_t g_laser_idle_ticks = 0u;

static void LaserCtrl_OnParsedPoint(const LaserPointCmd_t *point)
{
    if (point == NULL || g_laser_queue == NULL)
    {
        return;
    }

    osStatus_t status = osMessageQueuePut(g_laser_queue, point, 0u, 0u);
    if (status == osOK)
    {
        return;
    }

    LaserPointCmd_t throwaway;
    if (osMessageQueueGet(g_laser_queue, &throwaway, NULL, 0u) == osOK)
    {
        (void)osMessageQueuePut(g_laser_queue, point, 0u, 0u);
        g_laser_queue_drop_count++;
    }
}

void LaserCtrl_Init(void)
{
    g_laser_queue_drop_count = 0u;
    g_laser_idle_ticks = 0u;
    if (g_laser_queue == NULL)
    {
        g_laser_queue = osMessageQueueNew(32u, sizeof(LaserPointCmd_t), NULL);
    }

    LaserProto_Init();
    LaserProto_SetPointCallback(LaserCtrl_OnParsedPoint);
    Galvo_Init();
    Laser_Init();
}

void LaserCtrl_Task(void)
{
    if (g_laser_queue == NULL)
    {
        return;
    }

    LaserPointCmd_t point;
    if (osMessageQueueGet(g_laser_queue, &point, NULL, 1u) == osOK)
    {
        // 使用现有接口先打通“协议->队列->振镜发送”链路。
        Galvo_Send_XY_DMA(point.x_rel, point.y_rel);
        Laser_SetTargetState(LASER_STATE_EMIT);
        g_laser_idle_ticks = 0u;
    }
    else
    {
        if (g_laser_idle_ticks < LASER_IDLE_RELEASE_TICKS)
        {
            g_laser_idle_ticks++;
        }

        if (g_laser_idle_ticks >= LASER_IDLE_RELEASE_TICKS)
        {
            Laser_SetTargetState(LASER_STATE_IDLE);
        }
    }

    Laser_Tick();
}

void LaserCtrl_OnUsbRx(const uint8_t *data, uint16_t len)
{
    LaserProto_OnUsbRx(data, len);
}

void LaserCtrl_OnDmaCompleteIRQ(void)
{
    Galvo_DMA_Complete_Callback();
}

uint32_t LaserCtrl_GetQueueDropCount(void)
{
    return g_laser_queue_drop_count;
}

