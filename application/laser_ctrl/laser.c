#include "laser.h"

static volatile LaserState_e g_laser_state = LASER_STATE_IDLE;
static volatile LaserState_e g_laser_target_state = LASER_STATE_IDLE;

void Laser_Init(void)
{
    g_laser_state = LASER_STATE_IDLE;
    g_laser_target_state = LASER_STATE_IDLE;
}

void Laser_Tick(void)
{
    if (g_laser_state == LASER_STATE_FAULT)
    {
        return;
    }

    if (g_laser_state == g_laser_target_state)
    {
        return;
    }

    switch (g_laser_state)
    {
    case LASER_STATE_IDLE:
        if (g_laser_target_state == LASER_STATE_ARMED || g_laser_target_state == LASER_STATE_EMIT)
        {
            g_laser_state = LASER_STATE_ARMED;
        }
        break;

    case LASER_STATE_ARMED:
        if (g_laser_target_state == LASER_STATE_IDLE)
        {
            g_laser_state = LASER_STATE_IDLE;
        }
        else if (g_laser_target_state == LASER_STATE_EMIT)
        {
            g_laser_state = LASER_STATE_EMIT;
        }
        break;

    case LASER_STATE_EMIT:
        if (g_laser_target_state != LASER_STATE_EMIT)
        {
            g_laser_state = LASER_STATE_ARMED;
        }
        break;

    case LASER_STATE_FAULT:
    default:
        break;
    }
}

void Laser_SetTargetState(LaserState_e target)
{
    if (target < LASER_STATE_IDLE || target > LASER_STATE_FAULT)
    {
        return;
    }

    if (target == LASER_STATE_FAULT)
    {
        Laser_EnterFault();
        return;
    }

    if (g_laser_state == LASER_STATE_FAULT)
    {
        if (target == LASER_STATE_IDLE)
        {
            g_laser_state = LASER_STATE_IDLE;
        }
        else
        {
            return;
        }
    }

    g_laser_target_state = target;
}

void Laser_EnterFault(void)
{
    g_laser_state = LASER_STATE_FAULT;
    g_laser_target_state = LASER_STATE_FAULT;
}

LaserState_e Laser_GetState(void)
{
    return g_laser_state;
}
