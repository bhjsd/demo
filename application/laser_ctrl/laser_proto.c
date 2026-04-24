#include "laser_proto.h"

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define LASER_PROTO_FRAME_MAX_LEN 192u

static LaserProtoStats_t g_stats;
static LaserProtoPointCallback_t g_point_cb = NULL;

static char g_frame_buf[LASER_PROTO_FRAME_MAX_LEN + 1u];
static uint16_t g_frame_len = 0u;
static bool g_frame_collecting = false;

static bool LaserProto_ParsePointToken(const char *token, LaserPointCmd_t *out_point)
{
    char *endptr = NULL;
    long board_id = strtol(token, &endptr, 10);
    if (endptr == token || *endptr != ',' || board_id < 0 || board_id > 3)
    {
        return false;
    }

    char *x_start = endptr + 1;
    float x_rel = strtof(x_start, &endptr);
    if (endptr == x_start || *endptr != ',')
    {
        return false;
    }

    char *y_start = endptr + 1;
    float y_rel = strtof(y_start, &endptr);
    if (endptr == y_start || *endptr != '\0')
    {
        return false;
    }

    if (x_rel < -1000.0f || x_rel > 1000.0f || y_rel < -1000.0f || y_rel > 1000.0f)
    {
        return false;
    }

    out_point->board_id = (uint8_t)board_id;
    out_point->x_rel = x_rel;
    out_point->y_rel = y_rel;
    return true;
}

static void LaserProto_ProcessFrame(void)
{
    if (g_frame_len == 0u)
    {
        g_stats.frame_bad++;
        return;
    }

    g_frame_buf[g_frame_len] = '\0';

    bool frame_valid = true;
    bool has_point = false;
    char *save_ptr = NULL;
    char *token = strtok_r(g_frame_buf, ";", &save_ptr);

    while (token != NULL)
    {
        if (*token == '\0')
        {
            frame_valid = false;
            break;
        }

        LaserPointCmd_t point;
        if (!LaserProto_ParsePointToken(token, &point))
        {
            frame_valid = false;
            break;
        }

        has_point = true;
        g_stats.point_ok++;
        if (g_point_cb != NULL)
        {
            g_point_cb(&point);
        }

        token = strtok_r(NULL, ";", &save_ptr);
    }

    if (frame_valid && has_point)
    {
        g_stats.frame_ok++;
    }
    else
    {
        g_stats.frame_bad++;
    }
}

void LaserProto_Init(void)
{
    memset(&g_stats, 0, sizeof(g_stats));
    g_point_cb = NULL;
    g_frame_len = 0u;
    g_frame_collecting = false;
}

void LaserProto_OnUsbRx(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0u)
    {
        return;
    }

    g_stats.rx_bytes += (uint32_t)len;

    for (uint16_t i = 0u; i < len; i++)
    {
        char ch = (char)data[i];

        if (ch == '<')
        {
            g_frame_collecting = true;
            g_frame_len = 0u;
            continue;
        }

        if (!g_frame_collecting)
        {
            continue;
        }

        if (ch == '>')
        {
            LaserProto_ProcessFrame();
            g_frame_collecting = false;
            g_frame_len = 0u;
            continue;
        }

        if (g_frame_len >= LASER_PROTO_FRAME_MAX_LEN)
        {
            g_stats.overflow_bytes++;
            g_frame_collecting = false;
            g_frame_len = 0u;
            continue;
        }

        g_frame_buf[g_frame_len++] = ch;
    }
}

void LaserProto_SetPointCallback(LaserProtoPointCallback_t cb)
{
    g_point_cb = cb;
}

LaserProtoStats_t LaserProto_GetStats(void)
{
    return g_stats;
}
