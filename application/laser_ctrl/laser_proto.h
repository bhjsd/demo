#ifndef __LASER_PROTO_H
#define __LASER_PROTO_H

#include <stdint.h>

typedef struct
{
	uint8_t board_id;
	float x_rel;
	float y_rel;
} LaserPointCmd_t;

typedef struct
{
	uint32_t rx_bytes;
	uint32_t frame_ok;
	uint32_t frame_bad;
	uint32_t point_ok;
	uint32_t overflow_bytes;
} LaserProtoStats_t;

typedef void (*LaserProtoPointCallback_t)(const LaserPointCmd_t *point);

void LaserProto_Init(void);
void LaserProto_OnUsbRx(const uint8_t *data, uint16_t len);
void LaserProto_SetPointCallback(LaserProtoPointCallback_t cb);
LaserProtoStats_t LaserProto_GetStats(void);

#endif // __LASER_PROTO_H
