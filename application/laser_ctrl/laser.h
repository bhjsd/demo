#ifndef __LASER_H
#define __LASER_H

// 状态机四态
typedef enum {
    LASER_STATE_IDLE,    // 空闲且断光
    LASER_STATE_ARMED,   // 已锁定目标，准备出光（可能需要短延时）
    LASER_STATE_EMIT,    // 正在持续出光中
    LASER_STATE_FAULT    // 互锁信号异常、进入锁定必须手工复位
} LaserState_e;

// 硬件初始准备
void Laser_Init(void);

// 状态机流转Tick (在任务循环中调用)
void Laser_Tick(void);

// 外部请求切换状态
void Laser_SetTargetState(LaserState_e target);

// 查询当前状态
LaserState_e Laser_GetState(void);

// 主动触发进入硬件错误锁定（互锁生效）
void Laser_EnterFault(void);

#endif // __LASER_H
