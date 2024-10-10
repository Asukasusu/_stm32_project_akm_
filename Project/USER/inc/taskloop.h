#ifndef  __TASKLOOP_H__
#define  __TASKLOOP_H__

#include <stdint.h>

extern volatile uint32_t sysTickUpTimes;

typedef void (*task_func)(void);

typedef struct
{
    task_func pfunc;
    uint16_t  interval_ticks;
    uint16_t  last_ticks;
}sched_task_t;

void TaskLoop_Run(void);

#endif
