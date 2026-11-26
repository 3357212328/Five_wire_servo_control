#ifndef __SYS_CLOCK_H__
#define __SYS_CLOCK_H__

#include "board_config.h"

extern volatile unsigned char edata g_tick_1ms;
extern volatile unsigned long edata g_ms_counter;

void sys_clock_init(void);
void sys_tfpu_init(void);
void sys_gpio_init(void);
void sys_timer1_init(void);

#endif
