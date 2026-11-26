#ifndef __SERVO_PWM_H__
#define __SERVO_PWM_H__

#include "board_config.h"

/* Duty range: -100..+100 (%), 0 = coast, 127 = active brake */
#define SERVO_PWM_BRAKE     ((signed char)127)
#define SERVO_PWM_MAX_DUTY  ((signed char)100)

void servo_pwm_init(void);
void servo_pwm_set_channel(unsigned char ch, signed char duty);
void servo_pwm_stop_all(void);
void servo_pwm_set_led(unsigned char on);

#endif
