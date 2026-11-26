#ifndef __SERVO_CTRL_H__
#define __SERVO_CTRL_H__

#include "board_config.h"

void servo_ctrl_init(void);
void servo_ctrl_step_1ms(void);
void servo_ctrl_process_i2c(void);
void servo_ctrl_sync_telemetry(void);
void servo_ctrl_save_eeprom(void);
void servo_ctrl_set_global_enable(unsigned char en);
unsigned char servo_ctrl_get_global_enable(void);
unsigned char servo_ctrl_has_fault(void);

#endif
