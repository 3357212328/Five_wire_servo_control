#ifndef __I2C_SLAVE_H__
#define __I2C_SLAVE_H__

#include "board_config.h"

/* 128-byte I2C register map:
 * 0x00      : G_CTRL (bit0=en_all, bit1=save_eeprom, bit2=clear_faults)
 * 0x01      : G_STATUS (bit0=en_all, bit1=any_fault)
 * 0x02..0x15: TARGET[0..9] (int16 LE: 0..2700 = 0.0..270.0 deg in POS; -100..+100 in SPEED)
 * 0x16..0x1F: MODE[0..9] (0=coast, 1=pos, 2=speed, 3=brake)
 * 0x20..0x33: POS_NOW[0..9] (int16 LE, 0..2700 = 0.0..270.0 deg)
 * 0x34..0x47: ADC_NOW[0..9] (uint16 LE, 0..4095)
 * 0x48..0x51: DUTY_NOW[0..9] (int8)
 * 0x52..0x5B: FAULT[0..9] (bit0=stall)
 * 0x5C..0x61: KP_X100, KI_X100, KD_X100 (uint16 LE)
 */
#define I2C_REG_SIZE        128U
#define REG_G_CTRL          0x00U
#define REG_G_STATUS        0x01U
#define REG_TARGET_BASE     0x02U
#define REG_MODE_BASE       0x16U
#define REG_POS_BASE        0x20U
#define REG_ADC_BASE        0x34U
#define REG_DUTY_BASE       0x48U
#define REG_FAULT_BASE      0x52U
#define REG_KP_LO           0x5CU
#define REG_KI_LO           0x5EU
#define REG_KD_LO           0x60U

extern volatile unsigned char edata i2c_regs[I2C_REG_SIZE];
extern volatile unsigned char edata i2c_dirty_mask;

#define I2C_DIRTY_CTRL      0x01U
#define I2C_DIRTY_TARGET    0x02U
#define I2C_DIRTY_MODE      0x04U
#define I2C_DIRTY_PID       0x08U

void i2c_slave_init(void);

#endif
