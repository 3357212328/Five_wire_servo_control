#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

#include <AI8051U.H>
#include <INTRINS.H>

/* System clocks */
#define SYS_FOSC_HZ         40000000UL
#define TFPU_PLL_HZ         120000000UL

/* 10-channel 5-wire servo count */
#define SERVO_CH_COUNT      10U

/* Motor port bitmasks (only drive MX1616H INA/INB pins, preserve ADC/I2C/LED) */
#define P0_MOTOR_MASK       0xE0U   /* P0.5, P0.6, P0.7 */
#define P1_MOTOR_MASK       0xE0U   /* P1.5, P1.6, P1.7 */
#define P2_MOTOR_MASK       0xE7U   /* P2.0, P2.1, P2.2, P2.5, P2.6, P2.7 (preserves P2.3/P2.4 I2C) */
#define P4_MOTOR_MASK       0x47U   /* P4.0, P4.1, P4.2, P4.6 */
#define P5_MOTOR_MASK       0x0FU   /* P5.0, P5.1, P5.2, P5.3 (preserves P5.7 LED_STATUS) */

/* Status LED (Active-High: P5.7 -> R10 -> LED2 -> GND) & Boot/Stop Key (Active-Low: P3.2) */
#define LED_STATUS_PIN      P57
#define KEY_BOOT_PIN        P32

/* Hardware I2C Slave 7-bit address (P2.4=SCL, P2.3=SDA) */
#define I2C_SLAVE_ADDR      0x40U

/* Servo channel operating modes */
#define SERVO_MODE_COAST    0U
#define SERVO_MODE_POS      1U
#define SERVO_MODE_SPEED    2U
#define SERVO_MODE_BRAKE    3U

#endif
