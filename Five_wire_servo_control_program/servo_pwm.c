#include "servo_pwm.h"

/* Hot PWM state in edata for 20kHz Timer0 ISR */
static volatile signed char edata pwm_duty[SERVO_CH_COUNT];
static volatile unsigned char edata pwm_phase = 0U;
static volatile unsigned char edata g_led_status = 0U;

void servo_pwm_init(void)
{
    unsigned char i;
    for (i = 0U; i < SERVO_CH_COUNT; i++)
    {
        pwm_duty[i] = 0;
    }
    pwm_phase = 0U;
    g_led_status = 0U;

    /* Timer0 1T 16-bit auto-reload at 20kHz (40MHz / 2000 = 20kHz -> 200Hz 100-step PWM) */
    TR0 = 0;
    AUXR |= 0x80U;
    TMOD = (unsigned char)(TMOD & 0xF0U);
    TH0 = 0xF8U;
    TL0 = 0x30U;
    TF0 = 0;
    PT0 = 1;
    ET0 = 1;
    TR0 = 1;
}

void servo_pwm_set_channel(unsigned char ch, signed char duty)
{
    if (ch >= SERVO_CH_COUNT)
    {
        return;
    }
    if (duty != SERVO_PWM_BRAKE)
    {
        if (duty > SERVO_PWM_MAX_DUTY)
        {
            duty = SERVO_PWM_MAX_DUTY;
        }
        else if (duty < -SERVO_PWM_MAX_DUTY)
        {
            duty = -SERVO_PWM_MAX_DUTY;
        }
    }
    pwm_duty[ch] = duty;
}

void servo_pwm_stop_all(void)
{
    unsigned char i;
    for (i = 0U; i < SERVO_CH_COUNT; i++)
    {
        pwm_duty[i] = 0;
    }
}

void servo_pwm_set_led(unsigned char on)
{
    g_led_status = (on != 0U) ? 1U : 0U;
}

void timer0_isr(void) interrupt 1
{
    unsigned char phase;
    unsigned char p0_out;
    unsigned char p1_out;
    unsigned char p2_out;
    unsigned char p4_out;
    unsigned char p5_out;
    unsigned char a;
    unsigned char b;
    unsigned char ch;
    signed char d;
    unsigned char mag;

    phase = pwm_phase;
    pwm_phase = (unsigned char)(phase + 1U);
    if (pwm_phase >= 100U)
    {
        pwm_phase = 0U;
    }

    p0_out = 0U;
    p1_out = 0U;
    p2_out = 0U;
    p4_out = 0U;
    p5_out = 0U;

    for (ch = 0U; ch < SERVO_CH_COUNT; ch++)
    {
        d = pwm_duty[ch];
        if (d == SERVO_PWM_BRAKE)
        {
            a = 1U;
            b = 1U;
        }
        else if (d > 0)
        {
            mag = (unsigned char)d;
            a = (phase < mag) ? 1U : 0U;
            b = 0U;
        }
        else if (d < 0)
        {
            mag = (unsigned char)(-d);
            a = 0U;
            b = (phase < mag) ? 1U : 0U;
        }
        else
        {
            a = 0U;
            b = 0U;
        }

        switch (ch)
        {
            case 0U:
                if (a) { p1_out |= 0x20U; } /* P1.5 */
                if (b) { p1_out |= 0x40U; } /* P1.6 */
                break;
            case 1U:
                if (a) { p1_out |= 0x80U; } /* P1.7 */
                if (b) { p5_out |= 0x08U; } /* P5.3 */
                break;
            case 2U:
                if (a) { p5_out |= 0x04U; } /* P5.2 */
                if (b) { p4_out |= 0x04U; } /* P4.2 */
                break;
            case 3U:
                if (a) { p0_out |= 0x20U; } /* P0.5 */
                if (b) { p0_out |= 0x40U; } /* P0.6 */
                break;
            case 4U:
                if (a) { p0_out |= 0x80U; } /* P0.7 */
                if (b) { p4_out |= 0x40U; } /* P4.6 */
                break;
            case 5U:
                if (a) { p4_out |= 0x02U; } /* P4.1 */
                if (b) { p2_out |= 0x80U; } /* P2.7 */
                break;
            case 6U:
                if (a) { p2_out |= 0x40U; } /* P2.6 */
                if (b) { p2_out |= 0x20U; } /* P2.5 */
                break;
            case 7U:
                if (a) { p5_out |= 0x02U; } /* P5.1 */
                if (b) { p5_out |= 0x01U; } /* P5.0 */
                break;
            case 8U:
                if (a) { p2_out |= 0x04U; } /* P2.2 */
                if (b) { p2_out |= 0x02U; } /* P2.1 */
                break;
            case 9U:
                if (a) { p2_out |= 0x01U; } /* P2.0 */
                if (b) { p4_out |= 0x01U; } /* P4.0 */
                break;
            default:
                break;
        }
    }

    /* Direct latch writes: avoid RMW on P2 (keeps open-drain I2C P2.3/P2.4=1) and P5.7 LED */
    P0 = p0_out;
    P1 = p1_out;
    P2 = (unsigned char)(0x18U | p2_out);
    P4 = (unsigned char)(0xB8U | p4_out);
    P5 = (unsigned char)((g_led_status != 0U ? 0x80U : 0x00U) | p5_out);
}
