#include "board_config.h"
#include "sys_clock.h"
#include "servo_pwm.h"
#include "servo_ctrl.h"
#include "i2c_slave.h"

void main(void)
{
    unsigned int led_div;
    unsigned char led_state;
    unsigned char key_deb;
    unsigned char key_latched;
    unsigned int period_ms;

    sys_clock_init();
    sys_tfpu_init();
    sys_gpio_init();
    servo_pwm_init();
    i2c_slave_init();
    servo_ctrl_init();
    sys_timer1_init();

    EA = 1;

    led_div = 0U;
    led_state = 0U;
    key_deb = 0U;
    key_latched = 0U;

    while (1)
    {
        servo_ctrl_process_i2c();

        if (g_tick_1ms != 0U)
        {
            g_tick_1ms = 0U;

            servo_ctrl_step_1ms();
            servo_ctrl_sync_telemetry();

            /* Debounced active-low KEY_BOOT (P3.2) emergency stop/resume toggle */
            if (KEY_BOOT_PIN == 0)
            {
                if (key_deb < 30U)
                {
                    key_deb++;
                }
                else if (key_latched == 0U)
                {
                    key_latched = 1U;
                    servo_ctrl_set_global_enable((unsigned char)(servo_ctrl_get_global_enable() == 0U ? 1U : 0U));
                }
            }
            else
            {
                key_deb = 0U;
                key_latched = 0U;
            }

            /* Heartbeat LED_STATUS (P5.7, active-high): 1Hz normal, 5Hz on fault */
            period_ms = (servo_ctrl_has_fault() != 0U) ? 100U : 500U;
            led_div++;
            if (led_div >= period_ms)
            {
                led_div = 0U;
                led_state = (unsigned char)(led_state == 0U ? 1U : 0U);
                servo_pwm_set_led(led_state);
            }
        }
    }
}
