#include "sys_clock.h"
#include "set_io.h"

volatile unsigned char edata g_tick_1ms = 0U;
volatile unsigned long edata g_ms_counter = 0UL;

static void delay_100us(void)
{
    unsigned int i;
    for (i = 0U; i < 600U; i++)
    {
        _nop_();
    }
}

void sys_clock_init(void)
{
    /* Enable XFR access and zero wait-state memory timing */
    EAXFR = 1;
    WTST = 0x00U;
    CKCON = 0x00U;

    /* Configure 40MHz HIRC using factory trim values */
    CLKDIV = 0x04U;
    IRTRIM = CHIPID18;
    VRTRIM = CHIPID24;
    IRCBAND = (unsigned char)((IRCBAND & ~0x03U) | 0x03U);
    CLKDIV = 0x00U;

    /* Configure PLL to 120MHz for TFPU: 40MHz / 4 * 12 = 120MHz */
    CLKSEL |= 0x80U;
    USBCLK = (unsigned char)((USBCLK & ~0x60U) | 0x40U);
    USBCLK |= 0x80U;
    delay_100us();
    HSCLKDIV = 0x00U;
    TFPU_CLKDIV = 0x00U;
    CLKSEL |= 0x40U;
}

/* TFPU requires OPTIMIZE >= 1 so DMAIR writes compile to MOV DMAIR, #imm */
#pragma OPTIMIZE(3)
void sys_tfpu_init(void)
{
    DMAIR = 0x3FU;
    DMAIR = 0x31U;
    DMAIR = 0x32U;
}

void sys_gpio_init(void)
{
    /* Drive all 20 motor pins LOW before enabling push-pull outputs */
    P0 = (unsigned char)(P0 & ~P0_MOTOR_MASK);
    P1 = (unsigned char)(P1 & ~P1_MOTOR_MASK);
    P2 = (unsigned char)(P2 & ~P2_MOTOR_MASK);
    P4 = (unsigned char)(P4 & ~P4_MOTOR_MASK);
    P5 = (unsigned char)(P5 & ~P5_MOTOR_MASK);
    LED_STATUS_PIN = 0;

    set_io_mode(pp_mode,
                Pin05, Pin06, Pin07,
                Pin15, Pin16, Pin17,
                Pin20, Pin21, Pin22, Pin25, Pin26, Pin27,
                Pin_End);
    set_io_mode(pp_mode,
                Pin40, Pin41, Pin42, Pin46,
                Pin50, Pin51, Pin52, Pin53,
                Pin57,
                Pin_End);

    /* 10 potentiometer ADC pins: high-Z analog input with digital input disabled */
    set_io_mode(hz_mode,
                Pin00, Pin01, Pin02, Pin03, Pin04,
                Pin10, Pin11, Pin12, Pin13, Pin14,
                Pin_End);
    set_io_mode(dis_dinput,
                Pin00, Pin01, Pin02, Pin03, Pin04,
                Pin10, Pin11, Pin12, Pin13, Pin14,
                Pin_End);

    /* I2C pins P2.3(SDA) / P2.4(SCL): open-drain with internal pull-up */
    set_io_mode(od_mode, Pin23, Pin24, Pin_End);
    set_io_mode(en_pur, Pin23, Pin24, Pin_End);

    /* Boot / safe-stop button P3.2: quasi-bidirectional pull-up */
    set_io_mode(pu_mode, Pin32, Pin_End);
}

void sys_timer1_init(void)
{
    /* Timer1 1T 16-bit auto-reload at 1kHz (40MHz / 40000 = 1kHz; reload = 0x63C0) */
    TR1 = 0;
    AUXR |= 0x40U;
    TMOD = (unsigned char)(TMOD & 0x0FU);
    TH1 = 0x63U;
    TL1 = 0xC0U;
    TF1 = 0;
    ET1 = 1;
    TR1 = 1;
}

void timer1_isr(void) interrupt 3
{
    g_tick_1ms = 1U;
    g_ms_counter++;
}
