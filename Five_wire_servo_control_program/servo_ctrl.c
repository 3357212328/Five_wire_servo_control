#include "servo_ctrl.h"
#include "servo_pwm.h"
#include "i2c_slave.h"
#include "set_adc.h"
#include "set_eeprom.h"

#define CFG_MAGIC           0xA55A3410UL
#define PID_DEADBAND_X10    15.0f       /* 1.5 deg deadband */
#define PID_INTEG_MAX       400.0f
#define STALL_DUTY_THRESH   45
#define STALL_ADC_DELTA     18U
#define STALL_TIME_MS       400U

typedef struct
{
    unsigned long magic;
    unsigned int kp_x100;
    unsigned int ki_x100;
    unsigned int kd_x100;
    unsigned int adc_min[SERVO_CH_COUNT];
    unsigned int adc_max[SERVO_CH_COUNT];
    unsigned char checksum;
} servo_cfg_t;

/* Cold configuration/calibration stored in xdata and synced with EEPROM */
static servo_cfg_t xdata servo_cfg;

/* Hot 1kHz closed-loop state stored in fast edata */
static unsigned int edata adc_filt[SERVO_CH_COUNT];
static signed int edata pos_deg_x10[SERVO_CH_COUNT];
static signed int edata target_val[SERVO_CH_COUNT];
static unsigned char edata ch_mode[SERVO_CH_COUNT];
static signed char edata duty_cmd[SERVO_CH_COUNT];
static unsigned char edata fault_flag[SERVO_CH_COUNT];
static float edata pid_err_prev[SERVO_CH_COUNT];
static float edata pid_integ[SERVO_CH_COUNT];
static unsigned int edata stall_cnt[SERVO_CH_COUNT];
static unsigned int edata stall_ref_adc[SERVO_CH_COUNT];
static unsigned char edata g_all_enable = 1U;

/* Channel -> ADC channel lookup table matching schematic U1 pinout */
static const adc_ch code pot_adc_map[SERVO_CH_COUNT] = {
    Adc4_P14, Adc3_P13, Adc2_P12, Adc1_P11, Adc0_P10,
    Adc8_P00, Adc9_P01, Adc10_P02, Adc11_P03, Adc12_P04
};

static unsigned char calc_cfg_checksum(void)
{
    unsigned char sum;
    unsigned int i;
    const unsigned char xdata *p;

    sum = 0U;
    p = (const unsigned char xdata *)&servo_cfg;
    for (i = 0U; i < (unsigned int)(sizeof(servo_cfg_t) - 1U); i++)
    {
        sum = (unsigned char)(sum + p[i]);
    }
    return sum;
}

static void load_default_cfg(void)
{
    unsigned char i;
    servo_cfg.magic = CFG_MAGIC;
    servo_cfg.kp_x100 = 35U;    /* Kp = 0.35 */
    servo_cfg.ki_x100 = 2U;     /* Ki = 0.02 */
    servo_cfg.kd_x100 = 8U;     /* Kd = 0.08 */
    for (i = 0U; i < SERVO_CH_COUNT; i++)
    {
        servo_cfg.adc_min[i] = 160U;
        servo_cfg.adc_max[i] = 3935U;
    }
    servo_cfg.checksum = calc_cfg_checksum();
}

static signed int adc_to_deg_x10_mdu(unsigned char ch, unsigned int adc)
{
    unsigned int amin;
    unsigned int amax;
    unsigned long span;
    unsigned long num;
    unsigned long deg;

    amin = servo_cfg.adc_min[ch];
    amax = servo_cfg.adc_max[ch];
    if (amax <= amin)
    {
        return 0;
    }
    if (adc <= amin)
    {
        return 0;
    }
    if (adc >= amax)
    {
        return 2700;
    }
    /* 32-bit MDU32-accelerated scaling: (adc - amin) * 2700 / (amax - amin) */
    span = (unsigned long)(amax - amin);
    num = (unsigned long)(adc - amin) * 2700UL;
    deg = num / span;
    if (deg > 2700UL)
    {
        deg = 2700UL;
    }
    return (signed int)deg;
}

void servo_ctrl_init(void)
{
    unsigned char i;

    load_default_cfg();
    IAP_TPS = 40U;
    set_eeprom_mode(Hex_Mode, (void *)&servo_cfg, (unsigned int)sizeof(servo_cfg_t));
    set_eeprom_sync(Pull);
    if (servo_cfg.magic != CFG_MAGIC || servo_cfg.checksum != calc_cfg_checksum())
    {
        load_default_cfg();
    }

    for (i = 0U; i < SERVO_CH_COUNT; i++)
    {
        adc_filt[i] = 2048U;
        pos_deg_x10[i] = 1350;
        target_val[i] = 1350;
        ch_mode[i] = SERVO_MODE_COAST;
        duty_cmd[i] = 0;
        fault_flag[i] = 0U;
        pid_err_prev[i] = 0.0f;
        pid_integ[i] = 0.0f;
        stall_cnt[i] = 0U;
        stall_ref_adc[i] = 2048U;
    }
    g_all_enable = 1U;

    /* Start cyclic 10-channel ADC sampling */
    set_adc_mode(cycl_mode,
                 Adc4_P14, Adc3_P13, Adc2_P12, Adc1_P11, Adc0_P10,
                 Adc8_P00, Adc9_P01, Adc10_P02, Adc11_P03, Adc12_P04,
                 Adc_End);

    /* Initialize I2C register map with PID gains and default targets */
    i2c_regs[REG_G_CTRL] = 0x01U;
    i2c_regs[REG_KP_LO] = (unsigned char)(servo_cfg.kp_x100 & 0xFFU);
    i2c_regs[REG_KP_LO + 1U] = (unsigned char)(servo_cfg.kp_x100 >> 8);
    i2c_regs[REG_KI_LO] = (unsigned char)(servo_cfg.ki_x100 & 0xFFU);
    i2c_regs[REG_KI_LO + 1U] = (unsigned char)(servo_cfg.ki_x100 >> 8);
    i2c_regs[REG_KD_LO] = (unsigned char)(servo_cfg.kd_x100 & 0xFFU);
    i2c_regs[REG_KD_LO + 1U] = (unsigned char)(servo_cfg.kd_x100 >> 8);
    for (i = 0U; i < SERVO_CH_COUNT; i++)
    {
        i2c_regs[REG_TARGET_BASE + (i * 2U)] = (unsigned char)((unsigned int)target_val[i] & 0xFFU);
        i2c_regs[REG_TARGET_BASE + (i * 2U) + 1U] = (unsigned char)(((unsigned int)target_val[i] >> 8) & 0xFFU);
        i2c_regs[REG_MODE_BASE + i] = ch_mode[i];
    }
    servo_ctrl_sync_telemetry();
    I2CTXD = i2c_regs[0];
}

void servo_ctrl_save_eeprom(void)
{
    servo_pwm_stop_all();
    servo_cfg.magic = CFG_MAGIC;
    servo_cfg.checksum = calc_cfg_checksum();
    EA = 0;
    set_eeprom_sync(Push);
    EA = 1;
}

void servo_ctrl_set_global_enable(unsigned char en)
{
    g_all_enable = (en != 0U) ? 1U : 0U;
    if (g_all_enable == 0U)
    {
        servo_pwm_stop_all();
    }
}

unsigned char servo_ctrl_get_global_enable(void)
{
    return g_all_enable;
}

unsigned char servo_ctrl_has_fault(void)
{
    unsigned char i;
    for (i = 0U; i < SERVO_CH_COUNT; i++)
    {
        if (fault_flag[i] != 0U)
        {
            return 1U;
        }
    }
    return 0U;
}

void servo_ctrl_process_i2c(void)
{
    unsigned char dirty;
    unsigned char i;
    unsigned char ctrl;
    signed int new_tgt;
    signed int diff;

    EA = 0;
    dirty = i2c_dirty_mask;
    i2c_dirty_mask = 0U;
    EA = 1;

    if (dirty == 0U)
    {
        return;
    }

    if ((dirty & I2C_DIRTY_CTRL) != 0U)
    {
        ctrl = i2c_regs[REG_G_CTRL];
        servo_ctrl_set_global_enable((unsigned char)(ctrl & 0x01U));
        if ((ctrl & 0x04U) != 0U)
        {
            for (i = 0U; i < SERVO_CH_COUNT; i++)
            {
                fault_flag[i] = 0U;
                stall_cnt[i] = 0U;
            }
            i2c_regs[REG_G_CTRL] = (unsigned char)(ctrl & ~0x04U);
        }
        if ((ctrl & 0x02U) != 0U)
        {
            servo_ctrl_save_eeprom();
            i2c_regs[REG_G_CTRL] = (unsigned char)(i2c_regs[REG_G_CTRL] & ~0x02U);
        }
    }

    if ((dirty & I2C_DIRTY_TARGET) != 0U)
    {
        for (i = 0U; i < SERVO_CH_COUNT; i++)
        {
            new_tgt = (signed int)((unsigned int)i2c_regs[REG_TARGET_BASE + (i * 2U)] |
                      ((unsigned int)i2c_regs[REG_TARGET_BASE + (i * 2U) + 1U] << 8));
            diff = new_tgt - target_val[i];
            if (diff < 0)
            {
                diff = -diff;
            }
            if (diff > 30)
            {
                fault_flag[i] = 0U;
                stall_cnt[i] = 0U;
            }
            target_val[i] = new_tgt;
        }
    }

    if ((dirty & I2C_DIRTY_MODE) != 0U)
    {
        for (i = 0U; i < SERVO_CH_COUNT; i++)
        {
            if (ch_mode[i] != i2c_regs[REG_MODE_BASE + i])
            {
                ch_mode[i] = i2c_regs[REG_MODE_BASE + i];
                pid_integ[i] = 0.0f;
                pid_err_prev[i] = 0.0f;
                fault_flag[i] = 0U;
                stall_cnt[i] = 0U;
            }
        }
    }

    if ((dirty & I2C_DIRTY_PID) != 0U)
    {
        servo_cfg.kp_x100 = (unsigned int)i2c_regs[REG_KP_LO] | ((unsigned int)i2c_regs[REG_KP_LO + 1U] << 8);
        servo_cfg.ki_x100 = (unsigned int)i2c_regs[REG_KI_LO] | ((unsigned int)i2c_regs[REG_KI_LO + 1U] << 8);
        servo_cfg.kd_x100 = (unsigned int)i2c_regs[REG_KD_LO] | ((unsigned int)i2c_regs[REG_KD_LO + 1U] << 8);
    }
}

void servo_ctrl_step_1ms(void)
{
    unsigned char ch;
    unsigned char adc_ready;
    unsigned int raw_adc;
    unsigned int filt;
    unsigned int delta_adc;
    signed int abs_duty;
    float kp;
    float ki;
    float kd;
    float err;
    float deriv;
    float out_f;
    signed char cmd;

    /* TFPU-accelerated PID gains */
    kp = (float)servo_cfg.kp_x100 * 0.01f;
    ki = (float)servo_cfg.ki_x100 * 0.01f;
    kd = (float)servo_cfg.kd_x100 * 0.01f;

    for (ch = 0U; ch < SERVO_CH_COUNT; ch++)
    {
        /* Atomically check & read 12-bit ADC sample vs adc_isr, then update 4-tap EMA */
        EADC = 0;
        adc_ready = (unsigned char)get_adc_state(pot_adc_map[ch]);
        if (adc_ready != 0U)
        {
            raw_adc = adc_value[pot_adc_map[ch]] & 0x0FFFU;
        }
        EADC = 1;
        if (adc_ready != 0U)
        {
            filt = (unsigned int)(((unsigned long)adc_filt[ch] * 3UL + (unsigned long)raw_adc) >> 2);
            adc_filt[ch] = filt;
        }
        else
        {
            filt = adc_filt[ch];
        }
        pos_deg_x10[ch] = adc_to_deg_x10_mdu(ch, filt);

        if (g_all_enable == 0U || fault_flag[ch] != 0U)
        {
            duty_cmd[ch] = 0;
            servo_pwm_set_channel(ch, 0);
            continue;
        }

        switch (ch_mode[ch])
        {
            case SERVO_MODE_POS:
                err = (float)(target_val[ch] - pos_deg_x10[ch]);
                if (err > -PID_DEADBAND_X10 && err < PID_DEADBAND_X10)
                {
                    cmd = 0;
                    pid_integ[ch] = 0.0f;
                    pid_err_prev[ch] = 0.0f;
                }
                else
                {
                    pid_integ[ch] += err * 0.001f;
                    if (pid_integ[ch] > PID_INTEG_MAX)
                    {
                        pid_integ[ch] = PID_INTEG_MAX;
                    }
                    else if (pid_integ[ch] < -PID_INTEG_MAX)
                    {
                        pid_integ[ch] = -PID_INTEG_MAX;
                    }
                    deriv = err - pid_err_prev[ch];
                    pid_err_prev[ch] = err;
                    out_f = kp * err + ki * pid_integ[ch] + kd * deriv;
                    if (out_f > 100.0f)
                    {
                        out_f = 100.0f;
                    }
                    else if (out_f < -100.0f)
                    {
                        out_f = -100.0f;
                    }
                    cmd = (signed char)out_f;
                }
                break;

            case SERVO_MODE_SPEED:
                if (target_val[ch] > 100)
                {
                    cmd = SERVO_PWM_MAX_DUTY;
                }
                else if (target_val[ch] < -100)
                {
                    cmd = -SERVO_PWM_MAX_DUTY;
                }
                else
                {
                    cmd = (signed char)target_val[ch];
                }
                break;

            case SERVO_MODE_BRAKE:
                cmd = SERVO_PWM_BRAKE;
                stall_cnt[ch] = 0U;
                break;

            case SERVO_MODE_COAST:
            default:
                cmd = 0;
                stall_cnt[ch] = 0U;
                break;
        }

        /* Stall protection for active drive modes */
        if (ch_mode[ch] == SERVO_MODE_POS || ch_mode[ch] == SERVO_MODE_SPEED)
        {
            abs_duty = (cmd < 0) ? (signed int)(-cmd) : (signed int)cmd;
            if (abs_duty >= STALL_DUTY_THRESH)
            {
                delta_adc = (filt >= stall_ref_adc[ch]) ? (filt - stall_ref_adc[ch]) : (stall_ref_adc[ch] - filt);
                if (delta_adc < STALL_ADC_DELTA)
                {
                    stall_cnt[ch]++;
                    if (stall_cnt[ch] >= STALL_TIME_MS)
                    {
                        fault_flag[ch] |= 0x01U;
                        cmd = 0;
                    }
                }
                else
                {
                    stall_ref_adc[ch] = filt;
                    stall_cnt[ch] = 0U;
                }
            }
            else
            {
                stall_ref_adc[ch] = filt;
                stall_cnt[ch] = 0U;
            }
        }

        duty_cmd[ch] = cmd;
        servo_pwm_set_channel(ch, cmd);
    }
}

void servo_ctrl_sync_telemetry(void)
{
    unsigned char ch;
    unsigned int pos_u;
    unsigned int adc_u;
    unsigned char status;

    status = (g_all_enable != 0U) ? 0x01U : 0x00U;
    if (servo_ctrl_has_fault() != 0U)
    {
        status |= 0x02U;
    }
    i2c_regs[REG_G_STATUS] = status;

    for (ch = 0U; ch < SERVO_CH_COUNT; ch++)
    {
        pos_u = (unsigned int)pos_deg_x10[ch];
        adc_u = adc_filt[ch];
        i2c_regs[REG_POS_BASE + (ch * 2U)] = (unsigned char)(pos_u & 0xFFU);
        i2c_regs[REG_POS_BASE + (ch * 2U) + 1U] = (unsigned char)(pos_u >> 8);
        i2c_regs[REG_ADC_BASE + (ch * 2U)] = (unsigned char)(adc_u & 0xFFU);
        i2c_regs[REG_ADC_BASE + (ch * 2U) + 1U] = (unsigned char)(adc_u >> 8);
        i2c_regs[REG_DUTY_BASE + ch] = (unsigned char)duty_cmd[ch];
        i2c_regs[REG_FAULT_BASE + ch] = fault_flag[ch];
    }
}
