#include "i2c_slave.h"

volatile unsigned char edata i2c_regs[I2C_REG_SIZE];
volatile unsigned char edata i2c_dirty_mask = 0U;

static volatile unsigned char edata i2c_isda = 0U;
static volatile unsigned char edata i2c_isma = 0U;
static volatile unsigned char edata i2c_reg_addr = 0U;
static volatile unsigned char edata i2c_pending_dirty = 0U;

void i2c_slave_init(void)
{
    unsigned char i;
    for (i = 0U; i < I2C_REG_SIZE; i++)
    {
        i2c_regs[i] = 0U;
    }
    i2c_dirty_mask = 0U;
    i2c_pending_dirty = 0U;
    i2c_isda = 1U;
    i2c_isma = 1U;
    i2c_reg_addr = 0U;

    /* Select P2.4=SCL, P2.3=SDA (I2C_S1=0, I2C_S0=0) */
    P_SW2 = (unsigned char)(P_SW2 & ~0x30U);

    I2CCFG = 0x81U;
    I2CSLADR = (unsigned char)((I2C_SLAVE_ADDR << 1) & 0xFEU);
    I2CSLST = 0x00U;
    I2CTXD = i2c_regs[0];
    I2CSLCR = 0x78U;
}

void i2c_isr(void) interrupt 24
{
    unsigned char st;

    st = I2CSLST;
    if ((st & 0x40U) != 0U)
    {
        I2CSLST = (unsigned char)(st & ~0x40U);
        if (i2c_pending_dirty != 0U)
        {
            i2c_dirty_mask |= i2c_pending_dirty;
            i2c_pending_dirty = 0U;
        }
        i2c_isda = 1U;
    }
    else if ((st & 0x20U) != 0U)
    {
        I2CSLST = (unsigned char)(st & ~0x20U);
        if (i2c_isda != 0U)
        {
            i2c_isda = 0U;
        }
        else if (i2c_isma != 0U)
        {
            i2c_isma = 0U;
            i2c_reg_addr = (unsigned char)(I2CRXD & (I2C_REG_SIZE - 1U));
            I2CTXD = i2c_regs[i2c_reg_addr];
        }
        else
        {
            i2c_regs[i2c_reg_addr] = I2CRXD;
            if (i2c_reg_addr == REG_G_CTRL)
            {
                i2c_pending_dirty |= I2C_DIRTY_CTRL;
            }
            else if (i2c_reg_addr >= REG_TARGET_BASE && i2c_reg_addr < REG_MODE_BASE)
            {
                i2c_pending_dirty |= I2C_DIRTY_TARGET;
            }
            else if (i2c_reg_addr >= REG_MODE_BASE && i2c_reg_addr < REG_POS_BASE)
            {
                i2c_pending_dirty |= I2C_DIRTY_MODE;
            }
            else if (i2c_reg_addr >= REG_KP_LO && i2c_reg_addr <= (REG_KD_LO + 1U))
            {
                i2c_pending_dirty |= I2C_DIRTY_PID;
            }
            i2c_reg_addr = (unsigned char)((i2c_reg_addr + 1U) & (I2C_REG_SIZE - 1U));
        }
    }
    else if ((st & 0x10U) != 0U)
    {
        I2CSLST = (unsigned char)(st & ~0x10U);
        if ((st & 0x02U) != 0U)
        {
            I2CTXD = 0xFFU;
        }
        else
        {
            i2c_reg_addr = (unsigned char)((i2c_reg_addr + 1U) & (I2C_REG_SIZE - 1U));
            I2CTXD = i2c_regs[i2c_reg_addr];
        }
    }
    else if ((st & 0x08U) != 0U)
    {
        I2CSLST = (unsigned char)(st & ~0x08U);
        if (i2c_pending_dirty != 0U)
        {
            i2c_dirty_mask |= i2c_pending_dirty;
            i2c_pending_dirty = 0U;
        }
        i2c_isda = 1U;
        i2c_isma = 1U;
    }
    else
    {
        I2CSLST = 0x00U;
    }
}
