#ifndef _QST_SW_I2C_H_
#define _QST_SW_I2C_H_

#include "zf_driver_soft_iic.h"

#ifndef QST_SW_I2C_DEFAULT_SCL_PIN
#define QST_SW_I2C_DEFAULT_SCL_PIN    P33_8
#endif

#ifndef QST_SW_I2C_DEFAULT_SDA_PIN
#define QST_SW_I2C_DEFAULT_SDA_PIN    P32_4
#endif

#ifndef QST_SW_I2C_DEFAULT_DELAY
#define QST_SW_I2C_DEFAULT_DELAY      50
#endif

static soft_iic_info_struct qst_i2c_obj;

static inline void qst_sw_i2c_init(soft_iic_info_struct *i2c_obj,
                                   gpio_pin_enum scl_pin,
                                   gpio_pin_enum sda_pin,
                                   uint32 delay)
{
    soft_iic_init(i2c_obj, 0x00, delay, scl_pin, sda_pin);
}

static inline void qst_sw_i2c_init_default(void)
{
    qst_sw_i2c_init(&qst_i2c_obj,
                    QST_SW_I2C_DEFAULT_SCL_PIN,
                    QST_SW_I2C_DEFAULT_SDA_PIN,
                    QST_SW_I2C_DEFAULT_DELAY);
}

static inline uint8 qst_sw_probe(uint8 dev_addr)
{
    return soft_iic_probe_addr(&qst_i2c_obj, dev_addr);
}

static inline uint8 qst_sw_writereg(uint8 dev_addr, uint8 reg, uint8 val)
{
    qst_i2c_obj.addr = dev_addr;
    if(!qst_sw_probe(dev_addr))
    {
        return 1;
    }
    soft_iic_write_8bit_register(&qst_i2c_obj, reg, val);
    return 0;
}

static inline uint8 qst_sw_readreg(uint8 dev_addr, uint8 reg, uint8 *data, uint32 len)
{
    uint32 i = 0;
    qst_i2c_obj.addr = dev_addr;
    if(!qst_sw_probe(dev_addr))
    {
        for(i = 0; i < len; i++)
        {
            data[i] = 0xFF;
        }
        return 1;
    }
    soft_iic_read_8bit_registers(&qst_i2c_obj, reg, data, len);
    return 0;
}

#endif
