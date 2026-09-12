#include "xl9555.h"

static soft_iic_info_struct g_soft_iic_obj;

static uint8_t get_port_reg(uint8_t gpio, uint8_t base_reg)
{
    return (gpio < 8U) ? base_reg : (uint8_t)(base_reg + 1U);
}

static void read_reg_16(xl9555_dev_t *dev, uint8_t reg, uint16_t *val)
{
    uint8_t low_byte;
    uint8_t high_byte;

    low_byte = soft_iic_read_8bit_register(dev->soft_iic_obj, reg);
    high_byte = soft_iic_read_8bit_register(dev->soft_iic_obj, (uint8_t)(reg + 1U));
    *val = (uint16_t)(((uint16_t)high_byte << 8) | low_byte);
}

static void write_reg_16(xl9555_dev_t *dev, uint8_t reg, uint16_t val)
{
    uint8_t low_byte = (uint8_t)(val & 0xFFU);
    uint8_t high_byte = (uint8_t)((val >> 8) & 0xFFU);

    soft_iic_write_8bit_register(dev->soft_iic_obj, reg, low_byte);
    soft_iic_write_8bit_register(dev->soft_iic_obj, (uint8_t)(reg + 1U), high_byte);
}

static void read_reg_bit_8(xl9555_dev_t *dev, uint8_t reg, uint8_t *val, uint8_t bit)
{
    uint8_t reg_value = soft_iic_read_8bit_register(dev->soft_iic_obj, reg);
    *val = (uint8_t)((reg_value >> bit) & 0x01U);
}

static void write_reg_bit_8(xl9555_dev_t *dev, uint8_t reg, uint8_t val, uint8_t bit)
{
    uint8_t reg_value = soft_iic_read_8bit_register(dev->soft_iic_obj, reg);

    if (val)
    {
        reg_value = (uint8_t)(reg_value | (uint8_t)(1U << bit));
    }
    else
    {
        reg_value = (uint8_t)(reg_value & (uint8_t)(~(uint8_t)(1U << bit)));
    }

    soft_iic_write_8bit_register(dev->soft_iic_obj, reg, reg_value);
}

void xl9555_init_desc(xl9555_dev_t *dev, uint8 addr, uint32 delay, gpio_pin_enum scl_pin, gpio_pin_enum sda_pin)
{
    if (dev == NULL)
    {
        return;
    }

    if ((addr < XL9555_I2C_ADDRESS_BASE) || (addr > XL9555_I2C_ADDRESS_MAX))
    {
        return;
    }

    soft_iic_init(&g_soft_iic_obj, addr, delay, scl_pin, sda_pin);
    dev->soft_iic_obj = &g_soft_iic_obj;
    dev->addr = addr;
}

void xl9555_free_desc(xl9555_dev_t *dev)
{
    (void)dev;
}

void xl9555_get_full_gpio_level(xl9555_dev_t *dev, uint16 *levels)
{
    if ((dev == NULL) || (levels == NULL))
    {
        return;
    }

    read_reg_16(dev, XL9555_REG_INPUT_PORT0, levels);
}

void xl9555_get_gpio_level(xl9555_dev_t *dev, uint8 gpio, uint8 *level)
{
    uint8_t reg;

    if ((dev == NULL) || (level == NULL) || (gpio > 15U))
    {
        return;
    }

    reg = get_port_reg(gpio, XL9555_REG_INPUT_PORT0);
    read_reg_bit_8(dev, reg, level, (uint8_t)(gpio % 8U));
}

void xl9555_set_full_gpio_level(xl9555_dev_t *dev, uint16 levels)
{
    if (dev == NULL)
    {
        return;
    }

    write_reg_16(dev, XL9555_REG_OUTPUT_PORT0, levels);
}

void xl9555_set_gpio_level(xl9555_dev_t *dev, uint8 gpio, uint8 level)
{
    uint8_t reg;

    if ((dev == NULL) || (gpio > 15U))
    {
        return;
    }

    reg = get_port_reg(gpio, XL9555_REG_OUTPUT_PORT0);
    write_reg_bit_8(dev, reg, level, (uint8_t)(gpio % 8U));
}

void xl9555_get_full_gpio_polarity(xl9555_dev_t *dev, uint16 *polarity)
{
    if ((dev == NULL) || (polarity == NULL))
    {
        return;
    }

    read_reg_16(dev, XL9555_REG_POLARITY_PORT0, polarity);
}

void xl9555_get_gpio_polarity(xl9555_dev_t *dev, uint8 gpio, xl9555_polarity_t *polarity)
{
    uint8_t reg;
    uint8_t bit_value;

    if ((dev == NULL) || (polarity == NULL) || (gpio > 15U))
    {
        return;
    }

    reg = get_port_reg(gpio, XL9555_REG_POLARITY_PORT0);
    read_reg_bit_8(dev, reg, &bit_value, (uint8_t)(gpio % 8U));
    *polarity = bit_value ? XL9555_POLARITY_INVERTED : XL9555_POLARITY_NOT_INVERTED;
}

void xl9555_set_full_gpio_polarity(xl9555_dev_t *dev, uint16 polarity)
{
    if (dev == NULL)
    {
        return;
    }

    write_reg_16(dev, XL9555_REG_POLARITY_PORT0, polarity);
}

void xl9555_set_gpio_polarity(xl9555_dev_t *dev, uint8 gpio, xl9555_polarity_t polarity)
{
    uint8_t reg;
    uint8_t bit_value;

    if ((dev == NULL) || (gpio > 15U))
    {
        return;
    }

    reg = get_port_reg(gpio, XL9555_REG_POLARITY_PORT0);
    bit_value = (polarity == XL9555_POLARITY_INVERTED) ? 1U : 0U;
    write_reg_bit_8(dev, reg, bit_value, (uint8_t)(gpio % 8U));
}

void xl9555_get_full_gpio_mode(xl9555_dev_t *dev, uint16 *mode)
{
    if ((dev == NULL) || (mode == NULL))
    {
        return;
    }

    read_reg_16(dev, XL9555_REG_MODE_PORT0, mode);
}

void xl9555_get_gpio_mode(xl9555_dev_t *dev, uint8 gpio, xl9555_gpio_mode_t *mode)
{
    uint8_t reg;
    uint8_t bit_value;

    if ((dev == NULL) || (mode == NULL) || (gpio > 15U))
    {
        return;
    }

    reg = get_port_reg(gpio, XL9555_REG_MODE_PORT0);
    read_reg_bit_8(dev, reg, &bit_value, (uint8_t)(gpio % 8U));
    *mode = bit_value ? XL9555_GPIO_INPUT : XL9555_GPIO_OUTPUT;
}

void xl9555_set_full_gpio_mode(xl9555_dev_t *dev, uint16 mode)
{
    if (dev == NULL)
    {
        return;
    }

    write_reg_16(dev, XL9555_REG_MODE_PORT0, mode);
}

void xl9555_set_gpio_mode(xl9555_dev_t *dev, uint8 gpio, xl9555_gpio_mode_t mode)
{
    uint8_t reg;
    uint8_t bit_value;

    if ((dev == NULL) || (gpio > 15U))
    {
        return;
    }

    reg = get_port_reg(gpio, XL9555_REG_MODE_PORT0);
    bit_value = (mode == XL9555_GPIO_INPUT) ? 1U : 0U;
    write_reg_bit_8(dev, reg, bit_value, (uint8_t)(gpio % 8U));
}
//dy ahstu zhugeliang ltl
