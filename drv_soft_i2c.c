#include <rtthread.h>
#include <rtdevice.h>
#include "gd32f20x.h"
#include "drv_soft_i2c_config.h"

#ifdef RT_USING_I2C_BITOPS

struct gd32_soft_i2c_config
{
    rt_uint8_t scl;
    rt_uint8_t sda;
    const char *bus_name;
};

struct gd32_i2c
{
    struct rt_i2c_bit_ops ops;
    struct rt_i2c_bus_device i2c2_bus;
};

static const struct gd32_soft_i2c_config soft_i2c_config[] = DRV_SOFT_I2C_TABLE;

static struct gd32_i2c i2c_obj[sizeof(soft_i2c_config) / sizeof(soft_i2c_config[0])] = {0};

static void gd32_i2c_gpio_init(struct gd32_i2c *i2c)
{
    struct gd32_soft_i2c_config *cfg = (struct gd32_soft_i2c_config *)i2c->ops.data;

    rt_pin_mode(cfg->scl, PIN_MODE_OUTPUT_OD);
    rt_pin_mode(cfg->sda, PIN_MODE_OUTPUT_OD);

    rt_pin_write(cfg->scl, PIN_HIGH);
    rt_pin_write(cfg->sda, PIN_HIGH);
}

static void gd32_set_sda(void *data, rt_int32_t state)
{
    struct gd32_soft_i2c_config *cfg = (struct gd32_soft_i2c_config *)data;

    if(state)
    {
        rt_pin_write(cfg->sda, PIN_HIGH);
    }
    else
    {
        rt_pin_write(cfg->sda, PIN_LOW);
    }
}

static void gd32_set_scl(void *data, rt_int32_t state)
{
    struct gd32_soft_i2c_config *cfg = (struct gd32_soft_i2c_config *)data;

    if(state)
    {
        rt_pin_write(cfg->scl, PIN_HIGH);
    }
    else
    {
        rt_pin_write(cfg->scl, PIN_LOW);
    }
}

static rt_int32_t gd32_get_sda(void *data)
{
    struct gd32_soft_i2c_config *cfg = (struct gd32_soft_i2c_config *)data;

    return rt_pin_read(cfg->sda);
}

static rt_int32_t gd32_get_scl(void *data)
{
    struct gd32_soft_i2c_config *cfg = (struct gd32_soft_i2c_config *)data;

    return rt_pin_read(cfg->scl);
}

static void gd32_udelay(rt_uint32_t us)
{
    rt_uint32_t ticks;
    rt_uint32_t told, tnow, tcnt = 0;
    rt_uint32_t reload = SysTick->LOAD;

    ticks = us * reload / (1000000 / RT_TICK_PER_SECOND);
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

static const struct rt_i2c_bit_ops gd32_bit_ops_default =
{
    .data     = RT_NULL,
    .set_sda  = gd32_set_sda,
    .set_scl  = gd32_set_scl,
    .get_sda  = gd32_get_sda,
    .get_scl  = gd32_get_scl,
    .udelay   = gd32_udelay,
    .delay_us = 1,
    .timeout  = 100
};

static rt_err_t gd32_i2c_bus_unlock(const struct gd32_soft_i2c_config *cfg)
{
    rt_int32_t i = 0;

    if (PIN_LOW == rt_pin_read(cfg->sda))
    {
        while (i++ < 9)
        {
            rt_pin_write(cfg->scl, PIN_HIGH);
            gd32_udelay(100);
            rt_pin_write(cfg->scl, PIN_LOW);
            gd32_udelay(100);
        }
    }
    if (PIN_LOW == rt_pin_read(cfg->sda))
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static int rt_hw_i2c_init(void)
{
    rt_size_t obj_num = sizeof(i2c_obj) / sizeof(i2c_obj[0]);
    rt_err_t result;

    for (int i = 0; i < obj_num; i++)
    {
        i2c_obj[i].ops = gd32_bit_ops_default;
        i2c_obj[i].ops.data = (void*)&soft_i2c_config[i];
        i2c_obj[i].i2c2_bus.priv = &i2c_obj[i].ops;
        gd32_i2c_gpio_init(&i2c_obj[i]);
        result = rt_i2c_bit_add_bus(&i2c_obj[i].i2c2_bus, soft_i2c_config[i].bus_name);
        RT_ASSERT(result == RT_EOK);
        gd32_i2c_bus_unlock(&soft_i2c_config[i]);
    }

    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_hw_i2c_init);

#endif /* RT_USING_I2C_BITOPS */
