#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "gd32f20x.h"
#include "drv_gpio_config.h"

#ifdef RT_USING_PIN

struct gd32_gpio
{
    int index;
    rcu_periph_enum rcu_periph;
    uint32_t gpio_periph;
    uint32_t pin;
};

struct gd32_gpio_irq
{
    const int index;
    const uint8_t exti_port;
    const uint8_t exti_pin;
    const exti_line_enum linex;
    const IRQn_Type irqno;
    int used;
    int mode;
    void (*hdr)(void *args);
    void *args;
};

/* ----------------------------根据实际情况更改------------------------------- */
static const struct gd32_gpio gpio_obj[] = DRV_GPIO_TABLE;

static struct gd32_gpio_irq gpio_irq_obj[] = DRV_GPIO_IRQ_TABLE;
/* -------------------------------------------------------------------------- */

static uint32_t pin_irq_enable_mask = 0;

static const struct gd32_gpio *get_gpio(int pin)
{
    int num = sizeof(gpio_obj) / sizeof(gpio_obj[0]);
    if((pin < 0) || (pin >= num))
        return RT_NULL;
    
    return &gpio_obj[pin];
}

static void gd32_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    const struct gd32_gpio *gpio = get_gpio(pin);
    if(gpio == RT_NULL)
        return;
    
    rcu_periph_clock_enable(gpio->rcu_periph);

    uint32_t pin_mode = GPIO_MODE_OUT_PP;

    switch(mode)
    {
        case PIN_MODE_OUTPUT:
            pin_mode = GPIO_MODE_OUT_PP;
        break;

        case PIN_MODE_INPUT:
            pin_mode = GPIO_MODE_IN_FLOATING;
        break;

        case PIN_MODE_INPUT_PULLUP:
            pin_mode = GPIO_MODE_IPU;
        break;

        case PIN_MODE_INPUT_PULLDOWN:
            pin_mode = GPIO_MODE_IPD;
        break;

        case PIN_MODE_OUTPUT_OD:
            pin_mode = GPIO_MODE_OUT_OD;
        break;

        default:
            return;
    }

    gpio_init(gpio->gpio_periph, pin_mode, GPIO_OSPEED_50MHZ, gpio->pin);
}

static void gd32_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    const struct gd32_gpio *gpio = get_gpio(pin);
    if(gpio == RT_NULL)
        return;

    gpio_bit_write(gpio->gpio_periph, gpio->pin, (bit_status)(value ? 1 : 0));
}

static int gd32_pin_read(rt_device_t dev, rt_base_t pin)
{
    int value = PIN_LOW;
    const struct gd32_gpio *gpio = get_gpio(pin);
    if(gpio == RT_NULL)
        return value;

    value = gpio_input_bit_get(gpio->gpio_periph, gpio->pin);

    return value;
}

static rt_err_t gd32_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                                    rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
    if(get_gpio(pin) == RT_NULL)
        return -RT_ERROR;

    struct gd32_gpio_irq *gpio_irq = RT_NULL;
    int gpio_irq_num = sizeof(gpio_irq_obj) / sizeof(gpio_irq_obj[0]);

    for (int i = 0; i < gpio_irq_num; i++)
    {
        if(gpio_irq_obj[i].index == pin)
        {
            gpio_irq = &gpio_irq_obj[i];
            break;
        }
    }

    if(gpio_irq == RT_NULL)
        return -RT_ERROR;
    
    rt_base_t level = rt_hw_interrupt_disable();
    if(gpio_irq->used)
    {
        if(mode != gpio_irq->mode)
        {
            rt_hw_interrupt_enable(level);
            return -RT_ERROR;
        }
        gpio_irq->hdr = hdr;
        gpio_irq->args = args;
    }
    else
    {
        gpio_irq->mode = mode;
        gpio_irq->hdr = hdr;
        gpio_irq->args = args;
    }
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t gd32_pin_detach_irq(struct rt_device *device, rt_int32_t pin)
{
    if(get_gpio(pin) == RT_NULL)
        return -RT_ERROR;

    struct gd32_gpio_irq *gpio_irq = RT_NULL;
    int gpio_irq_num = sizeof(gpio_irq_obj) / sizeof(gpio_irq_obj[0]);

    for (int i = 0; i < gpio_irq_num; i++)
    {
        if(gpio_irq_obj[i].index == pin)
        {
            gpio_irq = &gpio_irq_obj[i];
            break;
        }
    }

    if(gpio_irq == RT_NULL)
        return -RT_ERROR;
    
    rt_base_t level = rt_hw_interrupt_disable();
    gpio_irq->hdr = RT_NULL;
    gpio_irq->args = RT_NULL;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t gd32_pin_irq_enable(struct rt_device *device, rt_base_t pin, rt_uint32_t enabled)
{
    const struct gd32_gpio *gpio = get_gpio(pin);
    if(gpio == RT_NULL)
        return -RT_ERROR;

    struct gd32_gpio_irq *gpio_irq = RT_NULL;
    int gpio_irq_num = sizeof(gpio_irq_obj) / sizeof(gpio_irq_obj[0]);

    for (int i = 0; i < gpio_irq_num; i++)
    {
        if(gpio_irq_obj[i].index == pin)
        {
            gpio_irq = &gpio_irq_obj[i];
            break;
        }
    }

    if(gpio_irq == RT_NULL)
        return -RT_ERROR;
    
    if(enabled == PIN_IRQ_ENABLE)
    {
        rt_base_t level = rt_hw_interrupt_disable();
        if(gpio_irq->used)
        {
            rt_hw_interrupt_enable(level);
            return -RT_ERROR;
        }

        exti_trig_type_enum trigger_mode;

        switch(gpio_irq->mode)
        {
            case PIN_IRQ_MODE_RISING:
                trigger_mode = EXTI_TRIG_RISING;
            break;

            case PIN_IRQ_MODE_FALLING:
                trigger_mode = EXTI_TRIG_FALLING;
            break;

            case PIN_IRQ_MODE_RISING_FALLING:
                trigger_mode = EXTI_TRIG_BOTH;
            break;

            default:
                rt_hw_interrupt_enable(level);
                return -RT_ERROR;
        }

        gpio_exti_source_select(gpio_irq->exti_port, gpio_irq->exti_pin);
        exti_init(gpio_irq->linex, EXTI_INTERRUPT, trigger_mode);
        exti_interrupt_flag_clear(gpio_irq->linex);
        nvic_irq_enable(gpio_irq->irqno, 5, 0);

        gpio_irq->used = 1;
        pin_irq_enable_mask |= gpio->pin;
        rt_hw_interrupt_enable(level);
    }
    else if(enabled == PIN_IRQ_DISABLE)
    {
        rt_base_t level = rt_hw_interrupt_disable();
        gpio_irq->used = 0;
        pin_irq_enable_mask &= ~gpio->pin;
        if((gpio->pin >= GPIO_PIN_5) && (gpio->pin <= GPIO_PIN_9))
        {
            if(!(pin_irq_enable_mask & (GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9)))
            {
                nvic_irq_disable(gpio_irq->irqno);
            }
        }
        else if((gpio->pin >= GPIO_PIN_10) && (gpio->pin <= GPIO_PIN_15))
        {
            if(!(pin_irq_enable_mask & (GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)))
            {
                nvic_irq_disable(gpio_irq->irqno);
            }
        }
        else
        {
            nvic_irq_disable(gpio_irq->irqno);
        }
        rt_hw_interrupt_enable(level);
    }
    else
        return -RT_ERROR;

    return RT_EOK;
}

static struct rt_pin_ops gd32_pin_ops = 
{
    gd32_pin_mode,
    gd32_pin_write,
    gd32_pin_read,
    gd32_pin_attach_irq,
    gd32_pin_detach_irq,
    gd32_pin_irq_enable,
    RT_NULL
};

void GD32_GPIO_EXTI_IRQHandler(int index)
{
    int num = sizeof(gpio_irq_obj) / sizeof(gpio_irq_obj[0]);
    if((index < 0) || (index >= num))
        return;

    struct gd32_gpio_irq *gpio_irq = &gpio_irq_obj[index];

    if(exti_interrupt_flag_get(gpio_irq->linex) != RESET)
    {
        exti_interrupt_flag_clear(gpio_irq->linex);
        if(gpio_irq->hdr)
            gpio_irq->hdr(gpio_irq->args);
    }
}

int rt_hw_pin_init(void)
{
    return rt_device_pin_register("pin", &gd32_pin_ops, RT_NULL);
}

#endif /* RT_USING_PIN */
