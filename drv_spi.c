#include <rtthread.h>
#include <rtdevice.h>
#include "gd32f20x.h"
#include "drv_spi_config.h"
#include "gd32f2xx_std_msp.h"

#ifdef RT_USING_SPI

#define DBG_TAG "drv.spi"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

struct gd32_hw_spi_cs
{
    int pin;
    int active_logic;
};

struct gd32_spi_config
{
    const char *bus_name;
    uint32_t spi_periph;
}; 

struct gd32_spi
{
    struct rt_spi_bus spi_bus;
    const struct gd32_spi_config *config;
    struct rt_spi_configuration *cfg;
};

#define DRV_SPI_CS_INIT()                                                       \
    {                                                                           \
        if((cs->pin >= 0) && (cs->active_logic >= 0))                           \
        {                                                                       \
            rt_pin_mode(cs->pin, PIN_MODE_OUTPUT);                              \
        }                                                                       \
    }

#define DRV_SPI_CS_ACTIVE()                                                     \
    {                                                                           \
        if((cs->pin >= 0) && (cs->active_logic >= 0))                           \
        {                                                                       \
            rt_pin_write(cs->pin, cs->active_logic);                            \
        }                                                                       \
    }

#define DRV_SPI_CS_INACTIVE()                                                   \
    {                                                                           \
        if((cs->pin >= 0) && (cs->active_logic >= 0))                           \
        {                                                                       \
            rt_pin_write(cs->pin, !(cs->active_logic));                         \
        }                                                                       \
    }

static const struct gd32_spi_config spi_config[] = DRV_SPI_TABLE;

static struct gd32_spi spi_obj[sizeof(spi_config) / sizeof(spi_config[0])] = {0};

static rt_err_t gd32_spi_init(struct gd32_spi *spi_drv, struct rt_spi_configuration *cfg)
{
    RT_ASSERT(spi_drv != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uint32_t spi_periph = spi_drv->config->spi_periph;
    STD_SPI_MSPInit(spi_periph);
    spi_i2s_deinit(spi_periph);

    spi_disable(spi_periph);

    spi_parameter_struct spi_init_struct;

    if (cfg->mode & RT_SPI_SLAVE)
    {
        spi_init_struct.device_mode = SPI_SLAVE;
    }
    else
    {
        spi_init_struct.device_mode = SPI_MASTER;
    }


    if (cfg->mode & RT_SPI_3WIRE)
    {
        spi_init_struct.trans_mode = SPI_TRANSMODE_BDTRANSMIT;
    }
    else
    {
        spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    }

    if(cfg->data_width == 8)
    {
        spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
    }
    else if(cfg->data_width == 16)
    {
        spi_init_struct.frame_size = SPI_FRAMESIZE_16BIT;
    }
    else
    {
        return RT_EIO;
    }

    switch(cfg->mode & RT_SPI_MODE_3)
    {
    case RT_SPI_MODE_0:
        spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
        break;
    case RT_SPI_MODE_1:
        spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
        break;
    case RT_SPI_MODE_2:
        spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_1EDGE;
        break;
    case RT_SPI_MODE_3:
        spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
        break;
    }

    if (cfg->mode & RT_SPI_NO_CS)
    {
        //FIXME: support hard
        spi_init_struct.nss = SPI_NSS_SOFT;
    }
    else
    {
        spi_init_struct.nss = SPI_NSS_SOFT;
    }

    rcu_clock_freq_enum spi_src;
    uint32_t spi_apb_clock;

    if (spi_periph == SPI1 || spi_periph == SPI2)
    {
        spi_src = CK_APB1;
    }
    else
    {
        spi_src = CK_APB2;
    }
    spi_apb_clock = rcu_clock_freq_get(spi_src);

    if (cfg->max_hz >= spi_apb_clock / 2)
    {
        spi_init_struct.prescale = SPI_PSC_2;
    }
    else if (cfg->max_hz >= spi_apb_clock / 4)
    {
        spi_init_struct.prescale = SPI_PSC_4;
    }
    else if (cfg->max_hz >= spi_apb_clock / 8)
    {
        spi_init_struct.prescale = SPI_PSC_8;
    }
    else if (cfg->max_hz >= spi_apb_clock / 16)
    {
        spi_init_struct.prescale = SPI_PSC_16;
    }
    else if (cfg->max_hz >= spi_apb_clock / 32)
    {
        spi_init_struct.prescale = SPI_PSC_32;
    }
    else if (cfg->max_hz >= spi_apb_clock / 64)
    {
        spi_init_struct.prescale = SPI_PSC_64;
    }
    else if (cfg->max_hz >= spi_apb_clock / 128)
    {
        spi_init_struct.prescale = SPI_PSC_128;
    }
    else
    {
        /*  min prescaler 256 */
        spi_init_struct.prescale = SPI_PSC_256;
    }

    /* MSB or LSB */
    if(cfg->mode & RT_SPI_MSB)
    {
        spi_init_struct.endian = SPI_ENDIAN_MSB;
    }
    else
    {
        spi_init_struct.endian = SPI_ENDIAN_LSB;
    }

    spi_init(spi_periph, &spi_init_struct);
    spi_crc_off(spi_periph);
    spi_nss_internal_high(spi_periph);
    spi_enable(spi_periph);

    return RT_EOK;
};

//FIXME: support hard
static rt_uint32_t spixfer(struct rt_spi_device* device, struct rt_spi_message* message)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);
    RT_ASSERT(device->bus->parent.user_data != RT_NULL);
    RT_ASSERT(message != RT_NULL);

    struct gd32_spi *spi_drv = (struct gd32_spi *)device->bus;
    uint32_t spi_periph = spi_drv->config->spi_periph;
    struct gd32_hw_spi_cs *cs = device->parent.user_data;

    if(message->cs_take)
    {
        DRV_SPI_CS_ACTIVE();
    }

    if (spi_drv->cfg->data_width == 8)
    {
        const rt_uint8_t *send_ptr = message->send_buf;
        rt_uint8_t *recv_ptr = message->recv_buf;
        rt_uint32_t size = message->length;

        while (size--)
        {
            rt_uint8_t data = 0xFF;

            if (send_ptr != RT_NULL)
            {
                data = *send_ptr++;
            }

            // Todo: replace register read/write by gd32f3 lib
            //Wait until the transmit buffer is empty
            while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TBE));
            // Send the byte
            spi_i2s_data_transmit(spi_periph, data);

            //Wait until a data is received
            while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RBNE));
            // Get the received data
            data = spi_i2s_data_receive(spi_periph);

            if (recv_ptr != RT_NULL)
            {
                *recv_ptr++ = data;
            }
        }

        SPI_DATA(spi_periph);
        SPI_STAT(spi_periph);
    }
    else if (spi_drv->cfg->data_width == 16)
    {
        const rt_uint16_t *send_ptr = message->send_buf;
        rt_uint16_t *recv_ptr = message->recv_buf;
        rt_uint32_t size = message->length;

        while (size--)
        {
            rt_uint16_t data = 0xFFFF;

            if (send_ptr != RT_NULL)
            {
                data = *send_ptr++;
            }

            //Wait until the transmit buffer is empty
            while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TBE));
            // Send the byte
            spi_i2s_data_transmit(spi_periph, data);

            //Wait until a data is received
            while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RBNE));
            // Get the received data
            data = spi_i2s_data_receive(spi_periph);

            if (recv_ptr != RT_NULL)
            {
                *recv_ptr++ = data;
            }
        }

        SPI_DATA(spi_periph);
        SPI_STAT(spi_periph);
    }

    /* release CS */
    if(message->cs_release)
    {
        DRV_SPI_CS_INACTIVE();
    }

    return message->length;
};

static rt_err_t spi_configure(struct rt_spi_device *device,
                              struct rt_spi_configuration *configuration)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);

    struct gd32_spi *spi_drv = (struct gd32_spi *)device->bus;
    spi_drv->cfg = configuration;

    return gd32_spi_init(spi_drv, configuration);
}

static const struct rt_spi_ops gd32_spi_ops =
{
    .configure = spi_configure,
    .xfer = spixfer,
};

static int rt_hw_spi_bus_init(void)
{
    rt_err_t result = RT_EOK;
    int obj_num = sizeof(spi_obj) / sizeof(spi_obj[0]);

    for (int i = 0; i < obj_num; i++)
    {
        spi_obj[i].config = &spi_config[i];
        spi_obj[i].spi_bus.parent.user_data = (void *)&spi_config[i];

        result = rt_spi_bus_register(&spi_obj[i].spi_bus, spi_config[i].bus_name, &gd32_spi_ops);
        RT_ASSERT(result == RT_EOK);
    }

    return result;
}

rt_err_t rt_hw_spi_device_attach(const char *bus_name, const char *device_name, int pin, int active_logic)
{
    RT_ASSERT(bus_name != RT_NULL);
    RT_ASSERT(device_name != RT_NULL);

    rt_err_t result;
    struct rt_spi_device *spi_device;
    struct gd32_hw_spi_cs *cs;

    spi_device = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));
    RT_ASSERT(spi_device != RT_NULL);
    cs = (struct gd32_hw_spi_cs *)rt_malloc(sizeof(struct gd32_hw_spi_cs));
    RT_ASSERT(cs != RT_NULL);
    cs->pin = pin;
    cs->active_logic = active_logic;
    DRV_SPI_CS_INIT();
    DRV_SPI_CS_INACTIVE();

    result = rt_spi_bus_attach_device(spi_device, device_name, bus_name, (void *)cs);

    if (result != RT_EOK)
    {
        LOG_E("%s attach to %s faild, %d\n", device_name, bus_name, result);
    }

    RT_ASSERT(result == RT_EOK);

    LOG_D("%s attach to %s done", device_name, bus_name);

    return result;
}

static int rt_hw_spi_init(void)
{
    return rt_hw_spi_bus_init();
}
INIT_BOARD_EXPORT(rt_hw_spi_init);

#endif /* RT_USING_SPI */
