#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>
#include "gd32f20x.h"
#include "drv_usart_config.h"
#include "gd32f2xx_std_msp.h"

#ifdef RT_USING_SERIAL

struct gd32_usart_config
{
    const char *name;
    uint32_t usart_periph;
    uint8_t only_sync;
    uint32_t tx_dma_periph;
    dma_channel_enum tx_dma_channel;
    uint32_t rx_dma_periph;
    dma_channel_enum rx_dma_channel;
    rt_uint8_t *read_buf;
    int read_bufsz;
    int rs485_control_pin;
    int rs485_send_logic;
};

struct gd32_usart
{
    struct rt_serial_device serial;
    uint32_t rx_index;
    uint8_t tx_activated;
    const struct gd32_usart_config *config;
};

#define DRV_USART_RS485_INIT()                                                                      \
    {                                                                                               \
        if((usart->config->rs485_control_pin >= 0) && (usart->config->rs485_send_logic >= 0))       \
        {                                                                                           \
            rt_pin_mode(usart->config->rs485_control_pin, PIN_MODE_OUTPUT);                         \
        }                                                                                           \
    }

#define DRV_USART_RS485_RECV()                                                                      \
    {                                                                                               \
        if((usart->config->rs485_control_pin >= 0) && (usart->config->rs485_send_logic >= 0))       \
        {                                                                                           \
            rt_pin_write(usart->config->rs485_control_pin, !(usart->config->rs485_send_logic));     \
        }                                                                                           \
    }

#define DRV_USART_RS485_SEND()                                                                      \
    {                                                                                               \
        if((usart->config->rs485_control_pin >= 0) && (usart->config->rs485_send_logic >= 0))       \
        {                                                                                           \
            rt_pin_write(usart->config->rs485_control_pin, usart->config->rs485_send_logic);        \
        }                                                                                           \
    }

static const struct gd32_usart_config usart_config[] = DRV_USART_TABLE;

static struct gd32_usart usart_obj[sizeof(usart_config) / sizeof(usart_config[0])] = {0};

static void _std_usart_deinit(struct gd32_usart *usart)
{
    uint32_t usart_periph = usart->config->usart_periph;

    usart_disable(usart_periph);

    STD_USART_MspDeInit(usart_periph);
}

static void _std_usart_abort(struct gd32_usart *usart)
{
    uint32_t usart_periph = usart->config->usart_periph;
    uint32_t tx_dma_periph = usart->config->tx_dma_periph;
    dma_channel_enum tx_dma_channel = usart->config->tx_dma_channel;
    uint32_t rx_dma_periph = usart->config->rx_dma_periph;
    dma_channel_enum rx_dma_channel = usart->config->rx_dma_channel;

    usart_interrupt_disable(usart_periph, USART_INT_RBNE);
    usart_interrupt_disable(usart_periph, USART_INT_IDLE);
    usart_interrupt_disable(usart_periph, USART_INT_PERR);
    usart_interrupt_disable(usart_periph, USART_INT_TBE);
    usart_interrupt_disable(usart_periph, USART_INT_TC);
    usart_interrupt_disable(usart_periph, USART_INT_ERR);

    if(tx_dma_periph)
    {
        usart_dma_transmit_config(usart_periph, USART_DENT_DISABLE);

        dma_interrupt_disable(tx_dma_periph, tx_dma_channel, DMA_INT_FTF);
        dma_interrupt_disable(tx_dma_periph, tx_dma_channel, DMA_INT_HTF);
        dma_interrupt_disable(tx_dma_periph, tx_dma_channel, DMA_INT_ERR);

        dma_channel_disable(tx_dma_periph, tx_dma_channel);

        dma_flag_clear(tx_dma_periph, tx_dma_channel, DMA_FLAG_G);
    }

    if(rx_dma_periph)
    {
        usart_dma_receive_config(usart_periph, USART_DENR_DISABLE);

        dma_interrupt_disable(rx_dma_periph, rx_dma_channel, DMA_INT_FTF);
        dma_interrupt_disable(rx_dma_periph, rx_dma_channel, DMA_INT_HTF);
        dma_interrupt_disable(rx_dma_periph, rx_dma_channel, DMA_INT_ERR);

        dma_channel_disable(rx_dma_periph, rx_dma_channel);

        dma_flag_clear(rx_dma_periph, rx_dma_channel, DMA_FLAG_G);
    }
}

static void _std_usart_abort_transmit(struct gd32_usart *usart)
{
    uint32_t usart_periph = usart->config->usart_periph;
    uint32_t tx_dma_periph = usart->config->tx_dma_periph;
    dma_channel_enum tx_dma_channel = usart->config->tx_dma_channel;

    usart_interrupt_disable(usart_periph, USART_INT_TBE);
    usart_interrupt_disable(usart_periph, USART_INT_TC);

    if(tx_dma_periph)
    {
        usart_dma_transmit_config(usart_periph, USART_DENT_DISABLE);

        dma_interrupt_disable(tx_dma_periph, tx_dma_channel, DMA_INT_FTF);
        dma_interrupt_disable(tx_dma_periph, tx_dma_channel, DMA_INT_HTF);
        dma_interrupt_disable(tx_dma_periph, tx_dma_channel, DMA_INT_ERR);

        dma_channel_disable(tx_dma_periph, tx_dma_channel);

        dma_flag_clear(tx_dma_periph, tx_dma_channel, DMA_FLAG_G);
    }
}

static void _std_usart_abort_receive(struct gd32_usart *usart)
{
    uint32_t usart_periph = usart->config->usart_periph;
    uint32_t rx_dma_periph = usart->config->rx_dma_periph;
    dma_channel_enum rx_dma_channel = usart->config->rx_dma_channel;

    usart_interrupt_disable(usart_periph, USART_INT_RBNE);
    usart_interrupt_disable(usart_periph, USART_INT_IDLE);
    usart_interrupt_disable(usart_periph, USART_INT_PERR);
    usart_interrupt_disable(usart_periph, USART_INT_ERR);

    if(rx_dma_periph)
    {
        usart_dma_receive_config(usart_periph, USART_DENR_DISABLE);

        dma_interrupt_disable(rx_dma_periph, rx_dma_channel, DMA_INT_FTF);
        dma_interrupt_disable(rx_dma_periph, rx_dma_channel, DMA_INT_HTF);
        dma_interrupt_disable(rx_dma_periph, rx_dma_channel, DMA_INT_ERR);

        dma_channel_disable(rx_dma_periph, rx_dma_channel);

        dma_flag_clear(rx_dma_periph, rx_dma_channel, DMA_FLAG_G);
    }
}

static void _std_usart_transmit(struct gd32_usart *usart, uint8_t *pData, uint16_t Size)
{
#define TRANSMIT_TIMEOUT    2

    uint32_t usart_periph = usart->config->usart_periph;
    struct rt_serial_device *serial = &usart->serial;

    DRV_USART_RS485_SEND();

    usart_flag_clear(usart_periph, USART_FLAG_TC);
    while(Size > 0)
    {
        while(usart_flag_get(usart_periph, USART_FLAG_TBE) == RESET);
        usart_data_transmit(usart_periph, *pData);
        pData++;
        Size--;

        if(!serial->sync_flag)
            return;
    }

    rt_tick_t timeout = rt_tick_get() + rt_tick_from_millisecond(TRANSMIT_TIMEOUT * 1000);
    while(usart_flag_get(usart_periph, USART_FLAG_TC) == RESET)
    {
        if(!serial->sync_flag)
            return;
        
        if((rt_tick_get() - timeout) < (RT_TICK_MAX / 2))
            break;
    }
    
    DRV_USART_RS485_RECV();
}

static void _std_usart_transmit_dma(struct gd32_usart *usart, uint8_t *pData, uint16_t Size)
{
    uint32_t usart_periph = usart->config->usart_periph;
    uint32_t tx_dma_periph = usart->config->tx_dma_periph;
    dma_channel_enum tx_dma_channel = usart->config->tx_dma_channel;

    dma_channel_disable(tx_dma_periph, tx_dma_channel);

    dma_flag_clear(tx_dma_periph, tx_dma_channel, DMA_FLAG_G);

    dma_transfer_number_config(tx_dma_periph, tx_dma_channel, Size);
    dma_memory_address_config(tx_dma_periph, tx_dma_channel, (uint32_t)pData);
    dma_periph_address_config(tx_dma_periph, tx_dma_channel, (uint32_t)&USART_DATA(usart_periph));

    dma_interrupt_disable(tx_dma_periph, tx_dma_channel, DMA_INT_HTF);
    dma_interrupt_enable(tx_dma_periph, tx_dma_channel, DMA_INT_FTF);
    dma_interrupt_enable(tx_dma_periph, tx_dma_channel, DMA_INT_ERR);

    dma_channel_enable(tx_dma_periph, tx_dma_channel);

    usart_flag_clear(usart_periph, USART_FLAG_TC);

    usart_dma_transmit_config(usart_periph, USART_DENT_ENABLE);
}

static void _std_usart_receive_dma(struct gd32_usart *usart, uint8_t *pData, uint16_t Size)
{
    uint32_t usart_periph = usart->config->usart_periph;
    uint32_t rx_dma_periph = usart->config->rx_dma_periph;
    dma_channel_enum rx_dma_channel = usart->config->rx_dma_channel;

    dma_channel_disable(rx_dma_periph, rx_dma_channel);

    dma_flag_clear(rx_dma_periph, rx_dma_channel, DMA_FLAG_G);

    dma_transfer_number_config(rx_dma_periph, rx_dma_channel, Size);
    dma_memory_address_config(rx_dma_periph, rx_dma_channel, (uint32_t)pData);
    dma_periph_address_config(rx_dma_periph, rx_dma_channel, (uint32_t)&USART_DATA(usart_periph));

    dma_interrupt_enable(rx_dma_periph, rx_dma_channel, DMA_INT_HTF);
    dma_interrupt_enable(rx_dma_periph, rx_dma_channel, DMA_INT_FTF);
    dma_interrupt_enable(rx_dma_periph, rx_dma_channel, DMA_INT_ERR);

    dma_channel_enable(rx_dma_periph, rx_dma_channel);

    USART_STAT0(usart_periph);
    USART_DATA(usart_periph);

    usart_interrupt_enable(usart_periph, USART_INT_PERR);
    usart_interrupt_enable(usart_periph, USART_INT_ERR);
    usart_interrupt_enable(usart_periph, USART_INT_IDLE);

    usart_dma_receive_config(usart_periph, USART_DENR_ENABLE);
}

static rt_err_t gd32_usart_init(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    RT_ASSERT(serial != RT_NULL);

    struct gd32_usart *usart = (struct gd32_usart *)serial;
    uint32_t usart_periph = usart->config->usart_periph;

    rt_base_t level = rt_hw_interrupt_disable();

    if(serial->parent.ref_count)
        _std_usart_deinit(usart);
    
    usart->rx_index = serial->rx_rb.buffer_size;
    usart->tx_activated = RT_FALSE;
    DRV_USART_RS485_INIT();
    DRV_USART_RS485_RECV();

    STD_USART_MspInit(usart_periph);
    usart_deinit(usart_periph);

    usart_disable(usart_periph);

    usart_baudrate_set(usart_periph, cfg->baud_rate);
    usart_receive_config(usart_periph, USART_RECEIVE_ENABLE);
    usart_transmit_config(usart_periph, USART_TRANSMIT_ENABLE);
    usart_hardware_flow_rts_config(usart_periph, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(usart_periph, USART_CTS_DISABLE);

    switch(cfg->data_bits)
    {
        case DATA_BITS_8:
        {
            if(cfg->parity == PARITY_ODD || cfg->parity == PARITY_EVEN)
                usart_word_length_set(usart_periph, USART_WL_9BIT);
            else
                usart_word_length_set(usart_periph, USART_WL_8BIT);
        }
        break;

        case DATA_BITS_9:
            usart_word_length_set(usart_periph, USART_WL_9BIT);
        break;

        default:
            usart_word_length_set(usart_periph, USART_WL_8BIT);
        break;
    }

    switch(cfg->stop_bits)
    {
        case STOP_BITS_1:
            usart_stop_bit_set(usart_periph, USART_STB_1BIT);
        break;

        case STOP_BITS_2:
            usart_stop_bit_set(usart_periph, USART_STB_2BIT);
        break;

        default:
            usart_stop_bit_set(usart_periph, USART_STB_1BIT);
        break;
    }

    switch(cfg->parity)
    {
        case PARITY_NONE:
            usart_parity_config(usart_periph, USART_PM_NONE);
        break;

        case PARITY_ODD:
            usart_parity_config(usart_periph, USART_PM_ODD);
        break;

        case PARITY_EVEN:
            usart_parity_config(usart_periph, USART_PM_EVEN);
        break;

        default:
            usart_parity_config(usart_periph, USART_PM_NONE);
        break;
    }

    usart_enable(usart_periph);

    _std_usart_abort(usart);
    _std_usart_receive_dma(usart, serial->rx_rb.buffer_ptr, serial->rx_rb.buffer_size);

    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t gd32_usart_deinit(struct rt_serial_device *serial)
{
    RT_ASSERT(serial != RT_NULL);

    struct gd32_usart *usart = (struct gd32_usart *)serial;

    rt_base_t level = rt_hw_interrupt_disable();
    _std_usart_deinit(usart);
    DRV_USART_RS485_RECV();
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t gd32_usart_stop(struct rt_serial_device *serial)
{
    RT_ASSERT(serial != RT_NULL);

    struct gd32_usart *usart = (struct gd32_usart *)serial;

    rt_base_t level = rt_hw_interrupt_disable();
    _std_usart_abort(usart);
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t gd32_usart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    return -RT_ERROR;
}

static rt_size_t gd32_usart_start_tx(struct rt_serial_device *serial, const void *buffer, rt_size_t size)
{
    RT_ASSERT(serial != RT_NULL);

    struct gd32_usart *usart = (struct gd32_usart *)serial;

    if(serial->sync_flag)
    {
        rt_base_t level = rt_hw_interrupt_disable();
        _std_usart_abort_transmit(usart);
        rt_hw_interrupt_enable(level);

        _std_usart_transmit(usart, (uint8_t *)buffer, size);
        return size;
    }

    {
        rt_base_t level = rt_hw_interrupt_disable();
        if(usart->tx_activated != RT_TRUE)
            DRV_USART_RS485_SEND();
        _std_usart_abort_transmit(usart);
        _std_usart_transmit_dma(usart, (uint8_t *)buffer, size);
        usart->tx_activated = RT_TRUE;
        rt_hw_interrupt_enable(level);
    }

    return size;
}

static rt_err_t gd32_usart_stop_tx(struct rt_serial_device *serial)
{
    RT_ASSERT(serial != RT_NULL);

    struct gd32_usart *usart = (struct gd32_usart *)serial;

    rt_base_t level = rt_hw_interrupt_disable();
    _std_usart_abort_transmit(usart);
    DRV_USART_RS485_RECV();
    usart->tx_activated = RT_FALSE;
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static rt_err_t gd32_usart_restart_rx(struct rt_serial_device *serial)
{
    RT_ASSERT(serial != RT_NULL);

    struct gd32_usart *usart = (struct gd32_usart *)serial;

    rt_base_t level = rt_hw_interrupt_disable();
    usart->rx_index = serial->rx_rb.buffer_size;
    usart->tx_activated = RT_FALSE;
    DRV_USART_RS485_RECV();
    _std_usart_abort_receive(usart);
    _std_usart_receive_dma(usart, serial->rx_rb.buffer_ptr, serial->rx_rb.buffer_size);
    rt_hw_interrupt_enable(level);

    return RT_EOK;
}

static void _std_usart_rxidlecallback(struct gd32_usart *usart)
{
    uint32_t rx_dma_periph = usart->config->rx_dma_periph;
    dma_channel_enum rx_dma_channel = usart->config->rx_dma_channel;
    struct rt_serial_device *serial = &usart->serial;
    uint32_t recv_len = 0;

    rt_base_t level = rt_hw_interrupt_disable();
    uint32_t index = dma_transfer_number_get(rx_dma_periph, rx_dma_channel);
    if(index < usart->rx_index)
    {
        recv_len = usart->rx_index - index;
        usart->rx_index = index;
    }
    rt_hw_interrupt_enable(level);

    serial->rx_indicate(serial, recv_len);
}

static void _std_usart_rxcpltcallback(struct gd32_usart *usart)
{
    uint32_t rx_dma_periph = usart->config->rx_dma_periph;
    dma_channel_enum rx_dma_channel = usart->config->rx_dma_channel;
    struct rt_serial_device *serial = &usart->serial;
    uint32_t recv_len = 0;

    rt_base_t level = rt_hw_interrupt_disable();
    uint32_t index = dma_transfer_number_get(rx_dma_periph, rx_dma_channel);
    if(index >= usart->rx_index)
    {
        recv_len = usart->rx_index + serial->rx_rb.buffer_size - index;
        usart->rx_index = index;
    }
    rt_hw_interrupt_enable(level);
    
    serial->rx_indicate(serial, recv_len);
}

static void _std_usart_rxhalfcpltcallback(struct gd32_usart *usart)
{
    uint32_t rx_dma_periph = usart->config->rx_dma_periph;
    dma_channel_enum rx_dma_channel = usart->config->rx_dma_channel;
    struct rt_serial_device *serial = &usart->serial;
    uint32_t recv_len = 0;

    rt_base_t level = rt_hw_interrupt_disable();
    uint32_t index = dma_transfer_number_get(rx_dma_periph, rx_dma_channel);
    if(index < usart->rx_index)
    {
        recv_len = usart->rx_index - index;
        usart->rx_index = index;
    }
    rt_hw_interrupt_enable(level);

    serial->rx_indicate(serial, recv_len);
}

static void _std_usart_txcpltcallback(struct gd32_usart *usart)
{
    uint32_t tx_dma_periph = usart->config->tx_dma_periph;
    dma_channel_enum tx_dma_channel = usart->config->tx_dma_channel;
    struct rt_serial_device *serial = &usart->serial;
    
    uint32_t buffer = DMA_CHMADDR(tx_dma_periph, tx_dma_channel);

    serial->tx_complete(serial, (void *)buffer);
}

static void _std_usart_errorcallback(struct gd32_usart *usart, uint32_t errorcode)
{
    struct rt_serial_device *serial = &usart->serial;

    serial->error_indicate(serial, errorcode);
}

void _std_usart_irqhandler(int index)
{
    int num = sizeof(usart_obj) / sizeof(usart_obj[0]);
    if((index < 0) || (index >= num))
        return;

    struct gd32_usart *usart = &usart_obj[index];
    uint32_t usart_periph = usart->config->usart_periph;
    uint32_t errorcode = 0;

    if(usart_interrupt_flag_get(usart_periph, USART_INT_FLAG_PERR) != RESET)
    {
        USART_STAT0(usart_periph);
        USART_DATA(usart_periph);

        errorcode |= RT_SERIAL_ERROR_PARITY;
    }

    if(usart_interrupt_flag_get(usart_periph, USART_INT_FLAG_ERR_FERR) != RESET)
    {
        USART_STAT0(usart_periph);
        USART_DATA(usart_periph);

        errorcode |= RT_SERIAL_ERROR_FRAMING;
    }

    if(usart_interrupt_flag_get(usart_periph, USART_INT_FLAG_ERR_NERR) != RESET)
    {
        USART_STAT0(usart_periph);
        USART_DATA(usart_periph);

        errorcode |= RT_SERIAL_ERROR_FRAMING;
    }

    if((usart_interrupt_flag_get(usart_periph, USART_INT_FLAG_RBNE_ORERR) != RESET) ||
       (usart_interrupt_flag_get(usart_periph, USART_INT_FLAG_ERR_ORERR) != RESET))
    {
        USART_STAT0(usart_periph);
        USART_DATA(usart_periph);

        errorcode |= RT_SERIAL_ERROR_OVERRUN;
    }

    if(errorcode)
    {
        _std_usart_abort_receive(usart);
        _std_usart_errorcallback(usart, errorcode);
        return;
    }

    if(usart_interrupt_flag_get(usart_periph, USART_INT_FLAG_IDLE) != RESET)
    {
        USART_STAT0(usart_periph);
        USART_DATA(usart_periph);

        _std_usart_rxidlecallback(usart);
    }

    if(usart_interrupt_flag_get(usart_periph, USART_INT_FLAG_TC) != RESET)
    {
        usart_flag_clear(usart_periph, USART_FLAG_TC);

        usart_interrupt_disable(usart_periph, USART_INT_TC);

        _std_usart_txcpltcallback(usart);
    }
}

void _std_usart_txdma_irqhandler(int index)
{
    int num = sizeof(usart_obj) / sizeof(usart_obj[0]);
    if((index < 0) || (index >= num))
        return;

    struct gd32_usart *usart = &usart_obj[index];
    uint32_t usart_periph = usart->config->usart_periph;
    uint32_t tx_dma_periph = usart->config->tx_dma_periph;
    dma_channel_enum tx_dma_channel = usart->config->tx_dma_channel;

    if(dma_interrupt_flag_get(tx_dma_periph, tx_dma_channel, DMA_INT_FLAG_ERR) != RESET)
    {
        _std_usart_abort(usart);
        _std_usart_errorcallback(usart, RT_SERIAL_ERROR_OTHER);
        return;
    }

    if(dma_interrupt_flag_get(tx_dma_periph, tx_dma_channel, DMA_INT_FLAG_FTF) != RESET)
    {
        dma_interrupt_disable(tx_dma_periph, tx_dma_channel, DMA_INT_FTF);
        dma_interrupt_disable(tx_dma_periph, tx_dma_channel, DMA_INT_ERR);

        dma_flag_clear(tx_dma_periph, tx_dma_channel, DMA_FLAG_FTF);

        usart_dma_transmit_config(usart_periph, USART_DENT_DISABLE);

        usart_interrupt_enable(usart_periph, USART_INT_TC);
    }
}

void _std_usart_rxdma_irqhandler(int index)
{
    int num = sizeof(usart_obj) / sizeof(usart_obj[0]);
    if((index < 0) || (index >= num))
        return;

    struct gd32_usart *usart = &usart_obj[index];
    uint32_t rx_dma_periph = usart->config->rx_dma_periph;
    dma_channel_enum rx_dma_channel = usart->config->rx_dma_channel;

    if(dma_interrupt_flag_get(rx_dma_periph, rx_dma_channel, DMA_INT_FLAG_ERR) != RESET)
    {
        _std_usart_abort(usart);
        _std_usart_errorcallback(usart, RT_SERIAL_ERROR_OTHER);
        return;
    }

    if(dma_interrupt_flag_get(rx_dma_periph, rx_dma_channel, DMA_INT_FLAG_HTF) != RESET)
    {
        dma_flag_clear(rx_dma_periph, rx_dma_channel, DMA_FLAG_HTF);

        _std_usart_rxhalfcpltcallback(usart);
    }

    if(dma_interrupt_flag_get(rx_dma_periph, rx_dma_channel, DMA_INT_FLAG_FTF) != RESET)
    {
        dma_flag_clear(rx_dma_periph, rx_dma_channel, DMA_FLAG_FTF);

        _std_usart_rxcpltcallback(usart);
    }
}

static const struct rt_uart_ops gd32_usart_ops =
{
    gd32_usart_init,
    gd32_usart_deinit,
    gd32_usart_stop,
    gd32_usart_control,
    gd32_usart_start_tx,
    gd32_usart_stop_tx,
    gd32_usart_restart_rx
};

void rt_hw_usart_monitor(struct rt_serial_device *serial)
{
    RT_ASSERT(serial != RT_NULL);

    struct gd32_usart *usart = (struct gd32_usart *)serial;
    uint32_t rx_dma_periph = usart->config->rx_dma_periph;
    dma_channel_enum rx_dma_channel = usart->config->rx_dma_channel;

    if(!serial->parent.ref_count)
        return;
    if(serial->error_flag)
        return;
    uint32_t index = dma_transfer_number_get(rx_dma_periph, rx_dma_channel);
    if(index >= usart->rx_index)
        return;
    
    uint32_t recv_len = 0;
    rt_base_t level = rt_hw_interrupt_disable();
    do
    {
        index = dma_transfer_number_get(rx_dma_periph, rx_dma_channel);
        if(index >= usart->rx_index)
            break;
        if(dma_flag_get(rx_dma_periph, rx_dma_channel, DMA_FLAG_HTF) != RESET)
            break;
        if(dma_flag_get(rx_dma_periph, rx_dma_channel, DMA_FLAG_FTF) != RESET)
            break;
        
        recv_len = usart->rx_index - index;
        usart->rx_index = index;
        
        serial->rx_indicate(serial, recv_len);
    }while(0);
    rt_hw_interrupt_enable(level);
}

int rt_hw_usart_init(void)
{
    int obj_num = sizeof(usart_obj) / sizeof(usart_obj[0]);
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

    for (int i = 0; i < obj_num; i++)
    {
        struct rt_serial_device *serial = &(usart_obj[i].serial);

        usart_obj[i].config = &usart_config[i];

        serial->config = config;
        rt_ringbuffer_init(&serial->rx_rb, usart_obj[i].config->read_buf, usart_obj[i].config->read_bufsz);
        serial->ops = &gd32_usart_ops;

        if(usart_obj[i].config->only_sync)
            rt_hw_serial_register(serial, usart_obj[i].config->name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        else
            rt_hw_serial_register(serial, usart_obj[i].config->name, RT_DEVICE_FLAG_RDWR | RT_SERIAL_FLAG_ASYNC, RT_NULL);
    }

    return RT_EOK;
}

#endif /* RT_USING_SERIAL */
