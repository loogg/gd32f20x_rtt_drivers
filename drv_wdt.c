#include <rtthread.h>
#include <rtdevice.h>
#include "gd32f20x.h"

#ifdef RT_USING_WDT

#define DBG_TAG "drv.wdt"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

static rt_watchdog_t watchdog;
static struct rt_watchdog_ops ops;

static rt_err_t wdt_init(rt_watchdog_t *wdt)
{
    uint32_t count = 0x100;
    rcu_osci_on(RCU_IRC40K);
    while(SUCCESS != rcu_osci_stab_wait(RCU_IRC40K) && (--count));

    if (count == 0)
    {
        LOG_E("RCU_IRC40K READY ERROR");
        return -RT_ERROR;
    }

    //4000*3.2ms
    if (fwdgt_config(4000, FWDGT_PSC_DIV128) != SUCCESS)
    {
        LOG_E("wdt init failed.");
        return -RT_ERROR;
    }
    fwdgt_enable();

    return RT_EOK;
}

static rt_err_t wdt_control(rt_watchdog_t *wdt, int cmd, void *arg)
{
    switch(cmd)
    {
        /* feed the watchdog */
        case RT_DEVICE_CTRL_WDT_KEEPALIVE:
            fwdgt_counter_reload();
        break;

        /* set watchdog timeout */
        case RT_DEVICE_CTRL_WDT_SET_TIMEOUT:
        {
            if(fwdgt_config(*((rt_uint32_t *)arg), FWDGT_PSC_DIV128) != SUCCESS)
            {
                LOG_E("wdg set timeout failed.");
                return -RT_ERROR;
            }
        }
        break;

        default:
            return -RT_ERROR;
    }
    return RT_EOK;
}

static int rt_wdt_init(void)
{
    ops.init = &wdt_init;
    ops.control = &wdt_control;
    watchdog.ops = &ops;
    /* register watchdog device */
    if(rt_hw_watchdog_register(&watchdog, "wdt", RT_DEVICE_FLAG_DEACTIVATE, RT_NULL) != RT_EOK)
    {
        LOG_E("wdt device register failed.");
        return -RT_ERROR;
    }
    LOG_D("wdt device register success.");
    return RT_EOK;
}
INIT_BOARD_EXPORT(rt_wdt_init);

#endif /* RT_USING_WDT */
