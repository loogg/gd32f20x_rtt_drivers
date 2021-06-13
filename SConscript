import os
import rtconfig
from building import *

cwd = GetCurrentDir()

src = Split("""
""")

if GetDepend(['BSP_USING_GPIO']):
    src += ['drv_gpio.c']

if GetDepend(['BSP_USING_USART']):
    src += ['drv_usart.c']

if GetDepend(['BSP_USING_WDT']):
    src += ['drv_wdt.c']

if GetDepend(['BSP_USING_I2C']):
    src += ['drv_soft_i2c.c']

if GetDepend(['BSP_USING_SPI']):
    src += ['drv_spi.c']

if GetDepend(['BSP_USING_ON_CHIP_FLASH']):
    src += ['drv_flash.c']

path = [cwd]

group = DefineGroup('Drivers', src, depend = [''], CPPPATH = path)
objs = [group]

src = Glob('Libraries/GD32F20x_standard_peripheral/Source/*.c')

path = [cwd + '/Libraries/CMSIS',
        cwd + '/Libraries/GD32F20x_standard_peripheral/Include']

CPPDEFINES = ['USE_STDPERIPH_DRIVER']
group = DefineGroup('Libraries', src, depend = [''], CPPPATH = path, CPPDEFINES = CPPDEFINES)
objs += [group]

Return('objs')
