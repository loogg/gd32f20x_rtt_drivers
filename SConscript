import os
import rtconfig
from building import *

cwd = GetCurrentDir()

src = Glob('*.c')

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
