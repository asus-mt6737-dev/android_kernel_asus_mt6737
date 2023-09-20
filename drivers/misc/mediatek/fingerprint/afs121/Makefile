# Copyright (C) Microarray
# MicroArray Fprint Driver Code
# MAKEFILE
# Date: 2016-10-12
# Version: v4.0.01 
# Author: guq
# Contact: guq@microarray.com.cn


subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include \
					-I$(srctree)/drivers/misc/mediatek/include/mt-plat

MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
subdir-ccflags-y += -Werror
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat
subdir-ccflags-y += -I$(srctree)/drivers/spi/mediatek/$(MTK_PLATFORM)



#ifeq ($(CONFIG_MICROARRAY_FP),y)

#ifeq ($(XXXX_TEE), y)
obj-y	+= madev_tee.o mtk-settings.o
#else
#obj-y	+= madev.o mtk-settings.o
#endif

#endif


