#include $(srctree)/../$(MTK_TARGET_PROJECT_FOLDER)/eastaeon.mk
# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/synaptics_dsx/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/synaptics_dsx/include/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/synaptics_dsx/include/config_default/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-y += synaptics_dsx_fw_update.o
obj-y += synaptics_dsx_i2c.o
obj-y += synaptics_dsx_rmi_dev.o


