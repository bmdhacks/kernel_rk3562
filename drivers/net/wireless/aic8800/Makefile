#CONFIG_AIC8800_BTLPM_SUPPORT := m
#CONFIG_AIC8800_WLAN_SUPPORT := m
#CONFIG_AIC_WLAN_SUPPORT := m
#CONFIG_AIC_FW_PATH = "/usr/lib/kernel-overlays/base/lib/firmware"

obj-$(CONFIG_AIC8800_BTLPM_SUPPORT) += aic8800_btlpm/
obj-$(CONFIG_AIC8800_WLAN_SUPPORT) += aic8800_fdrv/
obj-$(CONFIG_AIC_WLAN_SUPPORT) += aic8800_bsp/

########## config option ##########
export CONFIG_USE_FW_REQUEST = n
export CONFIG_PREALLOC_RX_SKB = y
export CONFIG_PREALLOC_TXQ = y
export CONFIG_OOB = n
export CONFIG_GPIO_WAKEUP = n
export CONFIG_RESV_MEM_SUPPORT = y
###################################

########## Platform support list ##########
#export CONFIG_PLATFORM_ROCKCHIP = n
export CONFIG_PLATFORM_ROCKCHIP2 = y
#export CONFIG_PLATFORM_ALLWINNER = n
#export CONFIG_PLATFORM_AMLOGIC = n
#export CONFIG_PLATFORM_UBUNTU = n
