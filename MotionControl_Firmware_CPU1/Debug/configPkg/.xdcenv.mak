#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /home/yifan/ti/bios_6_46_04_53/packages
override XDCROOT = /home/yifan/TI_CCS/V6.2.0/xdctools_3_32_01_22_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /home/yifan/ti/bios_6_46_04_53/packages;/home/yifan/TI_CCS/V6.2.0/xdctools_3_32_01_22_core/packages;..
HOSTOS = Linux
endif
