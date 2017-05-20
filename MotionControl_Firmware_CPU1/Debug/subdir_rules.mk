################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
build-2052408273: ../MotionControl_Firmware_CPU1.cfg
	@echo 'Building file: $<'
	@echo 'Invoking: XDCtools'
	"/home/yifan/TI_CCS/V6.2.0/xdctools_3_32_01_22_core/xs" --xdcpath="/home/yifan/ti/bios_6_46_04_53/packages;" xdc.tools.configuro -o configPkg -t ti.targets.C28_float -p ti.platforms.tms320x28:TMS320F28379D -r debug -c "/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS" "$<"
	@echo 'Finished building: $<'
	@echo ' '

configPkg/linker.cmd: build-2052408273
configPkg/compiler.opt: build-2052408273
configPkg/: build-2052408273


