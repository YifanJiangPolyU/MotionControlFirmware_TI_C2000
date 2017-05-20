################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Library/F2837xD_common/source/F2837xD_Adc.obj: ../Library/F2837xD_common/source/F2837xD_Adc.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_Adc.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_CpuTimers.obj: ../Library/F2837xD_common/source/F2837xD_CpuTimers.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_CpuTimers.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_DBGIER.obj: ../Library/F2837xD_common/source/F2837xD_DBGIER.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_DBGIER.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_DefaultISR.obj: ../Library/F2837xD_common/source/F2837xD_DefaultISR.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_DefaultISR.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_Dma.obj: ../Library/F2837xD_common/source/F2837xD_Dma.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_Dma.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_Emif.obj: ../Library/F2837xD_common/source/F2837xD_Emif.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_Emif.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_Gpio.obj: ../Library/F2837xD_common/source/F2837xD_Gpio.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_Gpio.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_I2C.obj: ../Library/F2837xD_common/source/F2837xD_I2C.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_I2C.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_Ipc.obj: ../Library/F2837xD_common/source/F2837xD_Ipc.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_Ipc.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_Ipc_Driver.obj: ../Library/F2837xD_common/source/F2837xD_Ipc_Driver.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_Ipc_Driver.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_Ipc_Driver_Lite.obj: ../Library/F2837xD_common/source/F2837xD_Ipc_Driver_Lite.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_Ipc_Driver_Lite.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_Ipc_Driver_Util.obj: ../Library/F2837xD_common/source/F2837xD_Ipc_Driver_Util.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_Ipc_Driver_Util.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_Mcbsp.obj: ../Library/F2837xD_common/source/F2837xD_Mcbsp.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_Mcbsp.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_PieCtrl.obj: ../Library/F2837xD_common/source/F2837xD_PieCtrl.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_PieCtrl.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_PieVect.obj: ../Library/F2837xD_common/source/F2837xD_PieVect.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_PieVect.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_Sci.obj: ../Library/F2837xD_common/source/F2837xD_Sci.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_Sci.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_SysCtrl.obj: ../Library/F2837xD_common/source/F2837xD_SysCtrl.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_SysCtrl.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_sci_io.obj: ../Library/F2837xD_common/source/F2837xD_sci_io.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_sci_io.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_sdfm_drivers.obj: ../Library/F2837xD_common/source/F2837xD_sdfm_drivers.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_sdfm_drivers.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_struct.obj: ../Library/F2837xD_common/source/F2837xD_struct.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_struct.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Library/F2837xD_common/source/F2837xD_usDelay.obj: ../Library/F2837xD_common/source/F2837xD_usDelay.asm $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_common/source/F2837xD_usDelay.d" --obj_directory="Library/F2837xD_common/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


