################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Source/src/CPU1_CLA1_interrupt.obj: ../Source/src/CPU1_CLA1_interrupt.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/src/CPU1_CLA1_interrupt.d" --obj_directory="Source/src" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Source/src/CPU1_CLA1_task.obj: ../Source/src/CPU1_CLA1_task.cla $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/src/CPU1_CLA1_task.d" --obj_directory="Source/src" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Source/src/CommutationMaster.obj: ../Source/src/CommutationMaster.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/src/CommutationMaster.d" --obj_directory="Source/src" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Source/src/ControlProcessMaster.obj: ../Source/src/ControlProcessMaster.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/src/ControlProcessMaster.d" --obj_directory="Source/src" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Source/src/CurrentLoopSweepSine.obj: ../Source/src/CurrentLoopSweepSine.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/src/CurrentLoopSweepSine.d" --obj_directory="Source/src" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Source/src/IPCMaster_CPU1.obj: ../Source/src/IPCMaster_CPU1.cpp $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/src/IPCMaster_CPU1.d" --obj_directory="Source/src" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Source/src/SystemInit.obj: ../Source/src/SystemInit.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/src/SystemInit.d" --obj_directory="Source/src" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '

Source/src/main.obj: ../Source/src/main.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Source/src/main.d" --obj_directory="Source/src" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


