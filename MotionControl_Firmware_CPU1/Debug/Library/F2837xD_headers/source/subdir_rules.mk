################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
Library/F2837xD_headers/source/F2837xD_GlobalVariableDefs.obj: ../Library/F2837xD_headers/source/F2837xD_GlobalVariableDefs.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: $<'
	@echo 'Invoking: C2000 Compiler'
	"/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/bin/cl2000" -v28 -mt -ml --tmu_support=tmu0 --cla_support=cla1 --vcu_support=vcu2 --float_support=fpu32 --include_path="/home/yifan/TI_CCS/V6.2.0/ccsv6/tools/compiler/c2000_15.12.3.LTS/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_common/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Library/F2837xD_headers/include" --include_path="/home/yifan/TI_CCS_MotionControl_workspace/MotionControl_Firmware_CPU1/Source/inc" -g --define=CPU1 --define=_FLASH --display_error_number --diag_wrap=off --diag_warning=225 --preproc_with_compile --preproc_dependency="Library/F2837xD_headers/source/F2837xD_GlobalVariableDefs.d" --obj_directory="Library/F2837xD_headers/source" $(GEN_OPTS__FLAG) "$(shell echo $<)"
	@echo 'Finished building: $<'
	@echo ' '


