# invoke SourceDir generated makefile for MotionControl_Firmware_CPU1.p28FP
MotionControl_Firmware_CPU1.p28FP: .libraries,MotionControl_Firmware_CPU1.p28FP
.libraries,MotionControl_Firmware_CPU1.p28FP: package/cfg/MotionControl_Firmware_CPU1_p28FP.xdl
	$(MAKE) -f /home/yifan/git/MotionControlFirmware_TI_C2000/MotionControl_Firmware_CPU1/src/makefile.libs

clean::
	$(MAKE) -f /home/yifan/git/MotionControlFirmware_TI_C2000/MotionControl_Firmware_CPU1/src/makefile.libs clean

