# MotionControlFirmware_TI_C2000
3 Phase PMSM Controller Firmware, based on TMS320F2837xD

## MotionControl_Firmware_CPU1
This folder contains firmware for the TMS320F28375 MCU.

## mcs_interface
This folder contains ROS nodes to communicate with the driver, using a CANOpen protocol realized over UART

## obd_code_generation
This folder contains a program that automatically generates the CANOpen object dictionary initialization code for the firmware. The program reads data from the obd_code_generation/src/input.txt file, and generates object dictionary initialization code automatically. 

At this moment this program contains hard-coded paths to files. It will be replaced with a more flexible configuration scheme in the future.
