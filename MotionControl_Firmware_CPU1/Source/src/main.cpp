
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include "stdint.h"
#include "F28x_Project.h"
#include "Drivers/SystemInit/SystemInit.h"
#include "CPU1_CLA1_common.h"
#include "ControlProcessMaster.h"
#include "SystemWarehouse.h"



/*
 *  ======== taskFxn ========

#pragma CODE_SECTION(".TI.ramfunc");
extern "C" Void taskFxn(UArg a0, UArg a1)
{
  for(;;){
		GpioDataRegs.GPADAT.bit.GPIO31 = 1;
    Task_sleep(500);
		GpioDataRegs.GPADAT.bit.GPIO31 = 0;
    Task_sleep(500);
  }
}
 */

void main(void)
{
  SystemFullInit();

  // configure interrupt
  // Interrupt_Init();

  CreateSystemWarehouseInstance();

  EALLOW;
  // sync ePWM clock
  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

  // start ePWM
  EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA to trigger ADC
  EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode

  EPwm4Regs.TBCTL.bit.CTRMODE = 2;  //unfreeze, and enter up-down mode
  EPwm5Regs.TBCTL.bit.CTRMODE = 2;
  EPwm6Regs.TBCTL.bit.CTRMODE = 2;

  // sync ePWM counter value
  CLA_Reset();
  EPwm1Regs.TBCTL.bit.SWFSYNC = 1;

  EDIS;

  // turn off red LED
  GpioDataRegs.GPBDAT.bit.GPIO34 = 1;

  // transmit some fummy data over SPI to test it
  SpibRegs.SPITXBUF = 0x11;
  SpibRegs.SPITXBUF = 0x22;
  SpibRegs.SPITXBUF = 0xAA;
  SpibRegs.SPITXBUF = 0xBB;

  //  start sys/bios
  BIOS_start();
}

//
// End of file
//
