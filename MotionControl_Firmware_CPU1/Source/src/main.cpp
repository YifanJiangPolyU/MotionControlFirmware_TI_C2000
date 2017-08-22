
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


typedef struct DrvConfigType{
  uint16_t Op   : 1;    // 1=read, 0=write
  uint16_t Addr : 4;    // address
  uint16_t Data : 11;   // data
}DrvConfig;

typedef union DrvAccessType{
  DrvConfig bit;
  uint16_t  all;
}DrvAccess;

volatile uint16_t d1;
volatile uint16_t d2;

void main(void)
{
  SystemFullInit();

  InitTempSensor(3.0);

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

  // read drv8301 status
  DrvAccess handle;
  handle.bit.Op = 1;
  handle.bit.Addr = 0x02;
  handle.bit.Data = 0;
  SpibRegs.SPITXBUF = handle.all;
  DELAY_US(100000);
  SpibRegs.SPITXBUF = handle.all;

  DELAY_US(500000);

  //d1 = SpibRegs.SPIRXBUF;
  //d2 = SpibRegs.SPIRXBUF;

  //  start sys/bios
  BIOS_start();
}

//
// End of file
//
