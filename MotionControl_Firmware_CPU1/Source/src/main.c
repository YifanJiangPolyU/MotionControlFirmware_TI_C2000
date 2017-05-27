
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>

#include "stdint.h"
#include "F28x_Project.h"
#include "SystemInit.h"
#include "CPU1_CLA1_common.h"

extern Mailbox_Handle ADC_fifo;

/*
 *  ======== taskFxn ========
 */
 #ifndef __cplusplus
     #ifdef __TI_COMPILER_VERSION__
         #if __TI_COMPILER_VERSION__ >= 15009000
             #pragma CODE_SECTION(taskFxn, ".TI.ramfunc");
         #else
             #pragma CODE_SECTION(taskFxn, "ramfuncs");
         #endif
     #endif
 #endif
Void taskFxn(UArg a0, UArg a1)
{

  for(;;){
    static uint16_t i = 0;
    static uint16_t sss = 0;

		GpioDataRegs.GPADAT.bit.GPIO31 = 1;
    Mailbox_pend(ADC_fifo, &sss, BIOS_NO_WAIT);
    System_printf("ADCA res: %d\n", sss);
    Task_sleep(500);
		GpioDataRegs.GPADAT.bit.GPIO31 = 0;
    Task_sleep(500);
    System_printf("exit taskFxn(): %d\n", i);
    i += 1;
  }
}


// Function Prototypes
interrupt void adca1_isr(void);



void main(void)
{

  // Initialize System Control:
  InitSysCtrl();
  SystemMemoryInit();

  // temporarily disable interrupt
  // re-enabled inside Interrupt_init()
  DINT;

  // Map ISR functions
  // EALLOW;
  // PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
  // EDIS;

  // Configure other peripherals
  GPIO_GroupInit();
  ADC_GroupInit();
  EPWM_GroupInit();

  // configure CLA
  CLA_ConfigClaMemory();
  CLA_InitCpu1Cla1();

  // configure interrupt
  Interrupt_Init();

  EALLOW;
  // sync ePWM
  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

  // start ePWM
  EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
  EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
  EDIS;

  //  start sys/bios
  BIOS_start();
}


// adca1_isr
interrupt void adca1_isr(void)
{
    sensorSampleA = AdcaResultRegs.ADCRESULT0;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// End of file
//
