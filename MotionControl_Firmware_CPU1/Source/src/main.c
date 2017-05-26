
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include "stdint.h"
#include "F28x_Project.h"
#include "SystemInit.h"


// Globals
volatile Uint16 sensorSample;
volatile int16 sensorTemp;

#ifdef __cplusplus
    #pragma DATA_SECTION("Cla1ToCpuMsgRAM")
    float result;
    #pragma DATA_SECTION(A,"CpuToCla1MsgRAM");
    float init;
#else
    #pragma DATA_SECTION(result,"Cla1ToCpuMsgRAM")
    float result;
    #pragma DATA_SECTION(init,"CpuToCla1MsgRAM")
    float init;
#endif

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
		GpioDataRegs.GPADAT.bit.GPIO31 = 1;
    System_printf("ADC res: %d\n", (int16_t) sensorSample);
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
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
    GPIO_GroupInit();

//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;


//
// Map ISR functions
//
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
    EDIS;

//
// Configure the ePWM
//
ADC_GroupInit();
EPWM_GroupInit();

Interrupt_Init();





//
// sync ePWM
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

//
// start ePWM
//
    EPwm1Regs.ETSEL.bit.SOCAEN = 1;  //enable SOCA
    EPwm1Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;
//
// take conversions indefinitely in loop
//
    BIOS_start();
}

//
// adca1_isr - Read Temperature ISR
//
interrupt void adca1_isr(void)
{
    sensorSample = AdcaResultRegs.ADCRESULT0;
    sensorTemp = GetTemperatureC(sensorSample);

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// End of file
//
