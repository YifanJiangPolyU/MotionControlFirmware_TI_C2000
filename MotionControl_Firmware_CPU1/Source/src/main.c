/*
 *  ======== main.c ========
 */
#include "F28x_Project.h"
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

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
    System_printf("enter taskFxn()\n");
    Task_sleep(500);
		GpioDataRegs.GPADAT.bit.GPIO31 = 0;
    Task_sleep(500);
    System_printf("exit taskFxn(): %d\n", i);
    i += 1;
  }
}

/*
 *  ======== main ========
 */
Int main()
{

  InitSysCtrl();

  // configure memory ownership
  MemCfgRegs.GSxMSEL.bit.MSEL_GS0 = 0;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS1 = 0;


  InitGpio();
	GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
  GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);

    /*
     * use ROV->SysMin to view the characters in the circular buffer
     */
    System_printf("enter main()\n");

    BIOS_start();    /* does not return */
    return(0);
}
