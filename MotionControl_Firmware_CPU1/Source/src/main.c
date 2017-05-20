
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include "F28x_Project.h"
#include "SystemInit.h"

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

  SystemFullInit();

  // use ROV->SysMin to view the characters in the circular buffer
  System_printf("enter main()\n");

  // start scheduler, does not return
  BIOS_start();
  return(0);
}
