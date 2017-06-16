/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/**
 * implement all user-define ISR here
 */

 #ifndef ISR_ABSTRACTION_H
 #define ISR_ABSTRACTION_H

 #include "stdint.h"
 #include "F28x_Project.h"

 /**
  *   C API to call C++ member functions
  */
void CallControlProcessMaster(void);
void CallUartDriverExecuteParsing(void);

// SCIA Rx fifo full interrupt handler
 __interrupt void ISR_SciaRx();


 // CLA interrupt handlers
 __interrupt void cla1Isr1();

 #endif
