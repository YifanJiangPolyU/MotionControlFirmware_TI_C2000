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

 #include "IsrAbstraction.h"

 __interrupt void ISR_SciaRx()
 {
   SciaRegs.SCIFFRX.bit.RXFFINTCLR = 1;
   PieCtrlRegs.PIEACK.all = PIEACK_GROUP9;
   CallUartDriverExecuteParsing();
 }


 // cla1Isr1 - CLA1 ISR 1
 __interrupt void cla1Isr1 ()
 {
   // Acknowledge the end-of-task interrupt for task 1
   PieCtrlRegs.PIEACK.all = (PIEACK_GROUP1 | PIEACK_GROUP11);

   CallControlProcessMaster();
 }
