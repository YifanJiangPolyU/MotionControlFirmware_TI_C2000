/******************************************************************************
* Copyright (C) 2017 by Yifan Jiang                                          *
* jiangyi@student.ethz.com                                                   *
*                                                                            *
* This program is distributed in the hope that it will be useful,            *
* but WITHOUT ANY WARRANTY; without even the implied warranty of             *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
******************************************************************************/

/*
* implements CLA interrupt handlers
*/


#include "stdint.h"
#include "CPU1_CLA1_common.h"
#include "F28x_Project.h"


//
// cla1Isr1 - CLA1 ISR 1
//
__interrupt void cla1Isr1 ()
{
  //
  // Acknowledge the end-of-task interrupt for task 1
  //
  PieCtrlRegs.PIEACK.all = M_INT11;
  //asm(" ESTOP0");
}

//
// cla1Isr2 - CLA1 ISR 2
//
__interrupt void cla1Isr2 ()
{
  PieCtrlRegs.PIEACK.all = M_INT11;
  //asm(" ESTOP0");
}

//
// cla1Isr3 - CLA1 ISR 3
//
__interrupt void cla1Isr3 ()
{
  PieCtrlRegs.PIEACK.all = M_INT11;
  //asm(" ESTOP0");
}

//
// cla1Isr4 - CLA1 ISR 4
//
__interrupt void cla1Isr4 ()
{
  PieCtrlRegs.PIEACK.all = M_INT11;
  //asm(" ESTOP0");
}

//
// cla1Isr5 - CLA1 ISR 5
//
__interrupt void cla1Isr5 ()
{
  PieCtrlRegs.PIEACK.all = M_INT11;
  //asm(" ESTOP0");
}

//
// cla1Isr6 - CLA1 ISR 6
//
__interrupt void cla1Isr6 ()
{
  PieCtrlRegs.PIEACK.all = M_INT11;
  //asm(" ESTOP0");
}

//
// cla1Isr7 - CLA1 ISR 7
//
__interrupt void cla1Isr7 ()
{
  PieCtrlRegs.PIEACK.all = M_INT11;
  //asm(" ESTOP0");
}

//
// cla1Isr8 - CLA1 ISR 8
//
__interrupt void cla1Isr8 ()
{
  //
  // Acknowledge the end-of-task interrupt for task 8
  //
  PieCtrlRegs.PIEACK.all = M_INT11;
  //asm(" ESTOP0");
}
