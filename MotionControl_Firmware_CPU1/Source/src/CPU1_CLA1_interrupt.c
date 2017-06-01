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
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>


#ifdef __cplusplus
    #pragma DATA_SECTION("CLADataLS1")
    uint16_t sensorSampleA;
    #pragma DATA_SECTION("CLADataLS1")  // count from 0~19
    uint16_t CLA_SampleCounter;
    #pragma DATA_SECTION("CLADataLS1")  // count from 0~9
    uint16_t CLA_CycleCounter;
    #pragma DATA_SECTION("CLADataLS1")
    uint16_t CLA_SampleBufferA[CLA_SAMPLE_BUFFER_LEN_X2]; // ADC data buffer, phase A
    #pragma DATA_SECTION("CLADataLS1")
    uint16_t CLA_SampleBufferB[CLA_SAMPLE_BUFFER_LEN_X2]; // ADC data buffer, phase B
    #pragma DATA_SECTION("CLADataLS1")
    uint16_t CLA_SampleBufferActiveHalf;  // which half of the CLA_SampleBuffer can be read?
    #pragma DATA_SECTION("CLADataLS1")
    uint16_t timeCounter;
#else
    #pragma DATA_SECTION(sensorSampleA,"CLADataLS1")
    uint16_t sensorSampleA;
    #pragma DATA_SECTION(CLA_SampleCounter,"CLADataLS1")
    uint16_t CLA_SampleCounter;
    #pragma DATA_SECTION(CLA_CycleCounter,"CLADataLS1")
    uint16_t CLA_CycleCounter;
    #pragma DATA_SECTION(CLA_SampleBufferA,"CLADataLS1") // ADC data buffer, phase A
    uint16_t CLA_SampleBufferA[CLA_SAMPLE_BUFFER_LEN_X2];
    #pragma DATA_SECTION(CLA_SampleBufferB,"CLADataLS1") // ADC data buffer, phase B
    uint16_t CLA_SampleBufferB[CLA_SAMPLE_BUFFER_LEN_X2];
    #pragma DATA_SECTION(CLA_SampleBufferActiveHalf,"CLADataLS1")
    uint16_t CLA_SampleBufferActiveHalf;
    #pragma DATA_SECTION(timeCounter,"CLADataLS1")
    uint16_t timeCounter;

#endif

#ifdef __cplusplus
    #pragma DATA_SECTION("Cla1ToCpuMsgRAM")
    uint16_t result;
    #pragma DATA_SECTION("CpuToCla1MsgRAM");
    float init;
#else
    #pragma DATA_SECTION(result,"Cla1ToCpuMsgRAM")
    uint16_t result;
    #pragma DATA_SECTION(init,"CpuToCla1MsgRAM")
    float init;
#endif


extern Mailbox_Handle ADC_fifo;

//
// cla1Isr1 - CLA1 ISR 1
//
__interrupt void cla1Isr1 ()
{
  // Acknowledge the end-of-task interrupt for task 1
  PieCtrlRegs.PIEACK.all = (PIEACK_GROUP1 | PIEACK_GROUP11);
  Mailbox_post(ADC_fifo, &(CLA_SampleBufferA[CLA_SampleCounter]), BIOS_NO_WAIT);
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
