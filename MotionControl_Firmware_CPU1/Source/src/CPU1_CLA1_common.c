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


/**
 *  Parameters and data shared between CPU and CLA
 */
    // process control
    #pragma DATA_SECTION(CLA_SampleCounter,"CLADataLS1")
    uint16_t CLA_SampleCounter;
    #pragma DATA_SECTION(CLA_CycleCounter,"CLADataLS1")
    uint16_t CLA_CycleCounter;
    #pragma DATA_SECTION(CLA_PosLoopCounter,"CLADataLS1")
    uint16_t CLA_PosLoopCounter;
    #pragma DATA_SECTION(CLA_CurrentLoopEnable,"CLADataLS1")
    uint16_t CLA_CurrentLoopEnable;
    #pragma DATA_SECTION(CLA_PositionLoopEnable,"CLADataLS1")
    uint16_t CLA_PositionLoopEnable;

    #pragma DATA_SECTION(sensorSampleA,"CLADataLS1")
    uint16_t sensorSampleA;
    #pragma DATA_SECTION(sensorSampleB,"CLADataLS1")
    uint16_t sensorSampleB;
    #pragma DATA_SECTION(dcVoltageSense,"CLADataLS1")
    uint16_t dcVoltageSense;

    // ADC data buffer
    #pragma DATA_SECTION(CLA_SampleBufferA,"CLADataLS1")
    uint16_t CLA_SampleBufferA[CLA_SAMPLE_BUFFER_LEN_X2];
    #pragma DATA_SECTION(CLA_SampleBufferB,"CLADataLS1")
    uint16_t CLA_SampleBufferB[CLA_SAMPLE_BUFFER_LEN_X2];
    #pragma DATA_SECTION(CLA_SampleBufferActiveHalf,"CLADataLS1")
    uint16_t CLA_SampleBufferActiveHalf;


    #pragma DATA_SECTION(timeCounter,"CLADataLS1")
    uint16_t timeCounter;

void CLA_Reset(){
  CLA_SampleCounter = 0;
  CLA_CycleCounter = 0;
  CLA_PosLoopCounter = 0;
}
