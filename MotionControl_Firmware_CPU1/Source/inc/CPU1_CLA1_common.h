/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* declare shared variables between CPU1 and CPU1_CLA1,
* CLA interrupt handlers (for CPU)
* and CLA task functions
*/

#ifndef CPU1_CLA1_COMMON_H
#define CPU1_CLA1_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include "F28x_Project.h"

#define CLA_SAMPLE_BUFFER_LEN_X2 20

#define DEBUG_CODE_PROFILING

#ifdef DEBUG_CODE_PROFILING
#define READ_CLOCK(X) __meallow();\
                      EPwm2Regs.TBCTL.bit.CTRMODE = TB_FREEZE;\
                      X = EPwm1Regs.TBCTR;\
                      __medis();
#define RESTART_CLOCK __meallow();\
                      EPwm2Regs.TBCTL.bit.CTRMODE = TB_FREEZE;\
                      EPwm2Regs.TBCTR = 0;\
                      EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;\
                      __medis();
#endif



// shared variables between CPU and CLA
extern float init;
extern uint16_t result;

extern uint16_t CLA_SampleCounter;
extern uint16_t CLA_CycleCounter;
extern uint16_t CLA_PosLoopCounter;
extern uint16_t CLA_CurrentLoopEnable;               // enable current loop
extern uint16_t CLA_PositionLoopEnable;              // enable position loop

extern uint16_t sensorSampleA;
extern uint16_t sensorSampleB;
extern uint16_t dcVoltageSense;
extern uint16_t CLA_SampleBufferA[CLA_SAMPLE_BUFFER_LEN_X2];    // ADC data buffer, phase A
extern uint16_t CLA_SampleBufferB[CLA_SAMPLE_BUFFER_LEN_X2];    // ADC data buffer, phase B
extern uint16_t CLA_SampleBufferActiveHalf;                     // double buffering active half

extern uint16_t timeCounter;



// CLA C Tasks
__interrupt void Cla1Task1();
__interrupt void Cla1Task2();
__interrupt void Cla1Task3();
__interrupt void Cla1Task4();
__interrupt void Cla1Task5();
__interrupt void Cla1Task6();
__interrupt void Cla1Task7();
__interrupt void Cla1Task8();

void CLA_CurrentLoop();
void CLA_PositionLoop();
void CLA_Reset();


#ifdef __cplusplus
}
#endif // extern "C"

#endif
