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
extern uint16_t CLA_SampleBufferA[CLA_SAMPLE_BUFFER_LEN_X2];    // ADC data buffer, phase A
extern uint16_t CLA_SampleBufferB[CLA_SAMPLE_BUFFER_LEN_X2];    // ADC data buffer, phase B
extern uint16_t CLA_SampleBufferActiveHalf;                     // double buffering active half

// current loop parameters
extern float32_t CL_Kp;                     // P gain
extern float32_t CL_Ki;                     // I gain
extern float32_t CL_Setpoint_Ia;            // current requirement (ADC raw), phase A
extern float32_t CL_Setpoint_Ib;            // current requirement (ADC raw), phase B
extern float32_t CL_OutputLimit;            // limit the output
extern float32_t CL_AdcScalingFactor;       // convert from ADC cnt to mA

extern float32_t CL_Error_Ia;               // current error, phase A
extern float32_t CL_Error_Ib;               // current error, phase B
extern float32_t CL_Integral_Ia;            // integral term, phase A
extern float32_t CL_Integral_Ib;            // integral term, phase B
extern float32_t CL_Output_Ua;               // output, phase A
extern float32_t CL_Output_Ub;               // output, phase B
extern float32_t CL_Output_Uc;               // output, phase C
extern float32_t CL_OutputOffset;            // offset required to give positive PWM duty
extern uint16_t CL_OutputPWM_Ua;             // output PWM duty, phase A
extern uint16_t CL_OutputPWM_Ub;             // output PWM duty, phase B
extern uint16_t CL_OutputPWM_Uc;             // output PWM duty, phase C

// commutation parameters
extern float32_t CommutationAngle_Cos;        // commutation angle cosine
extern float32_t CommutationAngle_Sin;        // commutation angle sine

// position loop parameters
extern int32_t   PL_ActualPosition;           // actual encoder position (cnt)
extern int32_t   PL_Setpoint_Pos;             // target position (cnt)
extern float32_t PL_Setpoint_Vel;             // profile velocity (cnt/sp), for feed-forward only
extern float32_t PL_Setpoint_Accel;           // profile acceleration (cnt/sp^2), for feed-forward only
extern float32_t PL_PosError;                 // position error
extern float32_t PL_PosIntegral;              // position error integral term
extern float32_t PL_Kp1;                      // position loop PID gains group 1
extern float32_t PL_Ki1;
extern float32_t PL_Kd1;
extern float32_t PL_Kp2;                      // position loop PID gains group 2
extern float32_t PL_Ki2;
extern float32_t PL_Kd2;
extern float32_t PL_Kp3;                      // position loop PID gains group 3
extern float32_t PL_Ki3;
extern float32_t PL_Kd3;
extern float32_t PL_FF_Vel;                   // velocity feed-forward gain
extern float32_t PL_FF_Accel;                 // acceleration feed-forward gain
extern float32_t PL_OutputRaw;                // position control raw output
extern float32_t PL_OutputLimit;              // position control upper bound
extern float32_t PL_OutputCurrent_D;          // cmd Id
extern float32_t PL_OutputCurrent_Q;          // cmd Iq
extern float32_t PL_OutputCurrent_Alpha;      // cmd I alpha
extern float32_t PL_OutputCurrent_Beta;       // cmd I beta

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
