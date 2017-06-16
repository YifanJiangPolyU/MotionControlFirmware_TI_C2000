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

    // ADC data buffer
    #pragma DATA_SECTION(CLA_SampleBufferA,"CLADataLS1")
    uint16_t CLA_SampleBufferA[CLA_SAMPLE_BUFFER_LEN_X2];
    #pragma DATA_SECTION(CLA_SampleBufferB,"CLADataLS1")
    uint16_t CLA_SampleBufferB[CLA_SAMPLE_BUFFER_LEN_X2];
    #pragma DATA_SECTION(CLA_SampleBufferActiveHalf,"CLADataLS1")
    uint16_t CLA_SampleBufferActiveHalf;

    // Current controller parameters
    #pragma DATA_SECTION(CL_Kp,"CLADataLS1")
    float32_t CL_Kp;
    #pragma DATA_SECTION(CL_Ki,"CLADataLS1")
    float32_t CL_Ki;
    #pragma DATA_SECTION(CL_Setpoint_Ia,"CLADataLS1")
    float32_t CL_Setpoint_Ia;
    #pragma DATA_SECTION(CL_Setpoint_Ib,"CLADataLS1")
    float32_t CL_Setpoint_Ib;
    #pragma DATA_SECTION(CL_AdcScalingFactor, "CLADataLS1")
    float32_t CL_AdcScalingFactor;
    #pragma DATA_SECTION(CL_Error_Ia,"CLADataLS1")
    float32_t CL_Error_Ia;
    #pragma DATA_SECTION(CL_Error_Ib,"CLADataLS1")
    float32_t CL_Error_Ib;
    #pragma DATA_SECTION(CL_Integral_Ia,"CLADataLS1")
    float32_t CL_Integral_Ia;
    #pragma DATA_SECTION(CL_Integral_Ib,"CLADataLS1")
    float32_t CL_Integral_Ib;
    #pragma DATA_SECTION(CL_OutputLimit,"CLADataLS1")
    float32_t CL_OutputLimit;
    #pragma DATA_SECTION(CL_Output_Ua,"CLADataLS1")
    float32_t CL_Output_Ua;
    #pragma DATA_SECTION(CL_Output_Ub,"CLADataLS1")
    float32_t CL_Output_Ub;
    #pragma DATA_SECTION(CL_Output_Uc,"CLADataLS1")
    float32_t CL_Output_Uc;
    #pragma DATA_SECTION(CL_OutputOffset, "CLADataLS1")
    float32_t CL_OutputOffset;
    #pragma DATA_SECTION(CL_OutputPWM_Ua,"CLADataLS1")
    uint16_t CL_OutputPWM_Ua;
    #pragma DATA_SECTION(CL_OutputPWM_Ub,"CLADataLS1")
    uint16_t CL_OutputPWM_Ub;
    #pragma DATA_SECTION(CL_OutputPWM_Uc,"CLADataLS1")
    uint16_t CL_OutputPWM_Uc;

    #pragma DATA_SECTION(CommutationAngle_Cos,"CLADataLS1")
    float32_t CommutationAngle_Cos;
    #pragma DATA_SECTION(CommutationAngle_Sin,"CLADataLS1")
    float32_t CommutationAngle_Sin;

    #pragma DATA_SECTION(PL_ActualPosition,"CLADataLS1")
    int32_t PL_ActualPosition;
    #pragma DATA_SECTION(PL_Setpoint_Pos,"CLADataLS1")
    int32_t PL_Setpoint_Pos;
    #pragma DATA_SECTION(PL_Setpoint_Vel,"CLADataLS1")
    float32_t PL_Setpoint_Vel;
    #pragma DATA_SECTION(PL_Setpoint_Accel,"CLADataLS1")
    float32_t PL_Setpoint_Accel;
    #pragma DATA_SECTION(PL_PosError,"CLADataLS1")
    float32_t PL_PosError;
    #pragma DATA_SECTION(PL_PosIntegral,"CLADataLS1")
    float32_t PL_PosIntegral;
    #pragma DATA_SECTION(PL_Kp1,"CLADataLS1")
    float32_t PL_Kp1;
    #pragma DATA_SECTION(PL_Ki1,"CLADataLS1")
    float32_t PL_Ki1;
    #pragma DATA_SECTION(PL_Kd1,"CLADataLS1")
    float32_t PL_Kd1;
    #pragma DATA_SECTION(PL_Kp2,"CLADataLS1")
    float32_t PL_Kp2;
    #pragma DATA_SECTION(PL_Ki2,"CLADataLS1")
    float32_t PL_Ki2;
    #pragma DATA_SECTION(PL_Kd2,"CLADataLS1")
    float32_t PL_Kd2;
    #pragma DATA_SECTION(PL_Kp3,"CLADataLS1")
    float32_t PL_Kp3;
    #pragma DATA_SECTION(PL_Ki3,"CLADataLS1")
    float32_t PL_Ki3;
    #pragma DATA_SECTION(PL_Kd3,"CLADataLS1")
    float32_t PL_Kd3;
    #pragma DATA_SECTION(PL_FF_Vel,"CLADataLS1")
    float32_t PL_FF_Vel;
    #pragma DATA_SECTION(PL_FF_Accel,"CLADataLS1")
    float32_t PL_FF_Accel;
    #pragma DATA_SECTION(PL_OutputRaw,"CLADataLS1")
    float32_t PL_OutputRaw;
    #pragma DATA_SECTION(PL_OutputLimit,"CLADataLS1")
    float32_t PL_OutputLimit;
    #pragma DATA_SECTION(PL_OutputCurrent_D,"CLADataLS1")
    float32_t PL_OutputCurrent_D;
    #pragma DATA_SECTION(PL_OutputCurrent_Q,"CLADataLS1")
    float32_t PL_OutputCurrent_Q;
    #pragma DATA_SECTION(PL_OutputCurrent_Alpha,"CLADataLS1")
    float32_t PL_OutputCurrent_Alpha;
    #pragma DATA_SECTION(PL_OutputCurrent_Beta,"CLADataLS1")
    float32_t PL_OutputCurrent_Beta;

    #pragma DATA_SECTION(timeCounter,"CLADataLS1")
    uint16_t timeCounter;

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

void CLA_Reset(){
  CLA_SampleCounter = 0;
  CLA_CycleCounter = 0;
  CLA_PosLoopCounter = 0;

  CLA_CurrentLoopEnable = 0;
  CLA_PositionLoopEnable = 0;

  CL_Integral_Ia = 0;
  CL_Integral_Ib = 0;
  PL_PosIntegral = 0;
}
