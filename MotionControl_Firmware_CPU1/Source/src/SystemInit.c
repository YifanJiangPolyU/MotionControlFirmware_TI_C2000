/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* contains functions used to initialize the WHOLE system, including BOTH CPU1
* and CPU2.
*/

#include "SystemInit.h"
#include "F28x_Project.h"
#include "CPU1_CLA1_common.h"

void SystemFullInit(void){

  // TI specific system initialization
  InitSysCtrl();

  DINT;

  // enable CLAs
  EALLOW;
  DevCfgRegs.DC1.bit.CPU1_CLA1 = 1;
  EDIS;

  // initialize CLA
  CLA_ConfigClaMemory();
  CLA_InitCpu1Cla1();

  // initialize peripherals
  GPIO_GroupInit();
  ADC_GroupInit();
  EPWM_GroupInit();

  // initialize all interrupt settings here
  PIE_Init();

  EINT;  // Enable Global interrupt INTM
  ERTM;  // Enable Global realtime interrupt DBGM

}

/**
 *  initialize all GPIOs in this function
*/
void GPIO_GroupInit(void){
  InitGpio();
  GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
  GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
  GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
  GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
}

/**
 *  initialize all interrupt (PIE) settings here
*/
void PIE_Init(void){
  InitPieCtrl();
  IER = 0x0000;
  IFR = 0x0000;
  InitPieVectTable();
}

/**
 *  initialize all ADCs here
 *  use ADC A&B to sample 2 phase currents
 *  third phase current adds up to zero, no need to measure.
*/
void ADC_GroupInit(void){
  EALLOW;

  // enable ADC clock
  CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
  CpuSysRegs.PCLKCR13.bit.ADC_B = 1;

  //set ADCCLK divider to /4, ADCCLK = 50MHz
  AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
  AdcbRegs.ADCCTL2.bit.PRESCALE = 6;

  // set 12-bit resolution and single ended input
  AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
  AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

  // power up ADC
  AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
  AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

  // conversion time and trigger
  AdcaRegs.ADCSOC0CTL.bit.CHSEL = 4;  //SOC0 will convert pin A0
  AdcaRegs.ADCSOC0CTL.bit.ACQPS = 19; //sample window is 20 SYSCLK cycles
  AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //end of SOC1 will set INT1 flag
  AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

  AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4;
  AdcbRegs.ADCSOC0CTL.bit.ACQPS = 19;
  AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1;
  AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1;
  AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

  EDIS;
}

/**
 *  initialize all EPWM modules here
*/
void EPWM_GroupInit(void){
  EALLOW;

  // EPWM modules assigned to CPU1
  DevCfgRegs.CPUSEL0.bit.EPWM1 = 0;
  DevCfgRegs.CPUSEL0.bit.EPWM4 = 0;
  DevCfgRegs.CPUSEL0.bit.EPWM5 = 0;
  DevCfgRegs.CPUSEL0.bit.EPWM6 = 0;

  // enable EPWM clock
  CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
  CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;
  CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;
  CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;

  // EPWM1, up-counting, 720 kHz, trigger ADC

  // EPWM4, up-down, 24 kHz

  // EPWM5, up-down, 24 kHz

  // EPWM6, up-down, 24 kHz

  EDIS;
}

/**
 *  configure memory for CPU1_CLA1
 *    L0 as program RAM
 *    L1 as data RAM
*/
void CLA_ConfigClaMemory(void){
  extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
  EALLOW;

#ifdef _FLASH
  // copy CLA code from flash to RAM
  memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
  (uint32_t)&Cla1funcsLoadSize);
#endif //_FLASH

  // enable CLA clock
  CpuSysRegs.PCLKCR0.bit.CLA1 = 1;

  // Initialize and wait for CLA1ToCPUMsgRAM
  MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
  while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1){};

  // Initialize and wait for CPUToCLA1MsgRAM
  MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
  while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1){};

  // LS0 as CLA program memory
  MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1;
  MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 1;

  // LS1 as CLA data memory
  MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
  MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 0;

  EDIS;
}


/**
 *  initialize tasks for CPU1_CLA1
*/
void CLA_InitCpu1Cla1(void){
  EALLOW;
  // enable CLA clock
  CpuSysRegs.PCLKCR0.bit.CLA1 = 1;

  // Compute all CLA task vectors
  Cla1Regs.MVECT1 = (uint16_t)(&Cla1Task1);
  Cla1Regs.MVECT2 = (uint16_t)(&Cla1Task2);
  Cla1Regs.MVECT3 = (uint16_t)(&Cla1Task3);
  Cla1Regs.MVECT4 = (uint16_t)(&Cla1Task4);
  Cla1Regs.MVECT5 = (uint16_t)(&Cla1Task5);
  Cla1Regs.MVECT6 = (uint16_t)(&Cla1Task6);
  Cla1Regs.MVECT7 = (uint16_t)(&Cla1Task7);
  Cla1Regs.MVECT8 = (uint16_t)(&Cla1Task8);

  // Enable the IACK instruction to start a task on CLA in software
  // for all  8 CLA tasks.
  Cla1Regs.MCTL.bit.IACKE = 1;
  Cla1Regs.MIER.all = 0x00FF;

  // Configure the vectors for the end-of-task interrupt for all
  // 8 tasks
  PieVectTable.CLA1_1_INT = &cla1Isr1;
  PieVectTable.CLA1_2_INT = &cla1Isr2;
  PieVectTable.CLA1_3_INT = &cla1Isr3;
  PieVectTable.CLA1_4_INT = &cla1Isr4;
  PieVectTable.CLA1_5_INT = &cla1Isr5;
  PieVectTable.CLA1_6_INT = &cla1Isr6;
  PieVectTable.CLA1_7_INT = &cla1Isr7;
  PieVectTable.CLA1_8_INT = &cla1Isr8;

  // Enable CLA interrupts at the group and subgroup levels
  PieCtrlRegs.PIEIER11.all = 0xFFFF;
  IER |= (M_INT11 );

  EDIS;
}
