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

  // temporarily disable watch dog
  DisableDog();

  // initialize memory
  SystemMemoryInit();

#ifdef CPU1
  // set state of unused GPIOs
  // only CPI1 need to do this
  GPIO_EnableUnbondedIOPullups();

  // re-init system clock, overriding SYS/BIOS setting
  // only CPI1 need to do this
  InitSysPll(XTAL_OSC,IMULT_40,FMULT_0,PLLCLK_BY_2);
#endif

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
  Interrupt_Init();

  EINT;  // Enable Global interrupt INTM
  ERTM;  // Enable Global realtime interrupt DBGM

}

/**
 *  Memory related initialization
*/
void SystemMemoryInit(void){
/*
  #ifdef _FLASH
      // copy time critical functions from FLASH to RAM
      memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t)&RamfuncsLoadSize);
      InitFlash();
  #endif
*/
#ifdef CPU1
  // configure memory ownership
  EALLOW;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS0 = 0; // RAMGS0~7 belongs to CPU1
  MemCfgRegs.GSxMSEL.bit.MSEL_GS1 = 0;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS2 = 0;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS3 = 0;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS4 = 0;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS5 = 0;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS6 = 0;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS7 = 0;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS8 = 1; // RAMGS8~15 belongs to CPU2
  MemCfgRegs.GSxMSEL.bit.MSEL_GS9 = 1;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS10 = 1;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS11 = 1;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS12 = 1;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS13 = 1;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS14 = 1;
  MemCfgRegs.GSxMSEL.bit.MSEL_GS15 = 1;
  EDIS;
#endif
}

/**
 *  Check if analog subsystem trim registers are properly set
 *  CPU1 does this
*/
#ifdef CPU1
void SystemCheckTriming(void){
  EALLOW;
  // enable ADC power to do this
  CpuSysRegs.PCLKCR13.bit.ADC_A = 1;
  CpuSysRegs.PCLKCR13.bit.ADC_B = 1;
  CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
  CpuSysRegs.PCLKCR13.bit.ADC_D = 1;

  // Check if device is trimmed
  if(*((Uint16 *)0x5D1B6) == 0x0000){
      // Device is not trimmed--apply static calibration values
      AnalogSubsysRegs.ANAREFTRIMA.all = 31709;
      AnalogSubsysRegs.ANAREFTRIMB.all = 31709;
      AnalogSubsysRegs.ANAREFTRIMC.all = 31709;
      AnalogSubsysRegs.ANAREFTRIMD.all = 31709;
  }

  CpuSysRegs.PCLKCR13.bit.ADC_A = 0;
  CpuSysRegs.PCLKCR13.bit.ADC_B = 0;
  CpuSysRegs.PCLKCR13.bit.ADC_C = 0;
  CpuSysRegs.PCLKCR13.bit.ADC_D = 0;
  EDIS;
}
#endif

/**
 *  initialize all GPIOs in this function
*/
void GPIO_GroupInit(void){
  InitGpio();
  // GPIOs to control LEDs
  GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
  GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
  GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
  GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);

  // UART Rx Tx pins
  GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1);
  GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL); // Rx
  GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1);
  GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC); // Tx

}

/**
 *  initialize all interrupt (PIE) settings here
*/
void Interrupt_Init(void){
  EALLOW;

  // clear cpu side interrupt flags
  IFR = 0x0000;

  // enable interrupts on PIE side
  // PieCtrlRegs.PIEIER1.bit.INTx1 = 1;  // INT1.1, ADCA1
  PieCtrlRegs.PIEIER11.all = 0xFFFF;


  // enable interrupts on CPU side
  //IER |= M_INT1; //Enable group 1 interrupts
  IER |= (M_INT11 ); // CLA end-of-task interrupt

  EINT;  // Enable Global interrupt INTM
  ERTM;  // Enable Global realtime interrupt DBGM

  EDIS;
}

/**
 * initialize UART interface for debug purpose
 */
void UART_Init(void){

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
  AdcaRegs.ADCSOC0CTL.bit.CHSEL = 4;  //SOC0 will convert pin A4
  AdcaRegs.ADCSOC0CTL.bit.ACQPS = 63; //sample window is 64 SYSCLK cycles
  AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of EOC0 will set INT1 flag
  AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
  AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5; // trigger source: EPWM1, ADCSOCA

  AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4;  //SOC0 will convert pin B4
  AdcbRegs.ADCSOC0CTL.bit.ACQPS = 63;
  AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 1;
  AdcbRegs.ADCINTSEL1N2.bit.INT1E = 0;
  AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
  AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

  // if interrupt is used, flag rises when conversion completes
  AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
  AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;

  DELAY_US(1000);

  EDIS;
}

/**
 *  initialize all EPWM modules here
*/
void EPWM_GroupInit(void){
  EALLOW;
  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;

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

  // EPWM1, up-counting, 320kHz, trigger ADC
  EPwm1Regs.CMPA.bit.CMPA = 4095;     // Set compare A value to 624 counts
  EPwm1Regs.TBPRD = 4096;             // Set period to 4096x4 counts
  EPwm1Regs.TBPHS.all = 0;
  EPwm1Regs.TBCTL.bit.PHSEN = 1;     // enable synchronization
  EPwm1Regs.TBCTL.bit.SYNCOSEL = 0;  // enable software forced sync
  EPwm1Regs.TBCTL.bit.CTRMODE = 3;   // freeze counter

  // EPWM4, up-down, 32 kHz
  EPwm4Regs.CMPA.bit.CMPA = 100;
  EPwm4Regs.TBPRD = 3125;
  EPwm4Regs.TBPHS.all = 3124;
  EPwm4Regs.TBCTL.bit.PHSEN = 1;
  EPwm4Regs.TBCTL.bit.SYNCOSEL = 0;
  EPwm4Regs.TBCTL.bit.CTRMODE = 3;
  EPwm4Regs.TBCTL.bit.PHSDIR = 1;    // count up after synchronization

  // set up EPWM4 sync source to EPWM1SYNCOUT
  // EPWM 5~6 sync signals are connected to EPWM4 internally
  SyncSocRegs.SYNCSELECT.bit.EPWM4SYNCIN = 0;

  // EPWM5, up-down, 32 kHz
  EPwm5Regs.CMPA.bit.CMPA = 100;
  EPwm5Regs.TBPRD = 3125;
  EPwm5Regs.TBPHS.all = 3124;
  EPwm5Regs.TBCTL.bit.PHSEN = 1;
  EPwm5Regs.TBCTL.bit.SYNCOSEL = 0;
  EPwm5Regs.TBCTL.bit.CTRMODE = 3;
  EPwm5Regs.TBCTL.bit.PHSDIR = 1;

  // EPWM6, up-down, 32 kHz
  EPwm6Regs.CMPA.bit.CMPA = 100;
  EPwm6Regs.TBPRD = 3125;
  EPwm6Regs.TBPHS.all = 3124;
  EPwm6Regs.TBCTL.bit.PHSEN = 1;
  EPwm6Regs.TBCTL.bit.SYNCOSEL = 0;
  EPwm6Regs.TBCTL.bit.CTRMODE = 3;
  EPwm6Regs.TBCTL.bit.PHSDIR = 1;

  // EPWM1: event trigger for ADC conversion
  EPwm1Regs.ETSEL.bit.SOCAEN = 0;    // Disable SOC on A group
  EPwm1Regs.ETSEL.bit.SOCASEL = 4;   // trigger when TBCTR = CMPA
  EPwm1Regs.ETPS.bit.SOCAPRD = 1;    // Generate pulse on every event

  // EPWM4: trigger control process master
  //        and position loop
  EPwm4Regs.ETSEL.bit.INTEN = 1;   // enable interrupt
  EPwm4Regs.ETSEL.bit.INTSEL = 4;
  EPwm4Regs.ETPS.bit.INTPRD = 1;

  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
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
  MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
  MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 1;

  // LS1 as CLA data memory
  MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
  MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 0;

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
  Cla1Regs.MIER.all = M_INT1;

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

  // trigger source
  DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK1 = 1;

  // Enable CLA interrupts at the group and subgroup levels
  PieCtrlRegs.PIEIER11.all = 0xFFFF;
  IER |= (M_INT11 );

  EDIS;
}
