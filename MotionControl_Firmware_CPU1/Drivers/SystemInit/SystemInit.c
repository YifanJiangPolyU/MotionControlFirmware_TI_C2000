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

  // BoostXL DRV8301 gate enable
  GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
  GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
  GpioDataRegs.GPADAT.bit.GPIO26 = 1;

  // UART Rx Tx pins
  GPIO_SetupPinMux(43, GPIO_MUX_CPU1, 15);
  GPIO_SetupPinOptions(43, GPIO_INPUT, GPIO_PUSHPULL); // Rx
  GPIO_SetupPinMux(42, GPIO_MUX_CPU1, 15);
  GPIO_SetupPinOptions(42, GPIO_OUTPUT, GPIO_ASYNC); // Tx

  // PWM output pins
  GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 1);
  GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_ASYNC);
  GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 1);
  GPIO_SetupPinOptions(7, GPIO_OUTPUT, GPIO_ASYNC);
  GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 1);
  GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_ASYNC);
  GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 1);
  GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_ASYNC);
  GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 1);
  GPIO_SetupPinOptions(10, GPIO_OUTPUT, GPIO_ASYNC);
  GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 1);
  GPIO_SetupPinOptions(11, GPIO_OUTPUT, GPIO_ASYNC);
  GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;

  // quadrature encoder interface pins (EQEP1)
  GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);               // A
  GPIO_SetupPinOptions(20, GPIO_INPUT, GPIO_SYNC);
  GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);               // B
  GPIO_SetupPinOptions(21, GPIO_INPUT, GPIO_SYNC);
  GPIO_SetupPinMux(99, GPIO_MUX_CPU1, 5);               // index
  GPIO_SetupPinOptions(99, GPIO_INPUT, GPIO_SYNC);
  GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;
  GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;
  GpioCtrlRegs.GPDPUD.bit.GPIO99 = 1;

}

/**
 *  initialize all interrupt (PIE) settings here
*/
void Interrupt_Init(void){
  EALLOW;

  // clear cpu side interrupt flags
  IFR = 0x0000;

  // enable interrupts on PIE side
  // PieCtrlRegs.PIEIER11 are CLA end-of-task interrupts
  PieCtrlRegs.PIEIER11.all = 0xFFFF;


  // enable interrupts on CPU side
  //IER |= M_INT11; //Enable group 11 interrupts
  IER |= (M_INT11 ); // CLA end-of-task interrupt

  EINT;  // Enable Global interrupt INTM
  ERTM;  // Enable Global realtime interrupt DBGM

  EDIS;
}

/**
 * initialize UART interface for debug purpose
 */
void UART_Init(void){

EALLOW;
  // set SCI clock (LSPCLK) to 100 MHz (sysclk/2)
  ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0b001;

  // enable SCIA clock
  CpuSysRegs.PCLKCR7.bit.SCI_A = 1;
EDIS;

  SciaRegs.SCICCR.all = 0x0000;
  SciaRegs.SCICCR.bit.PARITYENA = 0;  // disable parity
  SciaRegs.SCICCR.bit.STOPBITS = 0;   // 1 stop bit
  SciaRegs.SCICCR.bit.LOOPBKENA = 0;  // disable loop back
  SciaRegs.SCICCR.bit.SCICHAR = 7;    // 8-bit data
  SciaRegs.SCICCR.bit.ADDRIDLE_MODE = 0;  // idle-line mode (normal UART)

  SciaRegs.SCICTL1.all = 0x0000;
  SciaRegs.SCICTL1.bit.RXERRINTENA = 0;   // disable Rx error interrupt
  SciaRegs.SCICTL1.bit.TXENA = 1;         // enable Tx
  SciaRegs.SCICTL1.bit.RXENA = 1;         // enable Rx

  SciaRegs.SCICTL2.all = 0x0000;
  SciaRegs.SCICTL2.bit.TXINTENA = 0;      // disable Tx buffer empty interrupt
  SciaRegs.SCICTL2.bit.RXBKINTENA = 0;    // disable Rx break interrupt

  // baud rate: 2Mbps (2.0833Mpbs)
  // baud rate = LSPCLK/8/(SCIBAUD＋１)
  SciaRegs.SCIHBAUD.all = 0x0000;
  SciaRegs.SCILBAUD.all = 0x0005;

  SciaRegs.SCICTL1.bit.SWRESET = 1;

  // initialize these registers
  SciaRegs.SCIFFTX.all = 0xE040;
  SciaRegs.SCIFFRX.all = 0x2044;
  SciaRegs.SCIFFCT.all = 0x0;

  SciaRegs.SCIFFRX.bit.RXFFIENA = 1;    // enable SCI fifo interrupt
  SciaRegs.SCIFFRX.bit.RXFFIL = 0x10;    // generate fifo interrupt when it's full

  SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
  SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;

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
  CpuSysRegs.PCLKCR13.bit.ADC_C = 1;
  CpuSysRegs.PCLKCR13.bit.ADC_B = 1;

  //set ADCCLK divider to /4, ADCCLK = 50MHz
  AdcaRegs.ADCCTL2.bit.PRESCALE = 6;
  AdcbRegs.ADCCTL2.bit.PRESCALE = 6;
  AdccRegs.ADCCTL2.bit.PRESCALE = 6;


  // set 12-bit resolution and single ended input
  AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
  AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);
  AdcSetMode(ADC_ADCC, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

  // power up ADC
  AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
  AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;
  AdccRegs.ADCCTL1.bit.ADCPWDNZ = 1;

  // phase A current
  AdccRegs.ADCSOC0CTL.bit.CHSEL = 4;     //SOC0 will convert pin C4 (phase A)
  AdccRegs.ADCSOC0CTL.bit.ACQPS = 63;    //sample window is 64 SYSCLK cycles
  AdccRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of EOC0 will set INT1 flag
  AdccRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
  AdccRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
  AdccRegs.ADCSOC0CTL.bit.TRIGSEL = 5;   // trigger source: EPWM1, ADCSOCA

  // phase B current
  AdcbRegs.ADCSOC0CTL.bit.CHSEL = 4;  //SOC0 will convert pin B4 (phase B)
  AdcbRegs.ADCSOC0CTL.bit.ACQPS = 63;
  AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0;
  AdcbRegs.ADCINTSEL1N2.bit.INT1E = 0;
  AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
  AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

  // DC line voltage
  AdcaRegs.ADCSOC0CTL.bit.CHSEL = 15;  //SOC0 will convert pin ADCIN15
  AdcaRegs.ADCSOC0CTL.bit.ACQPS = 63;
  AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;
  AdcaRegs.ADCINTSEL1N2.bit.INT1E = 0;
  AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
  AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;

  // if interrupt is used, flag rises when conversion completes
  AdccRegs.ADCCTL1.bit.INTPULSEPOS = 1;
  AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
  AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

  // configure Post Processing Block (PPB) to sense over-current
  AdccRegs.ADCPPB1TRIPHI.bit.LIMITHI = 4095;
  AdccRegs.ADCPPB1TRIPLO.bit.LIMITLO = 0;
  AdcbRegs.ADCPPB1TRIPHI.bit.LIMITHI = 4095;
  AdcbRegs.ADCPPB1TRIPLO.bit.LIMITLO = 0;

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

  // EPWM runs at PLLCLK (200MHz)
  ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0;

  // enable EPWM clock
  CpuSysRegs.PCLKCR2.bit.EPWM1 = 1;
  CpuSysRegs.PCLKCR2.bit.EPWM4 = 1;
  CpuSysRegs.PCLKCR2.bit.EPWM5 = 1;
  CpuSysRegs.PCLKCR2.bit.EPWM6 = 1;

  // EPWM1, up-counting, 320kHz, trigger ADC
  EPwm1Regs.CMPA.bit.CMPA = 624;     // Set compare A value to 624 counts
  EPwm1Regs.TBPRD = 625;             // Set period to 625 counts
  EPwm1Regs.TBPHS.all = 0;
  EPwm1Regs.TBCTL.bit.PHSEN = 1;     // enable synchronization
  EPwm1Regs.TBCTL.bit.SYNCOSEL = 0;  // enable software forced sync
  EPwm1Regs.TBCTL.bit.CTRMODE = 3;   // freeze counter
  // ADC triggering setting
  EPwm1Regs.ETSEL.bit.SOCAEN = 0;    // Disable SOC on A group
  EPwm1Regs.ETSEL.bit.SOCASEL = 4;   // trigger when TBCTR = CMPA
  EPwm1Regs.ETPS.bit.SOCAPRD = 1;    // Generate pulse on every event
  // set up period EPWM clock (same as sysclk)
  EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT
  EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

  // EPWM4, up-down, 32 kHz
  EPwm4Regs.CMPA.bit.CMPA = 100;
  EPwm4Regs.TBPRD = 3125;
  EPwm4Regs.TBPHS.all = 3124;
  EPwm4Regs.TBCTL.bit.PHSEN = 1;
  EPwm4Regs.TBCTL.bit.SYNCOSEL = 0;
  EPwm4Regs.TBCTL.bit.CTRMODE = 3;
  EPwm4Regs.TBCTL.bit.PHSDIR = 1;    // count up after synchronization
  // configure PWM output, so larger CMPA value correspondes to higher duty
  // PWMA and PWMB are the inverse of one another
  EPwm4Regs.AQCTLA.bit.CAU = AQ_CLEAR;  // clear PWMA on COMPA in up-count mode
  EPwm4Regs.AQCTLA.bit.CAD = AQ_SET;    // set PWMA on COMPA in down-count mode
  EPwm4Regs.AQCTLB.bit.CAU = AQ_CLEAR;  // inversion of PWMB takes place in DB module
  EPwm4Regs.AQCTLB.bit.CAD = AQ_SET;
  // PWM dead band
  EPwm4Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;  // enable DB module output
  EPwm4Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;       // active high complementary (invert PEMB)
  EPwm4Regs.DBCTL.bit.IN_MODE = DBA_ALL;          // PWMA as delay source for both falling and rising edges
  EPwm4Regs.DBRED.bit.DBRED = 12;                 // dead time 60ns (x12 sysclk)
  EPwm4Regs.DBFED.bit.DBFED = 12;
  EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;
  // PWM trip
  //EPwm4Regs.TZSEL.bit.OSHT1 = 1;                      // enable on-shot triping
  //EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO;              // force A to low (higher gate cut-off)
  //EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_HI;              // force B to high (lower gates conducting)

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
  EPwm5Regs.AQCTLA.bit.CAU = AQ_CLEAR;
  EPwm5Regs.AQCTLA.bit.CAD = AQ_SET;
  EPwm5Regs.AQCTLB.bit.CAU = AQ_CLEAR;
  EPwm5Regs.AQCTLB.bit.CAD = AQ_SET;
  EPwm5Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm5Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm5Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm5Regs.DBRED.bit.DBRED = 12;
  EPwm5Regs.DBFED.bit.DBFED = 12;
  EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;
  //EPwm5Regs.TZSEL.bit.OSHT1 = 1;                      // enable on-shot triping
  //EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO;              // force A to low (higher gate cut-off)
  //EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_HI;              // force B to high (lower gates conducting)

  // EPWM6, up-down, 32 kHz
  EPwm6Regs.CMPA.bit.CMPA = 100;
  EPwm6Regs.TBPRD = 3125;
  EPwm6Regs.TBPHS.all = 3124;
  EPwm6Regs.TBCTL.bit.PHSEN = 1;
  EPwm6Regs.TBCTL.bit.SYNCOSEL = 0;
  EPwm6Regs.TBCTL.bit.CTRMODE = 3;
  EPwm6Regs.TBCTL.bit.PHSDIR = 1;
  EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;
  EPwm6Regs.AQCTLA.bit.CAD = AQ_SET;
  EPwm6Regs.AQCTLB.bit.CAU = AQ_CLEAR;
  EPwm6Regs.AQCTLB.bit.CAD = AQ_SET;
  EPwm6Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;
  EPwm6Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC;
  EPwm6Regs.DBCTL.bit.IN_MODE = DBA_ALL;
  EPwm6Regs.DBRED.bit.DBRED = 12;
  EPwm6Regs.DBFED.bit.DBFED = 12;
  EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
  EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;
  //EPwm6Regs.TZSEL.bit.OSHT1 = 1;                      // enable on-shot triping
  //EPwm6Regs.TZCTL.bit.TZA = TZ_FORCE_LO;              // force A to low (higher gate cut-off)
  //EPwm6Regs.TZCTL.bit.TZB = TZ_FORCE_HI;              // force B to high (lower gates conducting)

/*
  // EPWM4: trigger control process master
  //        and position loop
  EPwm4Regs.ETSEL.bit.INTEN = 1;   // enable interrupt
  EPwm4Regs.ETSEL.bit.INTSEL = 4;
  EPwm4Regs.ETPS.bit.INTPRD = 1;
*/

#ifdef DEBUG_CODE_PROFILING
  // initialize EPWM2 to measure code execution time
  EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP; // Count up
  EPwm2Regs.TBPRD = 0xFFFF; // Set timer period
  EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Disable phase loading
  EPwm2Regs.TBPHS.all = 0x0000; // Phase is 0
  EPwm2Regs.TBCTR = 0x0000; // Clear counter
  EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; // Clock ratio to SYSCLKOUT
  EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;
#endif

  CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
  EDIS;
}

/**
 *  initialize EQEP, the quadrature encoder interface
*/
void EQEP_GroupInit(void){

  EQep1Regs.QPOSMAX = 0xffffffff;       // max pos counter value
  EQep1Regs.QPOSINIT = 0;               // pos counter initial value
  EQep1Regs.QEPCTL.bit.FREE_SOFT = 2;   // emulation does not effect counter
  EQep1Regs.QEPCTL.bit.PCRM = 0b01;     // counter reset at overflow or underflow
  EQep1Regs.QEPCTL.bit.QPEN = 1;        // EQEP enable
}

/**
 *  configure memory for CPU1_CLA1
 *    L0 as program RAM
 *    L1 as data RAM
*/
void CLA_ConfigClaMemory(void){
  extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
  extern uint32_t Cla1ConstRunStart, Cla1ConstLoadStart, Cla1ConstLoadSize;
  EALLOW;

#ifdef _FLASH
  // copy CLA code from flash to RAM
  memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
  (uint32_t)&Cla1funcsLoadSize);

  // copy CLA const from flash to RAM
  memcpy((uint32_t *)&Cla1ConstRunStart, (uint32_t *)&Cla1ConstLoadStart,
  (uint32_t)&Cla1ConstLoadSize);

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
  // PieVectTable.CLA1_1_INT = &cla1Isr1;


  // trigger source
  DmaClaSrcSelRegs.CLA1TASKSRCSEL1.bit.TASK1 = 11;

  // Enable CLA interrupts at the group and subgroup levels
  // PieCtrlRegs.PIEIER11.bit.INTx1 = 1;
  // IER |= (M_INT11 );

  EDIS;
}
