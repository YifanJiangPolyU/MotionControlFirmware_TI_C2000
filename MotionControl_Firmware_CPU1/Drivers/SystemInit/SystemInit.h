/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* contains functions used to initialize the whole system, including CPU1 and
* CPU2 settings which have to be configured by CPU1.
*
* Other CPU2 initialization functions are in the CPU2 project.
*/

#ifndef SYSTEM_INIT_H
#define SYSTEM_INIT_H

#include "F28x_Project.h"
#include "IsrAbstraction.h"
#include "CPU1_CLA1_common.h"

#ifdef __cplusplus
  extern "C" {
#endif

extern void SystemFullInit(void);
extern void SystemMemoryInit(void);
extern void SystemCheckTriming(void);

extern void Interrupt_Init(void);
extern void UART_Init(void);
extern void GPIO_GroupInit(void);
extern void ADC_GroupInit(void);
extern void EPWM_GroupInit(void);
extern void EQEP_GroupInit(void);
extern void CLA_ConfigClaMemory(void);
extern void CLA_InitCpu1Cla1(void);

extern void SPI_Init(void);

#ifdef __cplusplus
  }
#endif

#endif
