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

void SystemFullInit(void);
void SystemMemoryInit(void);
void SystemCheckTriming(void);

void GPIO_GroupInit(void);
void PIE_Init(void);
void ADC_GroupInit(void);
void EPWM_GroupInit(void);
void CLA_ConfigClaMemory(void);
void CLA_InitCpu1Cla1(void);

#endif
