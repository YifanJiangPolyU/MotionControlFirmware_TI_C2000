/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* define system-wide values here
*/

#ifndef SYS_DEF_H
#define SYS_DEF_H


/**
 *  define type of motor
 */
#define MTR_TYPE_NONE           0x00    // default, no motor
#define MTR_TYPE_PMSM_ROTARY    0x01    // PMSM rotary (include BLDC)
#define MTR_TYPE_PMSM_LINEAR    0x02    // PMSM linear
#define MTR_TYPE_DC_ROTARY      0x03    // brushed DC rotary
#define MTR_TYPE_DC_LINEAR      0x04    // DC linear (include VCM)
#define MTR_TYPE_STEPPER        0x05    // stepper motor

/**
 *  define process IDs
 *  IMPORTANT: process ID MUST match the index of the corresponding process class
 *  in the _ProcessArray
 */
#define PROCESS_NONE        0     // no process
#define PROCESS_CURRENT     1     // current control
#define PROCESS_SPEED       2     // speed control (rotary only)
#define PROCESS_POSITION    3     // position control
#define PROCESS_FORCE       4     // force or torque control
#define PROCESS_CLSW        10    // current loop sweepsine test process
#define PROCESS_PLSW        11    // position loop sweepsine test process
#define PROCESS_POLARITY    12    // polarity test process


#endif
