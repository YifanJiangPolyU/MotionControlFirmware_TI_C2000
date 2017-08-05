/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

#ifndef PDO_TYPEDEF_H
#define PDO_TYPEDEF_H

#define PDO_ID_DEBUG     255     // debug PDO
#define PDO_ID_CLSW      1       // current loop sweepsine report
#define PDO_ID_PLSW      2       // position loop sweepsine report
#define PDO_ID_PR1       3       // process report 1
#define PDO_ID_PR2       4       // process report 1
#define PDO_ID_PR3       5       // process report 1


// define PDO data access types

/**
 *  Slave to master
 *  send current sweepsine data
 */
typedef struct PdoCLSWTypedef{
  int16_t CurrentActual[4];
  uint16_t StatusReg;
}PdoMessageCLSW;

/**
 *  Slave to master
 *  send position sweepsine data
 */
typedef struct PdoPLSWTypedef{
  uint16_t Status;
  int32_t EncPos;
  int16_t CurrentDemand;
  uint16_t StatusReg;
}PdoMessagePLSW;

/**
 *  Master to slave
 *  used for cyclic sync position mode
 */
typedef struct PdoPOSTypedef{
  int32_t TargetEncPos;
  int16_t TargetSpeed;
  int16_t TargetAccel;
  uint16_t StatusReg;
}PdoMessagePOS;

/**
 *  Master to slave
 *  used for cyclic sync current mode
 */
typedef struct PdoCURRENTypedef{
  int32_t TargetCurrent_Q;
  uint16_t Reserved[2];
  uint16_t StatusReg;
}PdoMessageCURRENT;

/**
 *  Slave to master
 *  send debug data
 */
typedef struct PdoDEBUGTypedef{
  uint16_t Reserved[4];
  uint16_t StatusReg;
}PdoMessageDEBUG;

typedef union PdoDataTypedef{
  PdoMessageCLSW    clsw;
  PdoMessagePLSW    plsw;
  PdoMessageDEBUG   debug;
  uint16_t          data[5];
}PdoData;

/**
 *  define data struct
 */
 typedef struct StatusRefBit_typedef{
       uint16_t State :                3;
       uint16_t ErrOverCurrentPeak :   1;
       uint16_t ErrOverCurrentRms :    1;
       uint16_t ErrOverTemperature :   1;
       uint16_t ErrUnderVoltage :      1;
       uint16_t ErrOverVoltage :       1;
       uint16_t ErrPowerStage :        1;
       uint16_t ErrSoftware :          1;
       uint16_t ErrHardware :          1;
       uint16_t ErrCommunication :     1;
       uint16_t reserve :              5;
} StatusRefBit;

typedef union SystemStatusRegTypedef{
  StatusRefBit bit;
  uint16_t all;
}SystemStatusReg;

#endif
