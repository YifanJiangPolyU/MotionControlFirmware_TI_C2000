/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* define CANOpen CiA data types
*/

#ifndef CIA_TYPEDEF_H
#define CIA_TYPEDEF_H

#include "stdint.h"

/**
 *  define CANOpen SDO data structure
 */
typedef struct CiA_SdoDataTypedef{

  uint32_t SdoCtrl    : 8;
  uint32_t SdoIdx     : 16;
  uint32_t SdoSubIdx  : 8;

  // CANOpen SDO data + extended data
  // CAN PHY: use first 4 bytes only
  // EtherCAT and UART: use all bytes
  uint32_t Data[3];

} CiA_SdoData;

/**
 *  define CANOpen PDO data structure
 */
typedef struct CiA_PdoDataTypedef{

  // CANOpen PDO data + extended data
  // CAN PHY: use first 8 bytes only
  // EtherCAT and UART: use all bytes
  uint32_t Data[4];

} CiA_PdoData;

/**
 *  define CANOpen NMT data structure
 */
typedef struct CiA_NmtDataTypedef{

  // CANOpen NMT data
  uint16_t State  : 8;
  uint16_t NodeID : 8;

} CiA_NmtData;

typedef struct CiA_MessageTypedef{

  // Fits
  uint16_t CANID;
  uint16_t Data[8];

} CiA_Message;


#endif
