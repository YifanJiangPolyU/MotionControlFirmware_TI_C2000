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
#include "F28x_Project.h"
#include "DataTypeHelper.h"

#define NODE_ID         0x003

#define CANID_NMT       0x000
#define CANID_SYNC      0x080
#define CANID_SDO_TX    0x580
#define CANID_SDO_RX    0x600
#define CANID_PDO_TX    0x180
#define CANID_PDO_RX    0x200

// CSS values to control segmented read/write
#define SDO_CSS_WRITE        0
#define SDO_CSS_READ         3

#define OBD_WRITE                          SDO_CSS_WRITE
#define OBD_READ                           SDO_CSS_READ
#define OBD_ACCESS_SUCCESS                 0x00  // access successful
#define OBD_ACCESS_ERR_READ                0x01  // access failed, trying to read a non-readable object
#define OBD_ACCESS_ERR_WRITE               0x02  // access failed, trying to write to a read-only object
#define OBD_ACCESS_ERR_IDX_NONEXIST        0x03  // access failed, index does not exist

// CSS values to control block read/write
#define SDO_CSS_BLKREAD            5
#define SDO_CSS_BLKWRITE           6

// NMT mode transitions
#define NMT_TO_OP          0x01
#define NMT_TO_STOP        0x02
#define NMT_TO_PREOP       0x80
#define NMT_RESET          0x81
#define NMT_RESET_COMM     0x82
// NMT custom modes
#define NMT_CLSW           0xA0
#define NMT_PLSW           0xA1
#define NMT_POLARITY       0xA2

/**
 *  define CANOpen SDO control data
 */
typedef struct CiA_SdoControlTypedef{
  uint32_t SdoCtrl_ccs    : 4;      // cmd type
  uint32_t SdoCtrl_n      : 2;      // data length
  uint32_t SdoCtrl_e      : 1;      // whether it's a expedited transfer
  uint32_t SdoCtrl_s      : 1;      // where to find data length for segmented access
  uint32_t SdoIdx         : 16;     // obj index and subindex
  uint32_t SdoSubIdx      : 8;
} CiA_SdoControl;

typedef union CiA_SdoDataTypedef{
  float32_t DataFloat32;
  uint32_t DataInt32;
  uint16_t DataInt16[3];
} CiA_SdoData;

/**
 *  define CANOpen SDO data structure
 */
typedef struct CiA_SdoMessageTypedef{

  uint16_t CANID;
  uint16_t reserve :7;    // not used
  uint16_t RTR :1;        // RTR bit
  uint16_t Length :8;     // data length

  uint32_t SdoCtrl_ccs    : 4;      // cmd type
  uint32_t SdoCtrl_n      : 2;      // data length
  uint32_t SdoCtrl_e      : 1;      // whether it's a expedited transfer
  uint32_t SdoCtrl_s      : 1;      // where to find data length for segmented access
  uint32_t SdoIdx         : 16;     // obj index and subindex
  uint32_t SdoSubIdx      : 8;
  uint16_t SdoAccessResult;
  // CANOpen SDO data + extended data
  // CAN PHY: use first 4 bytes only
  // EtherCAT and UART: use all bytes
  // access using __byte()
  uint16_t Data[3];

} CiA_SdoMessage;


/**
 *  define CANOpen PDO data structure
 */
typedef struct CiA_PdoMessageTypedef{

  uint16_t CANID;
  uint16_t reserve :7;    // not used
  uint16_t RTR :1;        // RTR bit
  uint16_t Length :8;     // data length

  // CANOpen PDO data + extended data
  // CAN PHY: use first 8 bytes only
  // EtherCAT and UART: use all bytes
  // access using __byte()
  uint16_t Data[5];
  uint16_t PDO_ID:    8;
  uint16_t reserved:  8;

} CiA_PdoMessage;

/**
 *  define CANOpen NMT data structure
 */
typedef struct CiA_NmtMessageTypedef{

  // CANOpen NMT data
  uint16_t CANID;
  uint16_t reserve :7;    // not used
  uint16_t RTR :1;        // RTR bit
  uint16_t Length :8;     // data length
  uint16_t State  : 8;    // requested state
  uint16_t NodeID : 8;    // target node

  uint16_t reserved[5];

} CiA_NmtMessage;

typedef struct CiA_CommonTypeDef{
  uint16_t CANID;
  uint16_t reserve :7;    // not used
  uint16_t RTR :1;        // RTR bit
  uint16_t Length :8;     // data length
  uint16_t Data[6];       // data buffer, access using __byte()
} CiA_Common;


typedef union CiA_MessageTypedef{

  CiA_Common       Common;
  CiA_NmtMessage   Nmt;
  CiA_SdoMessage   Sdo;
  CiA_PdoMessage   Pdo;
  uint16_t data[8];

} CiA_Message;



#endif
