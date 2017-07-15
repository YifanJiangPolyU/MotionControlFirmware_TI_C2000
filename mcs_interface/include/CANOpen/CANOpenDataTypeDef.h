/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

#ifndef CANOPEN_DATA_TYPEDEF_H
#define CANOPEN_DATA_TYPEDEF_H

#define NODE_ID         0x003

#define CIA_SOF_PATTERN  0xEF
#define CIA_EOF_PATTERN  0xDB

#define CANID_NMT       0x000
#define CANID_SYNC      0x080
#define CANID_SDO_TX    0x580
#define CANID_SDO_RX    0x600
#define CANID_PDO_TX    0x180
#define CANID_PDO_RX    0x200

// CSS values to control segmented read/write
#define SDO_CSS_WRITE        0
#define SDO_CSS_READ         3

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


typedef struct ObdAccessHandleTypedef{
  uint16_t AccessType     :  8;
  uint16_t AccessResult   :  8;
  union {
      float DataFloat32;
      int32_t   DataInt32;
      char      DataInt8[6];
      int16_t   DataInt16[3];
  } Data;

} ObdAccessHandle;


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
  uint8_t Data[12];       // data buffer, access using __byte()
} CiA_Common;


typedef union CiA_MessageTypedef{

  CiA_Common       Common;
  CiA_NmtMessage   Nmt;
  CiA_SdoMessage   Sdo;
  CiA_PdoMessage   Pdo;
  uint8_t data[16];

} CiA_Message;



#endif
