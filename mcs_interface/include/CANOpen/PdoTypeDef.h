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

typedef struct PdoCLSWTypedef{
  uint16_t CurrentActual[4];
  uint16_t Reserved;
}PdoMessageCLSW;

typedef struct PdoPLSWTypedef{
  uint16_t Status;
  uint32_t EncPos;
  uint16_t CurrentDemand;
  uint16_t Reserved;
}PdoMessagePLSW;

typedef struct PdoDEBUGTypedef{
  uint16_t Reserved[5];
}PdoMessageDEBUG;

typedef union PdoDataTypedef{
  PdoMessageCLSW    clsw;
  PdoMessagePLSW    plsw;
  PdoMessageDEBUG   debug;
  uint16_t          data[5];
}PdoData;

#endif
