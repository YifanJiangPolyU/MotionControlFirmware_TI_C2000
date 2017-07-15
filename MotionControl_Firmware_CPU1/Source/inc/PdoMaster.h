/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
*  define the PdoMaster class
*  handles the PDO protocol in CANOpen CiA301 standard
*/

#ifndef _PDO_MASTER_H
#define _PDO_MASTER_H

#include "CiATypeDef.h"
#include "ObjectDictionary.h"
#include "ControlProcessData.h"

#define PDO_MAX_BYTE    10

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



class PdoMaster{

  public:
    PdoMaster(ObjectDictionary * ObjectDictionaryPtr,
              ControlProcessData * ControlProcessDataPtr):
        _Obd(ObjectDictionaryPtr),
        _ControlProcessData(ControlProcessDataPtr)
    {

    }

    ~PdoMaster(){}

    void ComposePdoMessage(char id, CiA_Message * msg);

  private:

    void ComposeCLSW(CiA_Message * msg);
    void ComposePLSW(CiA_Message * msg);
    void ComposeDEBUG(CiA_Message * msg);
    void CopyData(PdoData * data, CiA_Message * msg);

    ObjectDictionary * _Obd;
    ControlProcessData * _ControlProcessData;

    char _PdoID;

};

#endif
