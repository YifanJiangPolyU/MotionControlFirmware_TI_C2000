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

#define PDO_MAX_BYTE    10

#define PDO_ID_CUSTOM    0
#define PDO_ID_CLSW      1
#define PDO_ID_PLSW      2

class PdoMaster{

  public:
    PdoMaster(ObjectDictionary * ObjectDictionaryPtr):
        _Obd(ObjectDictionaryPtr)
    {

    }

    ~PdoMaster(){}

  private:
    ObjectDictionary * _Obd;

    // current number of bytes in the PDO message
    uint16_t _PdoMsgNmberOfBytes;

    char _PdoID;
    uint16_t _PdoDataListPosition[PDO_MAX_BYTE];

};

#endif
