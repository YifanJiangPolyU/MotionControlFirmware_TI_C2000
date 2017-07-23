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
#include "PdoDataTypeDef.h"

class PdoMaster{

  public:
    PdoMaster(ObjectDictionary * ObjectDictionaryPtr,
              ControlProcessData * ControlProcessDataPtr):
        _Obd(ObjectDictionaryPtr),
        _ControlProcessData(ControlProcessDataPtr)
    {

    }

    ~PdoMaster(){}

    void ComposePdoMessage(CiA_Message * msg);

  private:

    void ComposeCLSW(CiA_Message * msg);
    void ComposePLSW(CiA_Message * msg);
    void ComposeDEBUG(CiA_Message * msg);
    void CopyData(PdoData * data, CiA_Message * msg);

    ObjectDictionary * _Obd;
    ControlProcessData * _ControlProcessData;

};

#endif
