/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
*  implement the PdoMaster class
*  handles the PDO protocol in CANOpen CiA301 standard
*/

#include "PdoMaster.h"

/**
 *  compose a PDO message and put it into CiA_Message buffer
 *  @param id     ID of PDO message (Predefined, see PdoMaster.h)
 *  @param msg    buffer to store PDO message
 */
#pragma CODE_SECTION(".TI.ramfunc");
void PdoMaster::ComposePdoMessage(char id, CiA_Message * msg){
  switch (id) {
    case PDO_ID_DEBUG:
      break;
    case PDO_ID_CLSW:
      break;
    case PDO_ID_PLSW:
      break;
    default:
      break;
  }
}


#pragma CODE_SECTION(".TI.ramfunc");
void PdoMaster::ComposeCLSW(CiA_Message * msg){
  PdoData data;

  data.clsw.CurrentActual[0] = 0;
  data.clsw.CurrentActual[1] = 0;
  data.clsw.CurrentActual[2] = 0;
  data.clsw.CurrentActual[3] = 0;

  CopyData(&data, msg);

  msg->Pdo.PDO_ID = PDO_ID_CLSW;
}

#pragma CODE_SECTION(".TI.ramfunc");
void PdoMaster::ComposePLSW(CiA_Message * msg){
  PdoData data;

  data.plsw.EncPos = _ControlProcessData->_Position;
  data.plsw.CurrentDemand = _ControlProcessData->_DQCurrentSetpoint.Q;

  CopyData(&data, msg);

  msg->Pdo.PDO_ID = PDO_ID_PLSW;
}

#pragma CODE_SECTION(".TI.ramfunc")
void PdoMaster::ComposeDEBUG(CiA_Message * msg){

  PdoData data;

  CopyData(&data, msg);

  msg->Pdo.PDO_ID = PDO_ID_DEBUG;
}

#pragma CODE_SECTION(".TI.ramfunc")
void CopyData(PdoData * data, CiA_Message * msg){
  msg->Pdo.Data[0] = data->data[0];
  msg->Pdo.Data[1] = data->data[1];
  msg->Pdo.Data[2] = data->data[2];
  msg->Pdo.Data[3] = data->data[3];
  msg->Pdo.Data[4] = data->data[4];
}
