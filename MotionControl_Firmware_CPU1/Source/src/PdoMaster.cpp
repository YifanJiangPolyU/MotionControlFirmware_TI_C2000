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
 *  @param msg    buffer to store PDO message
 */
#pragma CODE_SECTION(".TI.ramfunc");
void PdoMaster::ComposePdoMessage(CiA_Message * msg){
  msg->Common.CANID = CANID_PDO_TX + NODE_ID;
  msg->Common.Length = 11;
  switch (_ControlProcessData->_PdoID) {
    case PDO_ID_DEBUG:
      break;
    case PDO_ID_CLSW:
      ComposeCLSW(msg);
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

  data.clsw.CurrentActual[0] = _ControlProcessData->_CurrentSweepSineBuffer[0];
  data.clsw.CurrentActual[1] = _ControlProcessData->_CurrentSweepSineBuffer[1];
  data.clsw.CurrentActual[2] = _ControlProcessData->_CurrentSweepSineBuffer[2];
  data.clsw.CurrentActual[3] = _ControlProcessData->_CurrentSweepSineBuffer[3];
  _ControlProcessData->ClearCurrentSweepSineBuffer();

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
void PdoMaster::CopyData(PdoData * data, CiA_Message * msg){
  msg->Pdo.Data[0] = data->data[0];
  msg->Pdo.Data[1] = data->data[1];
  msg->Pdo.Data[2] = data->data[2];
  msg->Pdo.Data[3] = data->data[3];
  msg->Pdo.Data[4] = data->data[4];
}
