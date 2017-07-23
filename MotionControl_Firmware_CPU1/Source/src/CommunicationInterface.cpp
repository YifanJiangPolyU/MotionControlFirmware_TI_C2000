/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* communication interface
* collectively handle all communication traffic
*/

#include "CommunicationInterface.h"
#include "PdoDataTypeDef.h"
#include "Drivers/GpioDriver/GpioDriver.h"

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>

#pragma DATA_SECTION("CPU1DataRAM")
static CommunicationInterface * CommunicationInterfacePtr;

extern Semaphore_Handle SemaphoreObdAccess;

CommunicationInterface::CommunicationInterface(UartDriver * UartDriverPtr,
                       ObjectDictionary * ObjectDictionaryPtr,
                       ControlProcessData * ControlProcessDataPtr,
                       PdoMaster * PdoMasterPtr):
    _UartDriver(UartDriverPtr),
    _ObjectDictionary(ObjectDictionaryPtr),
    _ControlProcessData(ControlProcessDataPtr),
    _PdoMaster(PdoMasterPtr),
    _NmtUpdated(false),
    _PdoUpdated(false),
    _SdoReplyPending(false),
    _NmtNewState(0x00),
    _TxCounter(0),
    _TxRatePrescale(COMM_REPORT_RATE_8000)
{
  ciamsg = &_MsgBuffer;
  CommunicationInterfacePtr = this;
}

/**
 *  initialize the CiA message buffers
 *  @param msgptr    pointer to CiA_Message
 *  @param sdoptr
 *  @param pdoptr
 *  @retval None
 */
void CommunicationInterface::SetCiaMsgBuffer(CiA_Message * msgptr)
{
  //ciamsg = msgptr;
}

/**
 *  transmits message(s) over communication interface
 */
#pragma CODE_SECTION(".TI.ramfunc");
void CommunicationInterface::ExecuteTransmission(void){
  CiA_Message msg;

  if(_ControlProcessData->_SyncFlag == 0) {
      if(_SdoReplyPending==true){
        _SdoReplyPending = false;
        _UartDriver->SendMessage(&_SodReplyMsg);
      } else {
        // transmit PDO status report
        msg.Common.CANID = CANID_PDO_TX;
        msg.Common.Length = 11;

        _UartDriver->SendMessage(&msg);
      }
  }
}

/**
 *  check and process messages received over communication interface
 */
#pragma CODE_SECTION(".TI.ramfunc");
void CommunicationInterface::ExecuteReception(void){

  uint16_t msgcnt = _UartDriver->ExecuteParsing(ciamsg);
  if(msgcnt > 0 ){
    if(ciamsg->Common.CANID == CANID_NMT){
      // handle CANOpen NMT protocol
      _NmtNewState = __byte_uint16_t(ciamsg->Common.Data, 0);
      _NmtUpdated = true;

    } else if(ciamsg->Common.CANID == CANID_SYNC){
      // handle sync msg

    } else if((ciamsg->Common.CANID-NODE_ID)==CANID_SDO_RX) {
      // handle CANOpen SDO
      // _ObjectDictionary->AccessEntry(ciamsg, &_SodReplyMsg);
      // _SdoReplyPending = true;

      // post semaphore to unlock sysbios task, which accesses obd
      Semaphore_post(SemaphoreObdAccess);

    } else if((ciamsg->Common.CANID-NODE_ID)==CANID_PDO_RX) {
      // handle CANOpen PDO
      _PdoUpdated = true;
    } else {
      // default, nothing
    }
  }

  //ciamsg->reserve = 0x0A;
}

/**
 *  retrive state transition cmd from NMT message
 *  @param mnt_state  buffer to hold new state
 *  @retval true if new NMT message is received, false otherwise
 */
#pragma CODE_SECTION(".TI.ramfunc");
bool CommunicationInterface::CheckNmtUpdate(uint16_t * mnt_state){
  if(_NmtUpdated == true){
    *mnt_state = _NmtNewState;
    _NmtUpdated = false;
    return true;
  }

  return false;
}

/**
 *  retrive state transition cmd from PDO message
 *  @param
 *  @retval true if new PDO message is received, false otherwise
 */
#pragma CODE_SECTION(".TI.ramfunc");
bool CommunicationInterface::CheckPdoUpdate(void){
  if(_PdoUpdated == true){
    _PdoUpdated = false;
    return true;
  }

  return false;
}

/**
 *  function to handle object dictionary access. to be
 *  run as a Sys/Bios task
 */
void CommunicationInterface::HandleObdAccess(void){
  for(;;){
    Semaphore_pend(SemaphoreObdAccess, BIOS_WAIT_FOREVER);
    _ObjectDictionary->AccessEntry(ciamsg, &_SodReplyMsg);
    _SdoReplyPending = true;
  }
}

/**
 *  C warper function for SysBios tasks
 */
extern "C" void OsTaskWarper_HandleObdAccess(void){
  CommunicationInterfacePtr->HandleObdAccess();
}
