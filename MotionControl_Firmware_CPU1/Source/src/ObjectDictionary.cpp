/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* ObjectDictionary class
*
* implements object dictionary, which is compatible with CANOpen standard (CiA304)
* but is also extendable to make use of higher-BW communication methods
*/

#include "ObjectDictionary.h"

/**
 *  execute object dictionary access (read or write)
 *  @param msg_in       CiA＿message storing access commands (received from master)
 *  @param msg_out      CiA_Message storing access results
 *  @retval          None
 */
void ObjectDictionary::AccessEntry(CiA_Message * msg_in, CiA_Message * msg_out){
  ObdAccessHandle handle;
  handle.AccessType = msg_in->Sdo.SdoCtrl_ccs;
  handle.Data.DataInt16[0] = msg_in->Sdo.Data[0];
  handle.Data.DataInt16[1] = msg_in->Sdo.Data[1];

  int16_t pos = SearchEntry(msg_in->Sdo.SdoIdx, msg_in->Sdo.SdoSubIdx);
  if(pos==-1){
    // obj does not exist
    msg_out->Sdo.SdoAccessResult = OBD_ACCESS_ERR_IDX_NONEXIST;
  } else {
    if(_InstanceArray[pos]!=NULL){
      // call obd access function
      (_InstanceArray[pos]->*(_AccessFunctionArray[pos]))(&handle);

      // copy results
      msg_out->Sdo.SdoAccessResult = handle.AccessResult;
      msg_out->Sdo.Data[0] = handle.Data.DataInt16[0];
      msg_out->Sdo.Data[1] = handle.Data.DataInt16[1];
    } else {
      // obj does not exist
      msg_out->Sdo.SdoAccessResult = OBD_ACCESS_ERR_IDX_NONEXIST;
    }
  }

  // set som
  msg_out->Common.CANID = CANID_SDO_TX + NODE_ID;
  msg_out->Sdo.Length = 10;
}


/**
 *  search for the position of obj in the array, using binary search
 *  @param Idx       index of target object
 *  @param SubIdx    sub-index of the target object
 *  @retval          position of the object in the array
 *                   return -1 if not found.
 */
int16_t ObjectDictionary::SearchEntry(uint16_t Idx, uint16_t SubIdx){
  uint32_t target = (Idx<<8) | SubIdx;
  int16_t retval = -1;
  int16_t head = 0;
  int16_t tail = MAX_NO_OF_ENTRY-1;
  int16_t mid = 0;
  bool terminate = false;

  while(!terminate){
    mid = (head + tail)/2;
    if(target==_IdxArray[mid]){
      retval = mid;
      terminate = true;
    }else if(target>_IdxArray[mid]){
      head = mid+1;
    }else if(target<_IdxArray[mid]){
      tail = mid-1;
    }

    // found nothing
    if(head>tail){
      terminate = true;
    }
  }

  return retval;
}
