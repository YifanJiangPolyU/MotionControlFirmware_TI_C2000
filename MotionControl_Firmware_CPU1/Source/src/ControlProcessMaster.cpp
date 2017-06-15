/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/**
 * Control process master class
 *   contains the main state machine of CPU1, handling control processes
 */

#include "ControlProcessMaster.h"

/**
 *  pointer to globally unique object of ControlProcessMaster
 */
#pragma DATA_SECTION("CPU1DataRAM")
static ControlProcessMaster * This;

/**
 *  Constructor
 */
ControlProcessMaster::ControlProcessMaster(CommutationMaster * CommutationMasterPtr,
                                           ObjectDictionary * ObjectDictionaryPtr ):
  _state(STATE_IDEL),
  hehe(0)
  {
    This = this;

    _CommutationMaster = CommutationMasterPtr;
    _ObjectDictionary = ObjectDictionaryPtr;
  }

/**
 *  C warper to call ControlProcessMaster from ISR
 */
#pragma CODE_SECTION(".TI.ramfunc");
extern "C" void CallControlProcessMaster(void){
  This->Execute();
}

/**
 * execute ControlProcessMaster
 */
void ControlProcessMaster::Execute(void){
    hehe = 123;
}
