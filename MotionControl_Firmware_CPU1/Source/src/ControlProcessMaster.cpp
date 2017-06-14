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

#pragma DATA_SECTION("CPU1DataRAM")
static ControlProcessMaster * This;

void ControlProcessMaster::Update(void){
  hehe += 1;
}

#pragma CODE_SECTION(".TI.ramfunc");
extern "C" void CallControlProcessMaster(void){
  This->Update();
}

void CreateControProcessMasterInstance(void){

  #pragma DATA_SECTION("CPU1DataRAM")
  static ControlProcessMaster ControProcessMasterInstance;

  This = &ControProcessMasterInstance;
}
