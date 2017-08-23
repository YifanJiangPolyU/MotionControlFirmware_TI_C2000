/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* implement the calibration process class
*/

#ifndef CALIBRATION_PROCESS_H
#define CALIBRATION_PROCESS_H

#include "ControlTypeDef.h"
#include "ControlProcessBase.h"
#include "ControlProcessData.h"
#include "ObjectDictionaryEntryBase.h"
#include "Drivers/PowerStageControl/PowerStageControl.h"

class CalibrationProcess : public ControlProcessBase, public ObjectDictionaryEntryBase{

  public:

    CalibrationProcess(ControlProcessData * ControlProcessDataPtr);
    ~CalibrationProcess(){}

    virtual void Execute(void);
    virtual void Reset(void);

    enum CalibrationProcess_State{
      STATE_WAIT,
      STATE_CAL_CURRENT_OFFSET,
      STATE_COMPLETE
    };

    void AccessCurrentOffsetPhaseA(ObdAccessHandle * handle);
    void AccessCurrentOffsetPhaseB(ObdAccessHandle * handle);

  private:
    enum CalibrationProcess_State _State;

    float32_t _CurrentOffset_PhaseA;
    float32_t _CurrentOffset_PhaseB;

    float32_t _CurrentOffsetSumA;
    float32_t _CurrentOffsetSumB;
    int16_t   _CurrentOffsetSampleCnt;
    int16_t   _CurrentOffsetCycleCnt;

    uint16_t _WaitCounter;

    ControlProcessData * _ControlProcessData;

};

#endif
