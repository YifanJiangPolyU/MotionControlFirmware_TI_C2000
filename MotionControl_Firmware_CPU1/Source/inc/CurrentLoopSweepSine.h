/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* current loop sweep sine class
*
* Control the execution of current loop sweep sine, to obtain the frequency
* domain response of the current loop. This is the sweep sine process
* controller and setpoint generator only, current loop is executed by CLA.
*/

#ifndef _CURRENT_LOOP_SWEEPSINE_H
#define _CURRENT_LOOP_SWEEPSINE_H

#include "ControlProcessData.h"
#include "CurrentLoopController.h"
#include "ObjectDictionaryEntryBase.h"

class CurrentLoopSweepSine : public ControlProcessBase, public ObjectDictionaryEntryBase{

  public:
    CurrentLoopSweepSine(CurrentLoopController * CurrentLoopControllerPtr,
                          ControlProcessData * ControlProcessDataPtr)
    {
      _CurrentLoopController = CurrentLoopControllerPtr;
      _ControlProcessData = ControlProcessDataPtr;

      _ExcitationAmplitude = 0;
      _StartFreq = 0;
      _EndFreq = 0;

      _ExcitationLength = 0;
      _TimeCounter = 0;
    }

    ~CurrentLoopSweepSine(){}

    virtual void Execute(void);
    virtual void Reset(void);

    void AccessExcitationAmplitude(ObdAccessHandle * handle);
    void AccessExcitationLength(ObdAccessHandle * handle);
    void AccessStartFrequency(ObdAccessHandle * handle);
    void AccessEndFrequency(ObdAccessHandle * handle);
    void AccessSweepRate(ObdAccessHandle * handle);

  private:

    float32_t SignalGeneration(void);

    CurrentLoopController * _CurrentLoopController;
    ControlProcessData * _ControlProcessData;

    float32_t _ExcitationAmplitude;
    float32_t _StartFreq;
    float32_t _EndFreq;
    float32_t _SweepRate;

    uint16_t _ExcitationLength;
    uint16_t _TimeCounter;

};

#endif
