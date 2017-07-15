/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* commutation master class
*
* Contains functionalities related to commutation, including: a) initial
* commutation angle detection, b) initial polarity detection, and c) real-time
* commutation with encoder and/or hall sensors
*/

#ifndef COMMUTATION_MASTER_H
#define COMMUTATION_MASTER_H

#include "stdint.h"
#include "F28x_Project.h"

#include "CiATypeDef.h"
#include "ObjectDictionaryEntryBase.h"

class CommutationMaster: public ObjectDictionaryEntryBase{

  public:
    CommutationMaster():
      _NPolePair(1),
      _CountPerRev(1024),
      _PolePitch(20),
      _LinearResolution(1000),
      _HallSensorPattern(0b001),
      _CommAnglePerCount(0.0f),
      _CommAngle(0.0f),
      _CommAngle_Cos(1.0f),
      _CommAngle_Sin(0.0f)
    {};

    ~CommutationMaster(){};

    void Update(int32_t PosCounter);
    void UpdateCommResolution(void);

    void AccessNPolePair(ObdAccessHandle * handle);
    void AccessCountPerRev(ObdAccessHandle * handle);
    void AccessPolePitch(ObdAccessHandle * handle);
    void AccessLinearResolution(ObdAccessHandle * handle);

  private:
    int32_t _PosCounter_new;           // position counter value
    int32_t _PosCounter_old;

    uint16_t  _NPolePair;              // number of pole pairs (rotary)
    uint32_t  _CountPerRev;            // resolution: count per revolution (rotaty)

    float32_t _PolePitch;              // pitch of pole pair (linear, unit: mm)
    uint16_t _LinearResolution;       // resolution of linear encoder (unit: cnt/mm)

    uint16_t  _HallSensorPattern;      // hall sensor pattern (used only when hall sensor available)
    float32_t _CommAnglePerCount;      // electrical angle per count (rad)
    float32_t _CommAngle;              // electrical angle (rad)
    float32_t _CommAngle_Cos;          // cosine of electrical angle
    float32_t _CommAngle_Sin;          // sine of electrical angle

};

#endif
