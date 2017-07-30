/******************************************************************************
 * Copyright (C) 2017 by Yifan Jiang                                          *
 * jiangyi@student.ethz.com                                                   *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                       *
 ******************************************************************************/

/*
* implement functions for clark and park transformations
*/


#include "Transformation.h"
#include "math.h"

#define CLARK_C11    0.66666666666667f         // 2/3
#define CLARK_C12    -0.33333333333333f        // -1/3
#define CLARK_C22    0.577350269189625731058868041146f      // sqrt(3)/3

#define INVCLARK_C11   1.0f
#define INVCLARK_C12   0.0f
#define INVCLARK_C21   -0.5f
#define INVCLARK_C22   0.866025403784438596588302061718f     // sqrt(3)/2
#define INVCLARK_C31   -0.5f
#define INVCLARK_C32   -0.866025403784438596588302061718f    // -sqe(3)/2


/**
 *  perform clark transformation, from ABC to Alpha-Beta
 *  param in    input ABC vector
 *  param out   output Alpha-Beta vector
 */
#pragma CODE_SECTION(ClarkTransformation, ".TI.ramfunc");
void ClarkTransformation(ABCVec * in, AlBeVec * out){
  out->Alpha = CLARK_C11*in->A + CLARK_C12*(in->B+in->C);
  out->Beta = CLARK_C22*(in->B-in->C);
}

/**
 *  perform inverse clark transformation, from Alpha-Beta to ABC
 *  param in     input Alpha-Beta vector
 *  param out    output ABC vector
 */
#pragma CODE_SECTION(InvClarkTransformation, ".TI.ramfunc");
void InvClarkTransformation(AlBeVec * in, ABCVec * out){
  out->A = in->Alpha;
  out->B = INVCLARK_C21*in->Alpha + INVCLARK_C22*in->Beta;
  out->C = INVCLARK_C31*in->Alpha + INVCLARK_C32*in->Beta;
}

/**
 *  perform park transformation, from Alpha-Beta to DQ
 *  param in     input Alpha-Beta vector
 *  param out    output DQ vector
 *  param frame  frame of rotation, from Alpha-Beta to DQ
 */
#pragma CODE_SECTION(ParkTransformation, ".TI.ramfunc");
void ParkTransformation(AlBeVec * in, DQVec * out, RotationFrame * frame){
  out->D = frame->Cosine * in->Alpha + frame->Sine * in->Beta;
  out->Q = frame->Cosine * in->Beta - frame->Sine * in->Alpha;
}

/**
 *  perform inverse park transformation, from DQ to Alpha-Beta
 *  param in    input  DQ vector
 *  param out   output Alpha-Beta vector
 *  param frame  frame of rotation, from Alpha-Beta to DQ
 */
#pragma CODE_SECTION(InvParkTransformation, ".TI.ramfunc");
void InvParkTransformation(DQVec * in, AlBeVec * out, RotationFrame * frame){
  out->Alpha = frame->Cosine * in->D - frame->Sine * in->Q;
  out->Beta = frame->Sine * in->D - frame->Cosine * in->Q;
}
