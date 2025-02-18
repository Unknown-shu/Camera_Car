/*
 * body_contral.h
 *
 *  Created on: 2025年1月14日
 *      Author: 15955
 */

#ifndef CODE_BODY_CONTRAL_H_
#define CODE_BODY_CONTRAL_H_
#include "zf_common_headfile.h"

typedef struct
{
    float alphaLeft, betaLeft;
    float alphaRight, betaRight;
    float XLeft,YLeft;
    float XRight, YRight;
    float pointA_x,pointA_y,pointB_x,pointB_y,pointC_x,pointC_y,pointD_x,pointD_y;
}IKparam;

#define L1 60
#define L4 60
#define L5 35
#define L3 90
#define L2 90
#define L = 100;//体长
void inverse_Kinematics(void);
void body_contral();
void ForwardKinematics(float angle_alpha, float angle_beta);

#endif /* CODE_BODY_CONTRAL_H_ */
