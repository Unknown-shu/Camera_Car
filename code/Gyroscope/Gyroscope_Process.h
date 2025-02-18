/*
 * process.h
 *
 *  Created on: 2023Äê12ÔÂ13ÈÕ
 *      Author: Â¬¾¢º­
 */

#ifndef CODE_GYROSCOPE_PROCESS_H_
#define CODE_GYROSCOPE_PROCESS_H_

#include "Myheadfile.h"
#include "math.h"
extern float pitch[2], roll[2], yaw[2];

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
float LPF2_T2(float xin);

#endif /* CODE_GYROSCOPE_PROCESS_H_ */
