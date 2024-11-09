/*
 * Sys.h
 *
 *  Created on: 2024Äê10ÔÂ27ÈÕ
 *      Author: sun
 */

#ifndef CODE_SYS_SYS_H_
#define CODE_SYS_SYS_H_

#include "MyHeadFile.h"

float slope_calculate (uint8 begin, uint8 end,int * border);
void caculate_distance(uint8 start,uint8 end,int *border,float *slope_new,float *distance_new);
float Calculate_Curvature(int array[], int start, int size) ;
double curvature(double a[2],double b[2],double c[2]);
double distance(double a[2],double b[2]);
int collinear(double a[2],double b[2],double c[2]);

#endif /* CODE_SYS_SYS_H_ */
