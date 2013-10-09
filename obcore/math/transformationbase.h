/*
 * transformationbase.cpp
 *
 *  Created on: 09.10.2013
 *      Author: chris
 */


#ifndef TRANSFORMATIONBASE_H
#define TRANSFORMATIONBASE_H

#include <math.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <cstring>
#include <iostream>

using namespace std;


namespace obvious {

// TRANSLATIONS
void translateXAxis(double* tf, double delta)
{
  double tf_tmp[16] ={1,  0, 0, delta,
                      0,  1, 0, 0,
                      0,  0, 1, 0,
                      0,  0, 0, 1  };
  std::memcpy(tf, tf_tmp, sizeof(tf_tmp));
}

void translateYAxis(double* tf, double delta)
{
  double tf_tmp[16] ={1,  0, 0, 0,
                      0,  1, 0, delta,
                      0,  0, 1, 0,
                      0,  0, 0, 1  };
  std::memcpy(tf, tf_tmp, sizeof(tf_tmp));
}

void translateZAxis(double* tf, double delta)
{
  double tf_tmp[16] ={1,  0, 0, 0,
                      0,  1, 0, 0,
                      0,  0, 1, delta,
                      0,  0, 0, 1  };
  std::memcpy(tf, tf_tmp, sizeof(tf_tmp));
}

// ROTATIONS
void rotateXAxis(double* tf, double angle)
{
  double tf_tmp[16] = {1, 0,            0,         0,
                       0, cos(angle), -sin(angle), 0,
                       0, sin(angle),  cos(angle), 0,
                       0, 0,            0,         1};
  std::memcpy(tf, tf_tmp, sizeof(tf_tmp));
}

void rotateYAxis(double* tf, double angle)
{
  double tf_tmp[16] = {cos(angle),  0, sin(angle), 0,
                       0,           1, 0,          0,
                       -sin(angle), 0, cos(angle), 0,
                       0,           0, 0,          1};
  std::memcpy(tf, tf_tmp, sizeof(tf_tmp));
}

void rotateZAxis(double* tf, double angle)
{
  double tf_tmp[16] = {cos(angle), -sin(angle), 0, 0,
                       sin(angle),  cos(angle), 0, 0,
                       0,           0,          1, 0,
                       0,           0,          0, 1};
  std::memcpy(tf, tf_tmp, sizeof(tf_tmp));
}

};


#endif // TRANSFORMATIONBASE_H
