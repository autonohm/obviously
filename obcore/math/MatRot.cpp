/**
* @file   MatRot.cpp
* @author Christian Pfitzner
* @date   03.01.2013
*/

#include "obcore/math/MatRot.h"
#include "obcore/math/mathbase.h"

using namespace obvious;

MatRot::MatRot(void) : MatD(3,3)
{

}

MatRot::~MatRot(void)
{

}

void MatRot::setRot(const double& rotX, const double& rotY, const double& rotZ)
{
  // first column
  this->at(0,0) = cos(rotZ)*cos(rotY);
  this->at(1,0) = sin(rotZ)*cos(rotY);
  this->at(2,0) = -sin(rotY);
  // secound column
  this->at(0,1) = cos(rotZ)*sin(rotY)*sin(rotX) - sin(rotZ)*cos(rotX);
  this->at(1,1) = sin(rotZ)*sin(rotY)*cos(rotX) + cos(rotZ)*cos(rotX);
  this->at(2,1) = cos(rotY)*sin(rotX);
  // third column
  this->at(0,2) = cos(rotZ)*sin(rotY)*cos(rotX) + sin(rotZ)*sin(rotX);
  this->at(1,2) = sin(rotZ)*sin(rotY)*cos(rotX) - cos(rotZ)*sin(rotX);
  this->at(2,2) = cos(rotY)*cos(rotX);
}

void MatRot::setRot(const double& q0, const double& q1, const double& q2, const double&q3)
{
  // first column
  this->at(0,0) = 1-2*(q2*q2 + q3*q3);
  this->at(1,0) = 2*(q0*q3 + q1*q2);
  this->at(2,0) = -2*(q0*q2 + q1*q3);
  // secound column
  this->at(0,1) = -2*(q0*q3 + q1*q2);
  this->at(1,1) = 1-2*(q1*q1 + q3*q3);
  this->at(2,1) = 2*(q0*q1 + q2*q3);
  // third column
  this->at(0,2) = 2*(q0*q2 + q1*q3);
  this->at(1,2) = -2*(q0*q1 + q2*q3);
  this->at(2,2) = 1-2*(q1*q1 + q2*q2);
}




