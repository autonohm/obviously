/**
* @file     MatTransform.cpp
* @author   Christian Pfitzner
* @date     03.01.2013
*
*
*/

#include "obcore/math/MatTransform.h"

using namespace obvious;

MatTransform::MatTransform(void)
  : MatD(4,4)
{
  _rot   = new MatRot();
  _trans = new MatTranslation();
  for(unsigned int i=0 ; i<=3 ; i++)
    this->at(i,i) = 1.0;
}

MatTransform::~MatTransform(void)
{
  delete _rot;
  delete _trans;
}

void MatTransform::setRot(const double& rotX, const double& rotY, const double& rotZ)
{
  _rot->setRot(rotX, rotY, rotZ);
  updateRot();
}

void MatTransform::setTrans(const double& x, const double& y, const double& z)
{
  _trans->setTrans(x, y, z);
  updateTrans();
}

void MatTransform::setTrans(const Point3D& p)
{
  _trans->setTrans(p);
  updateTrans();
}

MatRot MatTransform::getMatRot(void)
{
  return *_rot;
}

MatTranslation MatTransform::getMatTranslation(void)
{
  return *_trans;
}

void MatTransform::updateRot(void)
{
  this->at(0,0) = _rot->at(0,0);
  this->at(1,0) = _rot->at(1,0);
  this->at(2,0) = _rot->at(2,0);

  this->at(0,1) = _rot->at(0,1);
  this->at(1,1) = _rot->at(1,1);
  this->at(2,1) = _rot->at(2,1);

  this->at(0,2) = _rot->at(0,2);
  this->at(1,2) = _rot->at(1,2);
  this->at(2,2) = _rot->at(2,2);
}
void MatTransform::updateTrans(void)
{
  this->at(0,3) = _trans->at(0,0);
  this->at(1,3) = _trans->at(1,0);
  this->at(2,3) = _trans->at(2,0);
}





