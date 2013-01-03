/**
* @file MatTranslation.cpp
* @autor christian
* @date  03.01.2013
*
*
*/

#include "obcore/math/MatTranslation.h"

using namespace obvious;

MatTranslation::MatTranslation(void)
: MatD(3,1)
{

}

MatTranslation::~MatTranslation(void)
{

}

void MatTranslation::setTrans(const double& x, const double& y, const double& z)
{
  this->at(0,0) = x;
  this->at(1,0) = y;
  this->at(2,0) = z;
}

void MatTranslation::setTrans(const Point3D& p)
{
  this->at(0,0) = p.x();
  this->at(1,0) = p.y();
  this->at(2,0) = p.z();
}
