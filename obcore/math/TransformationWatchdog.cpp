/*
 * TransformationWatchdog.cpp
 *
 *  Created on: 24.01.2014
 *      Author: user
 */

#include "TransformationWatchdog.h"
#include "obcore/math/geometry.h"
#include "Eigen/Eigen"

using namespace obvious;

TransformationWatchdog::TransformationWatchdog()
{
	_lastTransformation = new Matrix(4,4);
	_lastValidTransformation = new Matrix(4,4);
	_rotationTH = 0.0;
	_translationTH = 0.0;

}

TransformationWatchdog::~TransformationWatchdog()
{
	delete _lastTransformation;
	delete _lastValidTransformation;
}

void TransformationWatchdog::setInitTransformation(const obvious:: Matrix initMatrix)
{
	*_lastTransformation = initMatrix;
}

void TransformationWatchdog::setTranslationThreshold(double translationTH) {
	_translationTH = translationTH;
}

//void TransformationWatchdog::setTranslationThresholdX(const double translationTH_x) {
//
//}
//
//void TransformationWatchdog::setTranslationThresholdY(
//    const double* translationTH_y) {
//}
//
//void TransformationWatchdog::setTranslationThresholdZ(
//    const double* translationTH_z) {
//}

void TransformationWatchdog::setRotationThreshold(double rotationTH)
{
	_rotationTH = rotationTH;
}

//void TransformationWatchdog::setRotationThresholdRoll(
//    const double* rotationTH_roll) {
//}
//
//void TransformationWatchdog::setRotationThresholdPitch(
//    const double* rotationTH_pitch) {
//}
//
//void TransformationWatchdog::setRotationThresholdYaw(
//    const double* rotationTH_yaw) {
//}

bool TransformationWatchdog::checkWatchdog(obvious::Matrix newTransformation)
{
	// copy obvious matrix rotation to eigen rotation matrix
	Eigen::Matrix3f RNew, RValid;
	double sumNew = 0.0, sumValid = 0.0;
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++)
		{
			// rotation
			RNew(i, j)  = newTransformation(i,j);
			RValid(i, j)= _lastValidTransformation->operator()(i,j);

			// translation
			sumNew    += newTransformation(i, 3) * newTransformation(i, 3); // sum up
			sumValid  += _lastValidTransformation->operator()(i, 3)* _lastValidTransformation->operator()(i, 3);
		}
	}

	// get translation
	bool translationValid = false;
	const double euclideanDist = sqrt(sumNew) - sqrt(sumValid);
	if (euclideanDist >= _translationTH)
		translationValid = true;

	// get rotation angle
	bool rotationValid = false;
	Eigen::Vector3f anglesNew   = RNew.eulerAngles(0,1,2);
	Eigen::Vector3f anglesValid = RValid.eulerAngles(0,1,2);

	for(unsigned int i=0 ; i<3 ; i++)
	{
		if(std::fabs(anglesNew(i) - anglesValid(i)) >= _rotationTH)
			rotationValid = true;
	}

	if (translationValid || rotationValid)
	{
		// save new transformation as valid
		*_lastValidTransformation = newTransformation;
		return (true);
	}
	else
	{
		return (false);
	}
}

