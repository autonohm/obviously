/*
 * TransformationWatchdog.cpp
 *
 *  Created on: 24.01.2014
 *      Author: user
 */

#include "TransformationWatchdog.h"

TransformationWatchdog::TransformationWatchdog() {
	// TODO Auto-generated constructor stub

}

TransformationWatchdog::~TransformationWatchdog() {
	// TODO Auto-generated destructor stub
}

void TransformationWatchdog::setTranslationThreshold(
		const double* translationTH)
{
	_translationTH = translationTH;
}

void TransformationWatchdog::setTranslationThresholdX(
		const double* translationTH_x)
{

}

void TransformationWatchdog::setTranslationThresholdY(
		const double* translationTH_y) {
}

void TransformationWatchdog::setTranslationThresholdZ(
		const double* translationTH_z) {
}

void TransformationWatchdog::setRotationThreshold(const double* rotationTH)
{
	_rotationTH = rotationTH;
}

void TransformationWatchdog::setRotationThresholdRoll(
		const double* rotationTH_roll) {
}

void TransformationWatchdog::setRotationThresholdPitch(
		const double* rotationTH_pitch) {
}

void TransformationWatchdog::setRotationThresholdYaw(
		const double* rotationTH_yaw) {
}

bool TransformationWatchdog::checkWatchdog(
		const obvious::Matrix* newTransformation)
{
	// get translation
	bool translationValid;

	// euclidean distance
	double euclideanDist;

	// get rotation angle
	bool rotationValid;

	if(translationValid || rotationValid)
		return(true);
	else
		return(false);
}


