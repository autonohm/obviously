/*
 * TransformationWatchdog.h
 *
 *  Created on: 24.01.2014
 *      Author: user
 */

#ifndef TRANSFORMATIONWATCHDOG_H_
#define TRANSFORMATIONWATCHDOG_H_

#include "math/linalg/linalg.h"
/**
 * @class 	TransformationWatchdog
 * @author 	Christian Pfitzner
 * @date	2014-01-24
 */
class TransformationWatchdog {
public:
	/**
	 * Default constructor
	 */
	TransformationWatchdog();
	/**
	 * Default destructor
	 */
	virtual ~TransformationWatchdog();

	// ---> SETTER
	/**
	 * Function to set threshold for all translation degrees of freedom
	 * @param 	translationTH		threshold in meters
	 */
	void setTranslationThreshold(  const double* translationTH);
	/**
	 * Function to set threshold for translation in x direction
	 * @param translationTH_x		threshold in meters
	 */
	void setTranslationThresholdX( const double* translationTH_x);
	/**
	 * Function to set threshold for translation in y direction
	 * @param translationTH_y		threshold in meters
	 */
	void setTranslationThresholdY( const double* translationTH_y);
	/**
	 * Function to set threshold for translation in z direction
	 * @param translationTH_z		threshold in meters
	 */
	void setTranslationThresholdZ( const double* translationTH_z);
	/**
	 * Function to set threshold for all rotation degrees of freedom
	 * @param rotationTH			threshold for rotation in rad
	 */
	void setRotationThreshold(     const double* rotationTH);
	/**
	 * Function to set threshold for roll angle
	 * @param rotationTH_roll		threshold for rotation roll in rad
	 */
	void setRotationThresholdRoll( const double* rotationTH_roll);
	/**
	 * Function to set threshold for pitch angle
	 * @param rotationTH_pitch		threshold for rotation pitch in rad
	 */
	void setRotationThresholdPitch(const double* rotationTH_pitch);
	/**
	 * Function to set threshold for yaw angle
	 * @param rotationTH_yaw		threshold for rotation yaw in rad
	 */
	void setRotationThresholdYaw(  const double* rotationTH_yaw);

	/**
	 * Function to check watchdog
	 * @param 	newTransformation	last available transformation
	 * @return	TRUE if transformation is bigger than set threshold
	 */
	bool checkWatchdog(const obvious::Matrix* newTransformation);
private:
	obvious::Matrix _lastTransformation;			//!< matrix with last transformation
	obvious::Matrix _lastValidTransformation;		//!< matrix with last valid transformation

	double* _translationTH;							//!< threshold for translation
	double* _rotationTH;							//!< threshold for rotation
};

#endif /* TRANSFORMATIONWATCHDOG_H_ */
