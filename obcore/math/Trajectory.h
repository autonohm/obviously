/*
 * Trajectory.h
 *
 *  Created on: 30.01.2014
 *      Author: user
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>
#include "obcore/math/linalg/linalg.h"

namespace obvious{

/**
 * @class 	Trajectory
 * @author	Christian Pfitzner
 * @date 		2014-01-30
 */
class Trajectory {
public:
	/**
	 * Standard Constructor
	 */
	Trajectory();
	/**
	 * Default destructor
	 */
	virtual ~Trajectory();
	/**
	 * Function to add single pose to trajectory
	 * @param 	pose	pose based on obvious::Matrix 4x4
	 */
	void setPose(const Matrix pose);

	std::vector<Matrix> getTrajectory(void) const;
private:
	std::vector<Matrix> _trajectory;
};

};

#endif /* TRAJECTORY_H_ */
