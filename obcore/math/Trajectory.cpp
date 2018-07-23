/*
 * Trajectory.cpp
 *
 *  Created on: 30.01.2014
 *      Author: user
 */

#include "Trajectory.h"

using namespace obvious;

Trajectory::Trajectory()
{

}

Trajectory::~Trajectory()
{

}

void obvious::Trajectory::setPose(const Matrix pose)
{
	_trajectory.push_back(pose);
}

std::vector<Matrix> obvious::Trajectory::getTrajectory(void) const
{
	return _trajectory;
}


