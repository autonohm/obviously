/*
 * RobotFootprintFilter3D.h
 *
 *  Created on: 20.09.2013
 *      Author: chris
 */

#ifndef ROBOTFOOTPRINTFILTER3D_H_
#define ROBOTFOOTPRINTFILTER3D_H_

#include "obvision/icp/assign/filter/IPreAssignmentFilter.h"
#include "obcore/math/mathbase.h"

namespace obvious
{

class RobotFootprintFilter3D : public IPreAssignmentFilter
{
public:
  /**
   * Default constructor with radius mode
   * @param minRadius
   * @param maxRadius
   */
  RobotFootprintFilter3D(double minRadius = 0.0, double maxRadius = NAN);
  /**
   * Constructor to initialize filter with a box
   * @param[in]         x_lim           min and max value for x axis
   * @param[in]         y_lim           min and max value for y axis
   * @param[in]         z_lim           min and max value for z axis
   */
  RobotFootprintFilter3D(const double* x_lim,
                         const double* y_lim,
                         const double* z_lim);
  /**
   * Default destructor
   */
  ~RobotFootprintFilter3D();
  /**
   * Function to set offset
   * @param[in]          offset          offset in R3 coordinates
   */
  void setOffset(const double* offset);
  /**
   * Function to set min and max radius
   * @param[in]         minRadius        minimal radius around offset
   * @param[in]         maxRadius        maximum radius around offset
   */
  void setRadius(const double minRadius,
                 const double maxRadius);
  /**
   * Virtual function of IPreAssignmentFilter
   * @param scene
   * @param size
   * @param mask
   */
  virtual void filter(double** scene, unsigned int size, bool* mask);
private:
  double* _offset;
  double  _minRadius;
  double  _maxRadius;
  double* _x_lim;
  double* _y_lim;
  double* _z_lim;


};

};

#endif /* ROBOTFOOTPRINTFILTER_H_ */
