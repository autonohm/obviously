/**
* @file   MatRot.h
* @author Christian Pfitzner
* @date   03.01.2013
*
*
*/

#ifndef MATROT_H_
#define MATROT_H_

#include "obcore/math/MatD.h"


/**
 * @namespace obvious
 */
namespace obvious{
/**
 * @class MatRot
 * @brief Class to describe rotation matrix
 *
 * This class is derived from MatD and is responsible for rotations in 3d
 * @see MatD
 */
class MatRot : public MatD
{
public:
  /**
   * Standard constructor
   */
  MatRot(void);
  /**
   * Default destructor
   */
  virtual ~MatRot(void);
  //~~~~~~~~~~~~~~~~~~ Functions to SET ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  /**
   * Function to set rotation by angles
   * @param[in]     rotX    rotation around x axis
   * @param[in]     rotY    rotation around y axis
   * @param[in]     rotZ    rotation around z axis
   */
  void setRot(const double& rotX, const double& rotY, const double& rotZ);
  /**
   * Function to set rotation with quaternion
   * @param[in]     q0      real part of quarternion
   * @param[in]     q1      i part
   * @param[in]     q2      j part
   * @param[in]     q3      k part
   */
  void setRot(const double& q0, const double& q1, const double& q2, const double&q3);
};
}



#endif /* MATROT_H_ */
