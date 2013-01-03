/**
* @file   MatTranslation.h
* @author Christian Pfitzner
* @date   03.01.2013
*
*
*/

#ifndef MATTRANSLAT_H_
#define MATTRANSLAT_H_

#include "obcore/math/MatD.h"
#include "obcore/Point3D.h"

/**
 * @namespace obvious
 */
namespace obvious {
/**
 * @class MatTranslation
 * @brief Class for translations in 3d
 *
 * This class is derived from MatD
 * @see MatD
 */
class MatTranslation : public MatD
{
public:
  /**
   * Standard constructor
   */
  MatTranslation(void);
  /**
   * Default destructor
   */
  virtual ~MatTranslation(void);
  //~~~~~~~~~~~~~~~~~~ Functions to SET ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  /**
   * Function to set transformation by single values
   * @param     x   translation in x direction
   * @param     y   translation in y direction
   * @param     z   translation in z direction
   */
  void setTrans(const double& x, const double& y, const double& z);
  /**
   * Function to set transformation by point3d
   * @param     p   point with single components for x,y,z
   */
  void setTrans(const Point3D& p);
};

} //namespace



#endif /* MATTRANSLAT_H_ */
