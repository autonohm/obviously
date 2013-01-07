/**
* @file   MatTrans.h
* @author Christian Pfitzner
* @date   02.01.2013
*
*
*/

#ifndef MATTRANS_H_
#define MATTRANS_H_

#include "obcore/math/MatRot.h"
#include "obcore/math/MatTranslation.h"

/**
 * @namespace obvious
 */
namespace obvious {
/**
 * @class MatTrans
 * @brief Class for transformations in 3d
 *
 * This class is responsible for tranformations in 3d and includes a matrix
 * for rotation and translation. Rotation can be set by euler angles or
 * quarternions.
 */
class MatTransform : public MatD
{
public:
  /**
   * Standard constructor
   */
  MatTransform(void);
  /**
   * Default destructor
   */
  virtual ~MatTransform(void);
  /**
   * Function to set rotation with angles
   * @param[in]     rotX    angle around x axis
   * @param[in]     rotY    angle around y axis
   * @param[in]     rotZ    angle around z axis
   */
  void setRot(const double& rotX, const double& rotY, const double& rotZ);
  //~~~~~~~~~~~~~~~~~~ Functions to SET ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  /**
   * Function to set translation
   * @param     x   translation in x direction
   * @param     y   translation in y direction
   * @param     z   translation in z direction
   */
  void setTrans(const double& x, const double& y, const double& z);
  /**
   * Function to set translation with point
   * @param     p   point3d
   */
  void setTrans(const Point3D& p);
  //~~~~~~~~~~~~~~~~~~ Functions to GET ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  /**
   * Function to return rotation matrix
   * @return  rotation matrix
   */
  MatRot getMatRot(void);
  /**
   * Function to return translation vector
   * @return  translation vector
   */
  MatTranslation getMatTranslation(void);
  /**
   * Function to save calibration to xml file
   * @param filename
   */
  void saveToXML(std::string filename);
private:
  /**
   * Function to update rotation
   */
  void updateRot(void);
  /**
   * Function to update translation
   */
  void updateTrans(void);

  MatRot*          _rot;        //!< matrix for rotation
  MatTranslation*  _trans;      //!< matrix for translation
};

}


#endif /* MATTRANS_H_ */
