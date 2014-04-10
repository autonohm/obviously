#ifndef MATRIXFACTORY_H__
#define MATRIXFACTORY_H__

using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{

class Matrix;

/**
 * @class MatrixFactory
 * @author Stefan May
 */
class MatrixFactory
{
public:
  /**
   * Instantiate a 4x4 translation matrix, i.e. identity with last column set to translational input
   * @param tx x-component of translation
   * @param ty y-component of translation
   */
  static Matrix TranslationMatrix33(double tx, double ty);

  /**
   * Instantiate a 4x4 translation matrix, i.e. identity with last column set to translational input
   * @param tx x-component of translation
   * @param ty y-component of translation
   * @param tz z-component of translation
   */
  static Matrix TranslationMatrix44(double tx, double ty, double tz);

  /**
   * Instantiate a 3x3 transformation matrix
   * @param phi rotation about z-axis
   * @param tx x-component of translation
   * @param ty y-component of translation
   */
  static Matrix TransformationMatrix33(double phi, double tx, double ty);


  /**
   * Instantiate a 4x4 transformation matrix
   * @param phi rotation about z-axis
   * @param theta rotation about y-axis
   * @param psi rotation about x-axis
   * @param tx x-component of translation
   * @param ty y-component of translation
   * @param tz z-component of translation
   */
  static Matrix TransformationMatrix44(double phi, double theta, double psi, double tx=0, double ty=0, double tz=0);

private:

};

}

#endif //MATRIXFACTORY_H
