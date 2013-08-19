#ifndef _PROJECTION_H
#define _PROJECTION_H

#include "obcore/math/Matrix.h"

using obvious::Matrix;

/**
 * @class Projection
 * @date 29.08.2012
 * @author: Philipp Koch, Stefan May
 */
class Projection
{
  public:

	 /**
	  * Konstructor
	  * @param proj_data double poitner to 3x4 projection matrix
	  */
    Projection(double* proj_data);

    /**
     * Destructor
     */
    ~Projection();

    /**
     * Element operator
     */
    double* operator [] (unsigned int i);

 	 /**
	  * Calculates 3D coordinates out of depth image data
	  * @param col column index in depth image
	  * @param row row index in depth image
	  * @param depth value of depth image at given pixel
	  * @param point3 point in homogeneous coordinates (x,y,z,1) has to be allocated by calling function
	  */
    void project2Space(const unsigned int col, const unsigned int row, const double depth, Matrix* point3);

    void project2Plane(const Matrix* point3, double* col, double* row);

    Matrix* getProjection(void){return(_projection);}

  private:

    Matrix* _projection;

};


#endif
