#ifndef _PROJECTION_H
#define _PROJECTION_H

#include "obcore/math/Matrix.h"
#include "TsdSpace_utils.h"

namespace obvious
{

class Projection {
  public:

	 /**
	  * Konstruktor
	  * @param proj_data double poiner to parameters
	  */
    Projection(double* proj_data);

    inline ~Projection() {delete _projection;};

    double* operator [] (unsigned int i);

 	 /**
	  * Generates 3D-Point out of depth image data
	  * @param row,col pixel position in depth image
	  * @param depth value of depth image at given pixel
	  * @param point3 point with homogenous coordinates (x,y,z,1) has to be allocated by calling function
	  */
    MSG get_point3(const unsigned int col, const unsigned int row, const double depth, Matrix* point3);

    /**
    * get_pxl
    * @param point3 Vector that contains the point
    * @param col, row int-ptr to store the gained data in it
    */
    MSG get_pxl(const Matrix* point3, unsigned int* col, unsigned int* row);

    MSG project2Plane(const Matrix* point3, double* col, double* row);

    obvious::Matrix *getProjection(void){return(_projection);}

  private:
    obvious::Matrix * _projection;

};

}

#endif
