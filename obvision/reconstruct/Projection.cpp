/*
 * Projection.cpp
 * Contains all methods of the Projection class. This Class is part of the tsd_space.
 *
 *  Created on: 29.08.2012
 *      Author: phil
 */

#include "Projection.h"
#include "obcore/math/Matrix.h"
#include <math.h>
/*****************************************************************************************************************************/

namespace obvious
{

Projection::Projection(double *proj_data)
	{
	//Generate projection matrix
	_projection=new obvious::Matrix(3,4);
	_projection->setData(proj_data);
	std::cout<<"\nProjection Matrix has been set to\n"<<*_projection<<"\n";
	}

/*****************************************************************************************************************************/

double* Projection::operator [] (unsigned int i)
{
  return (*_projection)[i];
}

MSG Projection::get_point3(const unsigned int col,const unsigned int row,const double depth,obvious::Matrix *point3)
	{
	//Variables and Pointers
	double f;
	double T_x;
	double T_y;

	//get projection parameters
	f=(*_projection)[0][0];
	T_x=(*_projection)[0][2];
	T_y=(*_projection)[1][2];

	double x = (depth/f)*(col-T_x);
	double y = (depth/f)*(row-T_y);
	//double lambda_inv = 1./sqrt(x * x + y * y + 1.);
	//double z = depth * lambda_inv;

	//calculate point
	(*point3)[0][0]=x;
   (*point3)[1][0]=y;
   (*point3)[2][0]=depth;
   (*point3)[3][0]=1.0;

   return(OK);
	}

/*****************************************************************************************************************************/

MSG Projection::get_pxl(const obvious::Matrix *point3,unsigned int *col,unsigned int *row)
	{
	//Variables and pointers
	obvious::Matrix pxl_vecM(3,1);

	//calculate pixelvector
	pxl_vecM=*_projection*(*point3);

	//calculate pixel
	*col=(unsigned int)((pxl_vecM[0][0]/pxl_vecM[2][0])+0.5);
	*row=(unsigned int)((pxl_vecM[1][0]/pxl_vecM[2][0])+0.5);

	return(OK);

	}

/*****************************************************************************************************************************/

}
