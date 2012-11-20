/*
 * Tsd_space.cpp
 *
 *  Created on: 20.07.2012
 *      Author: phil
 */

#include <iostream>
#include <cstring>
#include <cmath>
#include <cstdio>
#include "TsdSpace.h"
#include "TsdSpace_utils.h"
#include "obcore/base/tools.h"
#include "obcore/math/Matrix.h"
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include "Projection.h"
#include "obcore/base/System.h"
#include "obcore/math/mathbase.h"
#include <vector>
#include <omp.h>


#include "obcore/base/Logger.h"

#define ROW_MAX 480            // Sensor depth-array-dimensions
#define COL_MAX 640
#define MAXWEIGHT 32//1.6e308     // maximum weight (near end of double)
#define RGB_MAX 255           // 2⁸-1
#define UNUSED 0.999
#define M_UNUSED -0.999
#define SCALE 1.//0.25        // fixed scale for raycaster

#define KINFU_WEIGHTING 1
#define CORRECT_SIGN_CHANGE 1
#define INTERPOLATEDEPTH 0

namespace obvious
{

TsdSpace::TsdSpace(const unsigned int height, const unsigned int width,	const unsigned int depth,  const double voxelDim, double* perspective)
{
	// determine number of voxels in each dimension
	_xDim = round(width  / voxelDim);
	_yDim = round(height / voxelDim);
	_zDim = round(depth  / voxelDim);

	LOGMSG(DBG_DEBUG, "Dimensions are (x/y/z) (" << _xDim << "/" << _yDim << "/" << _zDim << ")");
	LOGMSG(DBG_DEBUG, "Creating TsdVoxel Space...");

	// allocate Space Voxel ***
	System<TsdVoxel>::allocate(_zDim, _yDim, _xDim, _space);

	LOGMSG(DBG_DEBUG, "Space for " << _xDim * _yDim * _zDim	<< " TsdVoxels has been created");

	// init _space Voxel***
	for (int z = 0; z < _zDim; z++)
	{
		for (int y = 0; y < _yDim; y++)
		{
			for (int x = 0; x < _xDim; x++)
			{
				_space[z][y][x].tsdf   = 1.0;
#if KINFU_WEIGHTING
				_space[z][y][x].weight = 0.0;
#else
				_space[z][y][x].weight = 1.0;
#endif
			}
		}
	}

	_voxelDim = voxelDim;
	_height   = height;
	_width    = width;
	_depth    = depth;

	_projection = new Projection(perspective);

	_T = new Matrix(4, 4);
	_T->setIdentity();

	_Tinv = new Matrix(4, 4);
	_Tinv->setIdentity();

	//Initialize space for depth image
	_depthImage = new double[COL_MAX * ROW_MAX];

	//initialize homogeneous point at origin
	_zero_h = new Matrix(4, 1);
	_zero_h->setZero();
	(*_zero_h)[3][0] = 1.0;
}

TsdSpace::~TsdSpace(void)
{
	delete [] _space;
	delete _projection;
	delete _T;
	delete _Tinv;
	delete [] _depthImage;
	delete _zero_h;
}

MSG TsdSpace::Push(double *depthImage)
{
	std::vector<unsigned int> sliceIdcs;
	std::vector<unsigned int> colIdcs;
	std::vector<unsigned int> rowIdcs;
	int xInd=0;
	int yInd=0;
	int zInd=0;
	int oldZind=0;
	unsigned int ctr=0;
	double rayPos[3]={0};
	double *dirVec = new double[3];
	double *footPoint = new double[4];

	//store current depth image in member variable
	memcpy(_depthImage, depthImage, COL_MAX * ROW_MAX * sizeof(double));

	if((calcRay(ROW_MAX/2, COL_MAX/2, &dirVec, &footPoint,HALFSTEP))!=OK)
		{
		std::cout<<"\nRecPush: Error Calculating ray!\n";
		exit(3);
		}

	for(unsigned int i=0;i<3;i++)
		rayPos[i]=footPoint[i];

	xInd = (unsigned int) (rayPos[0] / _voxelDim);
	yInd = (_yDim - 1) - (unsigned int) (rayPos[1] / _voxelDim);
	zInd = (unsigned int) (rayPos[2] / _voxelDim);
	colIdcs.push_back(xInd);
	rowIdcs.push_back(yInd);
	sliceIdcs.push_back(zInd);


	//start raycaster to determine which slices to check
	while(1)
	{
		oldZind=zInd;                     //store idx of last iteration
		for(unsigned int i=0;i<3;i++)
			rayPos[i]=footPoint[i]+(double)ctr*dirVec[i];

		// calculate current indices
		xInd = (unsigned int) (rayPos[0] / _voxelDim);
		yInd = (_yDim - 1) - (unsigned int) (rayPos[1] / _voxelDim);
		zInd = (unsigned int) (rayPos[2] / _voxelDim);

		if ((xInd >= _xDim) || (xInd < 0) || (yInd >= _yDim) || (yInd < 0) || (zInd < 0)||(zInd >= _zDim))
		{
		//	cout<<"\nRayPos (edge): (x/y/z) ("<<rayPos[0]<<"/"<<rayPos[1]<<"/"<<rayPos[2]<<")\n";
			break;
		}

		if(zInd!=oldZind)
		{
			colIdcs.push_back(xInd);
			rowIdcs.push_back(yInd);
			sliceIdcs.push_back(zInd);
			//cout<<"\nRayPos: (x/y/z) ("<<rayPos[0]<<"/"<<rayPos[1]<<"/"<<rayPos[2]<<")\n";
		}
		else
		{
			ctr++;
			continue;
		}
		ctr++;
	}

	//Testing give me slices
/*	std::cout<<"\nDetected slice idces: \n";
	for(unsigned int i=0;i<(unsigned int)sliceIdcs.size();i++)
		cout<<colIdcs[i]<<" "<<rowIdcs[i]<<" "<<sliceIdcs[i]<<"\n";*/

#pragma omp parallel for schedule(dynamic)
	for(unsigned int i=0;i<(unsigned int)sliceIdcs.size();i++)   //Create a recursive Search object for each detected slice
	{
		//RecursiveSearch recSearch(this,_projection,_space[sliceIdcs[i]],colIdcs[i],rowIdcs[i],sliceIdcs[i]);
		secSearch(colIdcs[i],rowIdcs[i],sliceIdcs[i]);
	}
	//RecursiveSearch(TsdSpace *curSpace,Projection *curProjection,TsdVoxel **curZslice,unsigned int colStart,unsigned int rowStart,
		//						 unsigned int zInd);
	return(OK);
}

MSG TsdSpace::secSearch(const unsigned int xIdx,const unsigned int yIdx,const unsigned int zIdx)
{
	unsigned int ctr=0;           //stores Number of Voxels found in Virt. Kinect-View per Iteration
	unsigned int xRange=0;         //used to
	unsigned int yRange=0;
	unsigned int row=0;
	unsigned int col=0;
	unsigned int u=0;
	unsigned int v=0;
	Matrix cVxlc(4,1);
	double cVxlcCrds[4];
	bool **sliceState=NULL;
	obvious::System<bool>::allocate(_yDim,_xDim,sliceState);

	for(int rows=0;rows<_yDim;rows++)
	{
		for(int cols=0;cols<_xDim;cols++)
		{
			sliceState[rows][cols]=0;           //init state array with 0 = Voxel has not been checked yet
		}
	}

	cVxlcCrds[2]=((double)zIdx+0.5)*_voxelDim;
	cVxlcCrds[3]=1.0;

	while(1)
	{
		for(row=yIdx-yRange-1;row<yIdx+yRange+1;row++)
		{
			for(col=xIdx-xRange-1;col<xIdx+xRange+1;col++)
			{
				if(sliceState[row][col])
					continue;
				else
					sliceState[row][col]=1;
				cVxlcCrds[0]=((double)col+0.5)*_voxelDim;
				cVxlcCrds[1]=((double)row+0.5)*_voxelDim;
				cVxlc.setData(cVxlcCrds);
				cVxlc=(*_Tinv)*cVxlc;
				_projection->get_pxl(&cVxlc,&u,&v);
				if((v < 0) || (v >= ROW_MAX))
					continue;
				if((u < 0) || (u >= COL_MAX))
					continue;

				ctr++;
				calcTsdf(cVxlcCrds,u,v,col,row,zIdx);
			}
		}
		if(ctr==0)
			break;
		else
			ctr=0;
		if(((row-1-(yRange+1))>=0)&&((row+1+yRange+1)<_yDim))
			yRange++;
		if(((col-1-(xRange+1))>=0)&&((col+1+xRange+1)<_xDim))
			xRange++;
	}
	return(OK);
}

void TsdSpace::setMaxTruncation(double val)
{
	if(val < 2 * _voxelDim)
	{
		LOGMSG(DBG_WARN, "Truncation radius must be at 2 x voxel dimension. Setting minimum size.");
		val = 2 * _voxelDim;
	}
	_maxTruncation = val;
}

MSG TsdSpace::calcTsdf(const double curVxlccoords[4],const unsigned int u,const unsigned int v,const unsigned int col,const unsigned int row,const unsigned int z)
{
	double t[3]={0};
	double distance;
	double xl;
	double yl;
	double lambdaInv;
	double tsdf;
	double sdf;
	TsdVoxel *voxel;

	//get current translation t
	for (unsigned int i = 0; i < 3; i++)
		t[i] = (*_T)[i][3];

	// calculate distance of current voxel to kinect
	distance = euklideanDistance<double>((double*) t,(double *)curVxlccoords, 3);

	xl = (u - (*_projection)[0][2]) / (*_projection)[0][0];
	yl = (v - (*_projection)[1][2]) / (*_projection)[1][1];
	lambdaInv = 1. / sqrt(xl * xl + yl * yl + 1.);
	sdf = distance * lambdaInv	- ((_depthImage[((ROW_MAX - 1) - v) * COL_MAX + u])); //mm

	//turn sign of the signed distance function
	sdf *= -1.0;

	if(sdf >= -_maxTruncation)         //Voxel is in front of an object
		{
		voxel = &_space[z][(_yDim - 1) - row][col]; //turn row idx bsp. _yDim=480, row =0 -> y_idx=479-0 = 479

		//truncate
		tsdf = sdf / _maxTruncation; //determine whether sdf/max_truncation = ]-1;1[ otherwise continue

		if(tsdf > 1.0)
			tsdf = 1.0;

		//Calculate Mean through weight
#if KINFU_WEIGHTING
		voxel->tsdf = (voxel->tsdf * voxel->weight + 1.0 * tsdf) / (voxel->weight + 1.0);
		voxel->weight += 1.0;
		if(voxel->weight > MAXWEIGHT)
			voxel->weight = MAXWEIGHT;
#else
		//weight it with depth of the voxel
		//increment weight
		if (voxel->tsdf > UNUSED) //First hit? no need to weight it
			voxel->tsdf = tsdf;
		else
		{
			double* weight_var = &voxel->weight;
			if (*weight_var < MAXWEIGHT)
				*weight_var += 1.0;
			voxel->tsdf = (voxel->tsdf * (*weight_var - 1.0) + tsdf) / (*weight_var);
		}
#endif



		}
	return(OK);
}

MSG TsdSpace::buildSliceImage(const unsigned int depthIndex, unsigned char* image)
{
	unsigned char R[_xDim * _yDim];
	unsigned char G[_xDim * _yDim];
	unsigned char B[_xDim * _yDim];
	unsigned int ctr = 0;
	unsigned im_ctr = 0;
	double c_tsdf;

	// initialize arrays for RGB
	for (int i = 0; i < _xDim * _yDim; i++)
	{
		R[i] = 0;
		G[i] = 0;
		B[i] = 0;
	}

	// iterate over given slice, generate 2D-Picture
	for (int row = 0; row < _yDim; row++)
	{
		for (int col = 0; col < _xDim; col++)
		{
			// Get current tsdf
			c_tsdf = _space[depthIndex][row][col].tsdf;

			// Blue for depth behind Voxel
			if (c_tsdf > 0)
			{
				if (c_tsdf > UNUSED)
					G[im_ctr++] = (unsigned char) 150; //row*_xDim+col
				else
					B[im_ctr++] = (unsigned char) (c_tsdf * RGB_MAX + 0.5); //row*_xDim+col
			}

			// Red for depth in front of Voxel
			else if (c_tsdf < 0)
			{
				if (c_tsdf < M_UNUSED)
					G[im_ctr++] = (unsigned char) 50;
				else
					R[im_ctr++] =
							(unsigned char) ((c_tsdf * -1.0) * RGB_MAX + 0.5); //row*_xDim+col
			}
		}
	}

	//put components together to complete picture
	for (int i = 0; i < _xDim * _yDim * 3; i++)
	{
		image[i]   = R[ctr];
		image[++i] = G[ctr];
		image[++i] = B[ctr];
		ctr++;
	}

	return (OK);
}

unsigned int TsdSpace::getXDimension()
{
	return _xDim;
}

unsigned int TsdSpace::getYDimension()
{
	return _yDim;
}

unsigned int TsdSpace::getZDimension()
{
	return _zDim;
}

double TsdSpace::getVxlDimension()
{
	return _voxelDim;
}

MSG TsdSpace::setTransformation(double *transM_data)
{
	//Variables and Pointers
	Matrix var_M(4, 4);
	Matrix h_var_M(4, 4);

	//_T->setData(transM_data);
	var_M.setData(transM_data);
	(*_T) *= var_M;
	h_var_M = *_T;
	h_var_M.invert();
	*_Tinv = h_var_M;

	return (OK);
}

MSG TsdSpace::generatePointcloud(double **cloud, double **normals,unsigned int *nbr)
{
	std::vector<double> pointcloud;
	std::vector<double> cloudNormals;
	double depth;
	double *point=new double[3];
	double *normal=new double[3];

	/**
	 * X_AXS parallel to X-Axis Borders : COL = _zDim, ROW = _yDim
	 * X_AXS_N parallel to X-Axis negative direction
	 * Y_AXS parallel to Y-Axis Borders : COL = _xDim, ROW = _zDim
	 * Y_AXS_N parallel to Y-Axis negative direction
	 * Z_AXS parallel to Z-Axis Borders : COL = _xDim, ROW = _yDim
	 * Z_AXS_N parallel to Z-Axis negative direction
	 **/

	//x_parallel
	cout<<"\nX_AXIS\n";
	for (int row = 0; row < _yDim; row++)
	{
		for (int col = 0; col < _zDim; col++)
		{
			if((rayCast(row,col,&point,normal,&depth,X_AXS))==OK)
			{
				for(int i=0;i<3;i++)
				{
					pointcloud.push_back(point[i]);
					cloudNormals.push_back(normal[i]);
				}
			}
			if((rayCast(row,col,&point,normal,&depth,X_AXS_N))==OK)
			{
				for(unsigned int i=0;i<3;i++)
				{
					pointcloud.push_back(point[i]);
					cloudNormals.push_back(normal[i]);
				}
			}
		}
	}

	//y_parallel
	cout<<"\nY_AXIS\n";
	for (int row = 0; row < _zDim; row++)
	{
		for (int col = 0; col < _xDim; col++)
		{
			if((rayCast(row,col,&point,normal,&depth,Y_AXS))==OK)
			{
				for(unsigned int i=0;i<3;i++)
				{
					pointcloud.push_back(point[i]);
					cloudNormals.push_back(normal[i]);
				}
			}
			if((rayCast(row,col,&point,normal,&depth,Y_AXS_N))==OK)
			{
				for(unsigned int i=0;i<3;i++)
				{
					pointcloud.push_back(point[i]);
					cloudNormals.push_back(normal[i]);
				}
			}
		}
	}

	//z_parallel
	cout<<"\nZ_AXIS\n";
	for (int row = 0; row < _yDim; row++)
	{
		for (int col = 0; col < _xDim; col++)
		{
			if((rayCast(row,col,&point,normal,&depth,Z_AXS))==OK)
			{
				for(unsigned int i=0;i<3;i++)
				{
					pointcloud.push_back(point[i]);
					cloudNormals.push_back(normal[i]);
				}
			}
			if((rayCast(row,col,&point,normal,&depth,Z_AXS_N))==OK)
			{
				for(unsigned int i=0;i<3;i++)
				{
					pointcloud.push_back(point[i]);
					cloudNormals.push_back(normal[i]);
				}
			}
		}
	}

	*nbr=pointcloud.size();
	*cloud=new double[*nbr];
	*normals=new double[*nbr];
	for(unsigned int i=0;i<*nbr;i++)
	{
		(*cloud)[i]=pointcloud[i];
		(*normals)[i]=cloudNormals[i];
	}

			/*	if ((peak(row, col, nbr, cloud)) != OK)
			{
				LOGMSG(DBG_ERROR, "Peak in : (row/col) (" << row << "/" << col << ")");
				continue;
			}*/


	return (OK);
}

MSG TsdSpace::calcRay(const unsigned int row, const unsigned int col, double **dir_vec, double **foot_point,RAYC_MODE mode)
{
	Matrix dirp_vecM(4, 1);
	Matrix dirf_vecM(4, 1);
	double abs_vec = 0.0;

	if((mode==GENERAL)||(mode==HALFSTEP))
	{
		//get point out of given pixel
		_projection->get_point3(col, row, SCALE, &dirp_vecM);

		//bring peakpoint in Map-coordinate-system
		dirp_vecM = *_T * dirp_vecM;

		//calculate foot point
		dirf_vecM = *_T * (*_zero_h); //bring footpoint in Map-coordinate-system
		dirf_vecM.getData(*foot_point); //give data to calling function

		//calculate amount of vector
		abs_vec = abs3D(&dirf_vecM, &dirp_vecM);

		//calculate vector normalized to voxeldimension
		//((PEAK-FOOT)/ABS)*VOXELDIMENSION)
		if(mode==GENERAL)
		{
			for(unsigned int i = 0; i < 3; i++)
				(*dir_vec)[i] = ((dirp_vecM[i][0] - dirf_vecM[i][0]) / abs_vec)	* _voxelDim;
		}
		else if(mode==HALFSTEP)
		{
			for(unsigned int i = 0; i < 3; i++)
				(*dir_vec)[i] = ((dirp_vecM[i][0] - dirf_vecM[i][0]) / abs_vec)	* 0.5*_voxelDim;
		}
		else
		{
			std::cout<<"\nFatal error...unnokwn Raycasting mode!\n";
			exit(3);
		}

	}

	else if((mode==X_AXS)||(mode==X_AXS_N))       //parallel to x_axis
	{
		(*foot_point)[1]=((double) row + 0.5) * _voxelDim;
		(*foot_point)[2]=((double) col + 0.5) * _voxelDim;
		(*dir_vec)[1]=0.0;
		(*dir_vec)[2]=0.0;
		if(mode==X_AXS)
		{
			(*foot_point)[0]=0.5f*_voxelDim;
			(*dir_vec)[0]=_voxelDim;
		}
		else
		{
			(*foot_point)[0]=((double)(_xDim-1) + 0.5) * _voxelDim;
			(*dir_vec)[0]=(-1.0)*_voxelDim;
		}
	}

	else if((mode==Y_AXS)||(mode==Y_AXS_N))       //parallel to x_axis
	{
		(*foot_point)[0]=((double) col + 0.5) * _voxelDim;
		(*foot_point)[2]=((double) row + 0.5) * _voxelDim;
		(*dir_vec)[0]=0.0;
		(*dir_vec)[2]=0.0;
		if(mode==X_AXS)
		{
			(*foot_point)[1]=0.5f*_voxelDim;
			(*dir_vec)[1]=_voxelDim;
		}
		else
		{
			(*foot_point)[1]=((double)(_yDim-1) + 0.5) * _voxelDim;
			(*dir_vec)[1]=(-1.0)*_voxelDim;
		}
	}

	else if((mode==Z_AXS)||(mode==Z_AXS_N))       //parallel to x_axis
	{
		(*foot_point)[0]=((double) col + 0.5) * _voxelDim;
		(*foot_point)[1]=((double) row + 0.5) * _voxelDim;
		(*dir_vec)[0]=0.0;
		(*dir_vec)[1]=0.0;
		if(mode==Z_AXS)
		{
			(*foot_point)[2]=0.5f*_voxelDim;
			(*dir_vec)[2]=_voxelDim;
		}
		else
		{
			(*foot_point)[2]=((double)(_zDim-1) + 0.5) * _voxelDim;
			(*dir_vec)[2]=(-1.0)*_voxelDim;
		}
	}

	else
	{
		std::cout<<"\nERROR. Mode "<<mode<<" not found!\n";
		return(ERROR);
	}

	return (OK);
}

MSG TsdSpace::rayCast(const unsigned int row, const unsigned int col, double **coordinates,double *normal,double *depth,RAYC_MODE mode)
{
	double *dir_vec = new double[3];
	double *foot_point = new double[4];
	double *position = new double[3];
	double *position_prev = new double[3];
	int x_idx = 0;
	int y_idx = 0;
	int z_idx = 0;
	unsigned int ctr = 0;

	foot_point[3] = 1.0;
	if (calcRay(row, col, &dir_vec, &foot_point,mode) != OK)
	{
		LOGMSG(DBG_DEBUG, "Error calculating ray (row/col) (" << row << "/" << col << ")");
		delete dir_vec;
		delete foot_point;
		return (ERROR);
	}

	// Interpolation weight
	double interp;

	memcpy(position, foot_point, 3 * sizeof(*position));
	while (1)
	{
		// calculate current position
		memcpy(position_prev, position, 3 * sizeof(*position));

		for (unsigned int i = 0; i < 3; i++)
			position[i] = foot_point[i] + ((double) ctr) * (dir_vec[i]);

		// calculate current indices
		x_idx = (int) (position[0] / _voxelDim);
		y_idx = (_yDim - 1) - (int) (position[1] / _voxelDim);
		z_idx = (int) (position[2] / _voxelDim);

		// check whether RayCaster is in space or not
		if ((x_idx >= _xDim) || (x_idx < 0) || (y_idx >= _yDim) || (y_idx < 0) || (z_idx < 0)||(z_idx >= _zDim)) // rayCaster reached edge of space
		{
			delete dir_vec;
			delete foot_point;
			delete position;
			return EDGE;
		}

		/*if(z_idx < 0) // RayCaster is not in space
		{
			ctr++;
			continue;
		}

		if(ctr>100)
		{
			delete dir_vec;
			delete foot_point;
			delete position;
			return EDGE;
		}*/
		double tsdf;
		if (interpolateTrilinear(&position, &tsdf) != OK)
		{
//			std::cout<<"\ninterpolation of tsdf failed...\n";
//				std::printf("\n...at (%lf/%lf/%lf)\n",position[0],position[1],position[2]);

			ctr++;
			continue;
		}

		double tsdf_prev;
		if (interpolateTrilinear(&position_prev, &tsdf_prev) != OK)
		{
//			std::cout<<"\ninterpolation of tsdf_prev failed...\n";
	//		std::printf("\n...at (%lf/%lf/%lf)\n",position[0],position[1],position[2]);
			ctr++;
			continue;
		}

		// check sign change
		if(tsdf_prev > 0 && tsdf < 0)
		{
			interp = tsdf_prev / (tsdf_prev - tsdf);
			break;
		}

		//memcpy(position_prev, position, 3 * sizeof(*position));
		// increment counter
		ctr++;
	}

	// interpolate between voxels when sign change happened
	for (unsigned int i = 0; i < 3; i++)
		(*coordinates)[i] = position_prev[i] + dir_vec[i] * interp;

	if(getNormal(*coordinates,normal)!=OK)
		return(ERROR);

	delete dir_vec;
	delete foot_point;
	delete position;
	return (OK);
}

MSG TsdSpace::getNormal(const double *crosCoords,double *normalCoords)
{
	//Point pVar;
	double *coordVar=new double[3];
	double depthVarInc=0;
	double depthVarDec=0;
	double abs=0;

	coordVar[0]=crosCoords[0]+_voxelDim;        //interpolate around Voxel in x+1 direction
	coordVar[1]=crosCoords[1];
	coordVar[2]=crosCoords[2];

	if(interpolateTrilinear(&coordVar,&depthVarInc)!=OK)
	{
	//	std::cout<<"\nError Interpolation Normals-Estimator...\n";
		return(ERROR);
	}

	coordVar[0]=crosCoords[0]-_voxelDim;        //interpolate around Voxel in x-1 direction
	coordVar[1]=crosCoords[1];
	coordVar[2]=crosCoords[2];

	if(interpolateTrilinear(&coordVar,&depthVarDec)!=OK)
	{
	//	std::cout<<"\nError Interpolation Normals-Estimator...\n";
		return(ERROR);
	}

	normalCoords[0]=depthVarInc-depthVarDec;     //x_coordinate of the normal

	coordVar[0]=crosCoords[0];
	coordVar[1]=crosCoords[1]+_voxelDim;    //interpolate around Voxel in y+1 direction
	coordVar[2]=crosCoords[2];

	if(interpolateTrilinear(&coordVar,&depthVarInc)!=OK)
	{
	//	std::cout<<"\nError Interpolation Normals-Estimator...\n";
		return(ERROR);
	}

	coordVar[0]=crosCoords[0];
	coordVar[1]=crosCoords[1]-_voxelDim;    //interpolate around Voxel in y-1 direction
	coordVar[2]=crosCoords[2];

	if(interpolateTrilinear(&coordVar,&depthVarDec)!=OK)
	{
	//	std::cout<<"\nError Interpolation Normals-Estimator...\n";
		return(ERROR);
	}

	normalCoords[1]=depthVarInc-depthVarDec;     //y_coordinate of the normal

	coordVar[0]=crosCoords[0];
	coordVar[1]=crosCoords[1];
	coordVar[2]=crosCoords[2]+_voxelDim;         //interpolate around Voxel in z+1 direction

	if(interpolateTrilinear(&coordVar,&depthVarInc)!=OK)
	{
		//std::cout<<"\nError Interpolation Normals-Estimator...\n";
		return(ERROR);
	}

	coordVar[0]=crosCoords[0];
	coordVar[1]=crosCoords[1];
	coordVar[2]=crosCoords[2]-_voxelDim;         //interpolate around Voxel in z-1 direction

	if(interpolateTrilinear(&coordVar,&depthVarDec)!=OK)
	{
		//std::cout<<"\nError Interpolation Normals-Estimator...\n";
		return(ERROR);
	}

	normalCoords[2]=depthVarInc-depthVarDec;     //z_coordinate of the normal

	for(unsigned int i=0;i<3;i++)
		abs+=normalCoords[i]*normalCoords[i];

	abs=std::sqrt(abs);

	for(unsigned int i=0;i<3;i++)
		normalCoords[i]/=abs;

	delete coordVar;

	return(OK);
}

MSG TsdSpace::getModel(double **modelPcl,double *modelNormals, unsigned int *ctr)
{
	double *p_var    = new double[3];
	double *nVar=new double[3];
	bool found       = 0;
	double depth_var = 0.0;
	obvious::Matrix Mvar(4,1);

	// allocate space for pointcloud
	//if(!model_pcl)
		*modelPcl = new double[ROW_MAX * COL_MAX * 3];
	//LOGMSG(DBG_DEBUG, "Space allocated. Starting Raytracing...");

	//reset counter
	*ctr = 0;

	/*
	 * Room for Multithreading shall use up to 12 Threads which calculate a ray each
	 */
	for (unsigned int row = 0; row < ROW_MAX; row++)
	{
		for (unsigned int col = 0; col < COL_MAX; col++)
		{
		//	cout<<"\n(ROW/COL) ("<<row<<"/"<<col<<")\n";
			if (  (rayCast(row, col, &p_var,nVar,&depth_var,GENERAL)) == OK) //Ray returned with coordinates
			{
				found = 1;
				Mvar.setData(p_var);
				Mvar[3][0]=1.0;
				Mvar=*_Tinv*Mvar;
				for (unsigned int i = 0; i < 3; i++)
				{
					(*modelPcl)[*ctr] = Mvar[i][0];
					modelNormals[(*ctr)++]=nVar[i];
				}
			}
	//		else
//				cout<<"\nRaycaster returned with state : "<<racstate<<" \n";
		}
	}
	if (found)
		--(*ctr);

	LOGMSG(DBG_DEBUG, "Raycasting finished! Found " << *ctr << " coordinates.");

	delete p_var;
	delete nVar;
	return (OK);
}

Matrix* TsdSpace::getTransformation()
{
	return(_T);
}

Matrix* TsdSpace::getinvTransformation()
{
	return(_Tinv);
}

MSG TsdSpace::interpolateTrilinear(double **coordinates, double *tsdf)
{
	int x_idx = 0;
	int y_idx = 0;
	int z_idx = 0;
	double w_x = 0;
	double w_y = 0;
	double w_z = 0;
	Point c_vxl_mp;

	// initialize
	// get voxel indices
	x_idx = (int) (((*coordinates)[0] / _voxelDim));
	y_idx = (int) (((*coordinates)[1] / _voxelDim));
	z_idx = (int) (((*coordinates)[2] / _voxelDim));

	// check edges / 0 is edge because of voxelfinetuning
	if ((x_idx >= (_xDim - 2)) || (x_idx < 1) || (y_idx >= (_yDim - 2)) || (y_idx < 1) || (z_idx >= (_zDim - 2)) || (z_idx < 1))
		return (EDGE);

	// get current voxel middlepoint
	c_vxl_mp.x = (double(x_idx) + 0.5) * _voxelDim;
	c_vxl_mp.y = (double(y_idx) + 0.5) * _voxelDim;
	c_vxl_mp.z = (double(z_idx) + 0.5) * _voxelDim;

	// voxel fine tuning
	if ((*coordinates)[0] < c_vxl_mp.x)
	{
		x_idx--;
		c_vxl_mp.x = (double(x_idx) + 0.5) * _voxelDim;
	}
	if ((*coordinates)[1] < c_vxl_mp.y)
	{
		y_idx--;
		c_vxl_mp.y = (double(y_idx) + 0.5) * _voxelDim;
	}
	if ((*coordinates)[2] < c_vxl_mp.z)
	{
		z_idx--;
		c_vxl_mp.z = (double(z_idx) + 0.5) * _voxelDim;
	}

	// turn y-axis
	y_idx = (_yDim - 1) - y_idx;

	if (_space[z_idx + 0][y_idx + 0][x_idx + 0].tsdf > UNUSED) return EDGE;
	if (_space[z_idx + 1][y_idx + 0][x_idx + 0].tsdf > UNUSED) return EDGE;
	if (_space[z_idx + 0][y_idx - 1][x_idx + 0].tsdf > UNUSED) return EDGE;
	if (_space[z_idx + 1][y_idx - 1][x_idx + 0].tsdf > UNUSED) return EDGE;
	if (_space[z_idx + 0][y_idx + 0][x_idx + 1].tsdf > UNUSED) return EDGE;
	if (_space[z_idx + 1][y_idx + 0][x_idx + 1].tsdf > UNUSED) return EDGE;
	if (_space[z_idx + 0][y_idx - 1][x_idx + 1].tsdf > UNUSED) return EDGE;
	if (_space[z_idx + 1][y_idx - 1][x_idx + 1].tsdf > UNUSED) return EDGE;

	if (_space[z_idx + 0][y_idx + 0][x_idx + 0].tsdf < M_UNUSED) return EDGE;
	if (_space[z_idx + 1][y_idx + 0][x_idx + 0].tsdf < M_UNUSED) return EDGE;
	if (_space[z_idx + 0][y_idx - 1][x_idx + 0].tsdf < M_UNUSED) return EDGE;
	if (_space[z_idx + 1][y_idx - 1][x_idx + 0].tsdf < M_UNUSED) return EDGE;
	if (_space[z_idx + 0][y_idx + 0][x_idx + 1].tsdf < M_UNUSED) return EDGE;
	if (_space[z_idx + 1][y_idx + 0][x_idx + 1].tsdf < M_UNUSED) return EDGE;
	if (_space[z_idx + 0][y_idx - 1][x_idx + 1].tsdf < M_UNUSED) return EDGE;
	if (_space[z_idx + 1][y_idx - 1][x_idx + 1].tsdf < M_UNUSED) return EDGE;

	// get weights
	w_x = ((*coordinates)[0] - c_vxl_mp.x) / _voxelDim;
	w_y = ((*coordinates)[1] - c_vxl_mp.y) / _voxelDim;
	w_z = ((*coordinates)[2] - c_vxl_mp.z) / _voxelDim;

	// Interpolate
	*tsdf =   _space[z_idx + 0][y_idx + 0][x_idx + 0].tsdf * (1 - w_x) * (1 - w_y) * (1 - w_z)
        	+ _space[z_idx + 1][y_idx + 0][x_idx + 0].tsdf * (1 - w_x) * (1 - w_y) * w_z
        	+ _space[z_idx + 0][y_idx - 1][x_idx + 0].tsdf * (1 - w_x) * w_y * (1 - w_z)
        	+ _space[z_idx + 1][y_idx - 1][x_idx + 0].tsdf * (1 - w_x) * w_y * w_z
        	+ _space[z_idx + 0][y_idx + 0][x_idx + 1].tsdf * w_x * (1 - w_y) * (1 - w_z)
        	+ _space[z_idx + 1][y_idx + 0][x_idx + 1].tsdf * w_x * (1 - w_y) * w_z
        	+ _space[z_idx + 0][y_idx - 1][x_idx + 1].tsdf * w_x * w_y * (1 - w_z)
        	+ _space[z_idx + 1][y_idx - 1][x_idx + 1].tsdf * w_x * w_y * w_z;

	return (OK);
}

double TsdSpace::interpolateBilinear(double u, double v)
{
	v = ((double)ROW_MAX-1) - v;
	unsigned int nu = (unsigned int)u;
	unsigned int nv = (unsigned int)v;

	double du = u - (double)nu;
	double dv = v - (double)nv;

	//cout << u << " " << du << " " << v << " " << dv << endl;
	return _depthImage[nv*COL_MAX + nu]         * (1.-du)* (1.-dv) +
		   _depthImage[nv*COL_MAX + nu + 1]     *     du * (1.-dv) +
		   _depthImage[(nv+1)*COL_MAX + nu]     * (1.-du)*     dv  +
		   _depthImage[(nv+1)*COL_MAX + nu + 1] *     du *     dv;
}

} // end namespace
