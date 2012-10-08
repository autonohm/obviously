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

#include "obcore/base/Logger.h"

#define ROW_MAX 480            //Kinect camera depth-array-dimensions
#define COL_MAX 640
#define MAXWEIGHT 1.6e308     //maximum weight (near end of double)
#define RGB_MAX 255           //2⁸-1
#define UNUSED 0.999
#define M_UNUSED -0.999
#define SCALE 1.//0.25              //fixed scale for raycaster
/*****************************************************************************************************************************/
namespace obvious
{

TsdSpace::TsdSpace(const unsigned int height, const unsigned int width,
		const unsigned int depth, const double vxldimension, double* perspective)
{
	//Variables and Pointers
	_x_nbr = round(width / vxldimension); //determine nbr of voxels in each dimension
	_y_nbr = round(height / vxldimension);
	_z_nbr = round(depth / vxldimension);

	LOGMSG(DBG_DEBUG, "\nDimensions are (x/y/z) (" << _x_nbr << "/" << _y_nbr << "/" << _z_nbr << ")\n");
	LOGMSG(DBG_WARN, "\nCreating TsdVoxel Space...\n");

	//allocate Space Voxel ***
	obvious::System<TsdVoxel>::allocate(_z_nbr, _y_nbr, _x_nbr, _space);

	std::cout << "\nSpace for " << _x_nbr * _y_nbr * _z_nbr	<< " TsdVoxels has been created\n\nInitializing Space...\n";

	//init _space Voxel***
	for (unsigned int z = 0; z < _z_nbr; z++)
	{
		for (unsigned y = 0; y < _y_nbr; y++)
		{
			for (unsigned int x = 0; x < _x_nbr; x++)
			{
				_space[z][y][x].tsdf = 1.0;
				_space[z][y][x].weight = 1.0;
			}
		}
	}

	//store space dimensions
	std::cout << "\nSaving Spacedimensions...\n";

	_voxeldimension = vxldimension;
	_height = height;
	_width = width;
	_depth = depth;

	//initialize projection class
	std::cout << "\nInitializing projection class...\n";
	_projection = new Projection(perspective);

	//initalize viewer
	//	std::cout<<"\nInitializing viewer...\n";
	//	_tsd_viewer=new TSD_Viewer(_projection);

	//initialize transformation Matrix with identity
	_transformation = new obvious::Matrix(4, 4);
	*_transformation = _transformation->identity();

	//initialize inv_transformation Matrix with identity
	_inv_transformation = new obvious::Matrix(4, 4);
	*_inv_transformation = _inv_transformation->identity();

	//initalize space for depth image
	_act_depth_image = new double[COL_MAX * ROW_MAX];

	//initialize zero point
	_zero_h = new obvious::Matrix(4, 1);
	for (unsigned int i = 0; i < 3; i++)
		(*_zero_h)[i][0] = 0.0;
	(*_zero_h)[3][0] = 1.0;

}

/*****************************************************************************************************************************/

TsdSpace::~TsdSpace(void)
{
	delete[] _space;
	delete _projection;
	delete _transformation;
	delete _inv_transformation;
	//	delete _act_depth_image;
	delete _zero_h;
}

/*****************************************************************************************************************************/

MSG
TsdSpace::push(double *depth_image)
{
	//store current depth image in member variable
	cout << "\nPUSH: Store current depth image...\n";
	memcpy(_act_depth_image, depth_image, COL_MAX * ROW_MAX * sizeof(double));

	/*
	 * Room for Multithreading shall use up to 12 Threads which calculate a slide each
	 */
	for (unsigned int i = 0; i < _z_nbr; i++)
	{
		if (depth_slice(i) != OK)
			std::cout << "\nPUSH: Error slice nr. " << i << "\n";
	}

	return (OK);
}

/*****************************************************************************************************************************/

MSG
TsdSpace::depth_slice(const unsigned int depth)
{
	//Variables and Pointers
	double coordVoxel[4];
	double t[3];
	double distance;
	double sdf;
	double tsdf;
	double *weight_var;
	obvious::Matrix mCoordVoxel(4, 1);
	TsdVoxel *voxel;

	//initialize
	coordVoxel[2] = ((double) depth + 0.5) * _voxeldimension; //center point of all voxels in given z-slice
	coordVoxel[3] = 1.0; //homogeneous coordinates

	/*
	 * Start main loop
	 */
	//Loop over the z-slice given in depth
	for (unsigned int row = 0; row < _y_nbr; row++)
	{
		for (unsigned int col = 0; col < _x_nbr; col++)
		{
			//calculate center point of current voxel
			coordVoxel[0] = ((double) col + 0.5) * _voxeldimension;
			coordVoxel[1] = ((double) row + 0.5) * _voxeldimension;

			//generate vector
			mCoordVoxel.setData(coordVoxel);

			//Transform coordinates of current voxel in Kinect-CS
			mCoordVoxel = *_inv_transformation * mCoordVoxel;

			//Get current Pixel
			unsigned int u;
			unsigned int v;
			_projection->get_pxl(&mCoordVoxel, &u, &v);

			if ((v > 0) && (v < ROW_MAX))
			{
				if ((u > 0) && (u < COL_MAX))
				{
					if (_act_depth_image[((ROW_MAX - 1) - v) * COL_MAX + u]
					                     < 0.001) //invalid point-->test?
					                    		 continue;
				}
				else
					//col not in frustum
					continue;
			}
			else
				//row not in frustum
				continue;

			//current voxel in Kinect frustrum -> Calculate TSDF
			//get current translation_vector to kinect
			for (unsigned int i = 0; i < 3; i++)
				t[i] = (*_transformation)[i][3];

			//calculate distance of current voxel to kinect
			distance = euklideanDistance<double>((double*) t,
					(double*) coordVoxel, 3);

			//calculate signed distance function
			//sdf=distance-((_act_depth_image[((ROW_MAX-1)-v)*COL_MAX+u]));     //sdf(i)=abs(t(i)-v(g))-D(i)(p) /Kinect Z-Image = millimeters!FUCK!!

			float xl = (u - (*_projection)[0][2]) / (*_projection)[0][0];
			float yl = (v - (*_projection)[1][2]) / (*_projection)[1][1];
			float lambda_inv = 1. / sqrt(xl * xl + yl * yl + 1.);

			sdf = distance * lambda_inv
					- ((_act_depth_image[((ROW_MAX - 1) - v) * COL_MAX + u])); //mm
					sdf *= -1.0;

					//calculate truncated sdf
					//get ptr to current TsdVoxel
					voxel = &_space[depth][(_y_nbr - 1) - row][col]; //turn row idx bsp. _y_nbr=480, row =0 -> y_idx=479-0 = 479

					//truncate
					tsdf = sdf / _max_truncation; //determine whether sdf/max_truncation = ]-1;1[ otherwise continue

					if ((tsdf < -1.0) || (tsdf > 1.0))
						continue;

					//weight it with depth of the voxel
					//increment weight
					if (voxel->tsdf > UNUSED) //First hit? no need to weight it
						voxel->tsdf = tsdf;
					else
					{
						weight_var = &voxel->weight;
						if (*weight_var < MAXWEIGHT)
							*weight_var += 1.0;
						voxel->tsdf = (voxel->tsdf * (*weight_var - 1.0) + tsdf)
                    		/ (*weight_var);
					}

		}
	}

	return (OK);
}

/*****************************************************************************************************************************/

MSG
TsdSpace::buildSliceImage(const unsigned int depthIndex, unsigned char* image)
{
	//Variables and Pointers
	char path[90];
	unsigned char R[_x_nbr * _y_nbr];
	unsigned char G[_x_nbr * _y_nbr];
	unsigned char B[_x_nbr * _y_nbr];
	unsigned int ctr = 0;
	unsigned im_ctr = 0;
	TsdVoxel *c_vxlPtr;
	double c_tsdf;

	//initialize arrays for RGB
	for (unsigned int i = 0; i < _x_nbr * _y_nbr; i++)
	{
		R[i] = 0;
		G[i] = 0;
		B[i] = 0;
	}
	/*
	 * Main loop
	 */

	//iterate over given slice, generate 2D-Picture
	for (int row = 0; row < _y_nbr; row++)
	{
		for (int col = 0; col < _x_nbr; col++)
		{
			//Get current tsdf
			c_tsdf = _space[depthIndex][row][col].tsdf;

			//Blue for depth behind Voxel
			if (c_tsdf > 0)
			{
				if (c_tsdf > UNUSED)
					G[im_ctr++] = (unsigned char) 150; //row*_x_nbr+col
				else
					B[im_ctr++] = (unsigned char) (c_tsdf * RGB_MAX + 0.5); //row*_x_nbr+col
			}

			//Red for depth in front of Voxel
			else if (c_tsdf < 0)
			{
				if (c_tsdf < M_UNUSED)
					G[im_ctr++] = (unsigned char) 50;
				else
					R[im_ctr++] =
							(unsigned char) ((c_tsdf * -1.0) * RGB_MAX + 0.5); //row*_x_nbr+col
			}
		}
	}

	//put components together to complete picture
	for (unsigned int i = 0; i < _x_nbr * _y_nbr * 3; i++)
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
	return _x_nbr;
}

unsigned int TsdSpace::getYDimension()
{
	return _y_nbr;
}

unsigned int TsdSpace::getZDimension()
{
	return _z_nbr;
}

MSG
TsdSpace::set_transformation(double *transM_data)
{
	//Variables and Pointers
	obvious::Matrix var_M(4, 4);
	obvious::Matrix h_var_M(4, 4);

	//_transformation->setData(transM_data);
	var_M.setData(transM_data);
	(*_transformation) *= var_M;
	h_var_M = *_transformation;
	h_var_M.invert();
	*_inv_transformation = h_var_M;

	return (OK);
}

/*****************************************************************************************************************************/

MSG
TsdSpace::peak(unsigned int row, unsigned int col, unsigned int *nbr,
		double **coordinates)
{
	//Variables and pointers
	double c_tsdf = 0.0;
	double n_tsdf = 0.0;
	const double X_POS = ((double) col + 0.5) * _voxeldimension; //const vars to store x,y coordinates of peak
	const double Y_POS = ((double) row + 0.5) * _voxeldimension;
	const unsigned int Y_IND = (_y_nbr - 1) - row;
	const unsigned int Z_MAX = _z_nbr - 1;

	/*
	 * Main loop
	 */
	//Iterate through peak
	for (unsigned int i = 0; i < Z_MAX; i++)
	{

		//compare the signs
		c_tsdf = _space[i][Y_IND][col].tsdf;
		n_tsdf = _space[i + 1][Y_IND][col].tsdf;

		if ((c_tsdf < 0 && c_tsdf > M_UNUSED)
				&& (n_tsdf > 0 && n_tsdf < UNUSED))
		{
			//Store coordinates
			(*coordinates)[(*nbr)++] = X_POS;
			(*coordinates)[(*nbr)++] = Y_POS;
			(*coordinates)[(*nbr)++] = ((double) i + 0.5) * _voxeldimension;
			//cout << X_POS << " "  << Y_POS << " " << ((double)i+0.5)*_voxeldimension << " " << c_tsdf << endl;
		}
	}
	return (OK);
}

/*****************************************************************************************************************************/

MSG
TsdSpace::gen_pcl(double **cloud, unsigned int *nbr)
{
	//initialize
	//allocate space for pointcloud
	*cloud = new double[_x_nbr * _y_nbr * _z_nbr * 3];

	/*
	 * Main loop
	 */

	/*
	 * Room for Multithreading shall use up to 12 Threads which calculate a peak each
	 */

	//send peaks through space
	for (unsigned int row = 0; row < _y_nbr; row++)
	{
		for (unsigned int col = 0; col < _x_nbr; col++)
		{
			if ((peak(row, col, nbr, cloud)) != OK)
			{
				std::cout << "\nError Peak in : (row/col) (" << row << "/"
						<< col << ")\n";
				continue;
			}
		}
	}
	return (OK);
}

/*****************************************************************************************************************************/

MSG
TsdSpace::calc_ray(const unsigned int row, const unsigned int col,
		double **dir_vec, double **foot_point)
{
	//Variables and pointers
	obvious::Matrix dirp_vecM(4, 1);
	obvious::Matrix dirf_vecM(4, 1);
	double abs_vec = 0.0;

	//get point out of given pixel
	_projection->get_point3(col, row, SCALE, &dirp_vecM);

	//bring peakpoint in Map-coordinate-system
	dirp_vecM = *_transformation * dirp_vecM;

	//calculate foot point
	dirf_vecM = *_transformation * (*_zero_h); //bring footpoint in Map-coordinate-system
	dirf_vecM.getData(*foot_point); //give data to calling function

	//calculate amount of vector
	abs_vec = abs3D(&dirf_vecM, &dirp_vecM);

	//calculate vector normalized to voxeldimension
	//((PEAK-FOOT)/ABS)*VOXELDIMENSION)
	for (unsigned int i = 0; i < 3; i++)
		(*dir_vec)[i] = ((dirp_vecM[i][0] - dirf_vecM[i][0]) / abs_vec)
		* _voxeldimension;

	return (OK);
}

/*****************************************************************************************************************************/

MSG
TsdSpace::get_model(double **model_pcl, unsigned int *ctr)
{
	//Variables and pointers
	double *p_var = new double[3];
	bool found = 0;
	double depth_var = 0.0;

	//initalize
	//allocate space for pointcloud
	*model_pcl = new double[ROW_MAX * COL_MAX * 3];
	std::cout << "\nGET_MODEL: Space allocated. Starting Raytracing...\n";
	//reset counter
	*ctr = 0;

	/*
	 * Main loop
	 */

	/*
	 * Room for Multithreading shall use up to 12 Threads which calculate a ray each
	 */

	for (unsigned int row = 0; row < ROW_MAX; row++)
	{
		for (unsigned int col = 0; col < COL_MAX; col++)
		{
			if (ray_trace(row, col, &p_var, &depth_var) == OK) //Ray returned with coordinates
			{
				found = 1;
				for (unsigned int i = 0; i < 3; i++)
					(*model_pcl)[(*ctr)++] = p_var[i];
			}
		}
	}
	if (found)
		--(*ctr);
	std::cout << "\nGET_MODEL: Raytracing finished! Found " << *ctr
			<< " coordinates.\n";

	delete p_var;
	return (OK);
}

/*****************************************************************************************************************************/

MSG
TsdSpace::ray_trace(const unsigned int row, const unsigned int col,
		double **coordinates, double *depth)
{
	//Variables and Pointers
	double *dir_vec = new double[3];
	double *foot_point = new double[4];
	double *position = new double[3];
	double *position_prev = new double[3];
	double c_tsdf = 1.0;
	double l_tsdf = 1.0;
	int x_idx = 0;
	int y_idx = 0;
	int z_idx = 0;
	unsigned int ctr = 0;

	//initialize
	//homogeneous coordinates
	foot_point[3] = 1.0;
	//calc direction vector
	if (calc_ray(row, col, &dir_vec, &foot_point) != OK)
	{
		std::cout << "\nRAY_TRACE: Error calculating ray (row/col) (" << row
				<< "/" << col << ")\n";
		delete dir_vec;
		delete foot_point;
		return (ERROR);
	}

	// Interpolation weight
	double interp;

	/*
	 * Main loop
	 */
	while (1)
	{
		//calculate current position
		memcpy(position_prev, position, 3 * sizeof(*position));

		for (unsigned int i = 0; i < 3; i++)
			position[i] = foot_point[i] + ((double) ctr) * (dir_vec[i]);

		//calculate current indices
		x_idx = (int) (position[0] / _voxeldimension);
		y_idx = (_y_nbr - 1) - (int) (position[1] / _voxeldimension);
		z_idx = (int) (position[2] / _voxeldimension);

		//check whether tracer is in space or not
		if ((x_idx >= _x_nbr) || (x_idx < 0) || (y_idx >= _y_nbr) || (y_idx < 0)
				|| (z_idx >= _z_nbr) || (z_idx < 0)) //ratraycer reached edge of space
		{
			delete dir_vec;
			delete foot_point;
			delete position;
			return (EDGE);
		}

		//get current tsdf
		c_tsdf = _space[z_idx][y_idx][x_idx].tsdf;

		//check sign change
		if ((l_tsdf > 0 && l_tsdf < UNUSED)	&& (c_tsdf < 0 && c_tsdf > M_UNUSED))
		{
			double tsdf;
			if (interpolate_trilineary(&position, &tsdf) != OK)
			{
				delete dir_vec;
				delete foot_point;
				delete position;
				return (ERROR);
			}

			double tsdf_prev;
			if (interpolate_trilineary(&position_prev, &tsdf_prev) != OK)
			{
				delete dir_vec;
				delete foot_point;
				delete position;
				return (ERROR);
			}

			interp = tsdf_prev / (tsdf_prev - tsdf);

			break;
		}
		//store current tsdf in last tsdf
		l_tsdf = c_tsdf;

		//increment counter
		ctr++;
	}

	// interpolate between voxels when sign change happened
	for (unsigned int i = 0; i < 3; i++)
		(*coordinates)[i] = position_prev[i] + dir_vec[i] * interp;

	delete dir_vec;
	delete foot_point;
	delete position;
	return (OK);
}

/*****************************************************************************************************************************/

/*****************************************************************************************************************************/

// Return value is the true signed distance!
MSG
TsdSpace::interpolate_trilineary(double **coordinates, double *tsdf)
{
	//Variables and pointers
	int x_idx = 0;
	int y_idx = 0;
	int z_idx = 0;
	double w_x = 0;
	double w_y = 0;
	double w_z = 0;
	double abs = 0;
	double str_cr_val = 0;
	Point c_vxl_mp;

	//initialize
	//get voxel indices
	x_idx = (int) (((*coordinates)[0] / _voxeldimension));
	y_idx = (int) (((*coordinates)[1] / _voxeldimension));
	z_idx = (int) (((*coordinates)[2] / _voxeldimension));

	//check edges / 0 is edge because of voxelfinetuning
	if ((x_idx >= (_x_nbr - 2)) || (x_idx < 1) || (y_idx >= (_y_nbr - 2))
			|| (y_idx < 1) || (z_idx >= (_z_nbr - 2)) || (z_idx < 1))
		return (EDGE);

	//get current voxel middlepoint
	c_vxl_mp.x = (double(x_idx) + 0.5) * _voxeldimension;
	c_vxl_mp.y = (double(y_idx) + 0.5) * _voxeldimension;
	c_vxl_mp.z = (double(z_idx) + 0.5) * _voxeldimension;

	//voxelfinetuning
	if ((*coordinates)[0] < c_vxl_mp.x)
	{
		x_idx--;
		c_vxl_mp.x = (double(x_idx) + 0.5) * _voxeldimension;
	}
	if ((*coordinates)[1] < c_vxl_mp.y)
	{
		y_idx--;
		c_vxl_mp.y = (double(y_idx) + 0.5) * _voxeldimension;
	}
	if ((*coordinates)[2] < c_vxl_mp.z)
	{
		z_idx--;
		c_vxl_mp.z = (double(z_idx) + 0.5) * _voxeldimension;
	}

	//turn y-axis
	y_idx = (_y_nbr - 1) - y_idx;

	if (_space[z_idx + 0][y_idx + 0][x_idx + 0].tsdf > 0.99) return (EDGE);
	if (_space[z_idx + 1][y_idx + 0][x_idx + 0].tsdf > 0.99) return (EDGE);
	if (_space[z_idx + 0][y_idx - 1][x_idx + 0].tsdf > 0.99) return (EDGE);
	if (_space[z_idx + 1][y_idx - 1][x_idx + 0].tsdf > 0.99) return (EDGE);
	if (_space[z_idx + 0][y_idx + 0][x_idx + 1].tsdf > 0.99) return (EDGE);
	if (_space[z_idx + 1][y_idx + 0][x_idx + 1].tsdf > 0.99) return (EDGE);
	if (_space[z_idx + 0][y_idx - 1][x_idx + 1].tsdf > 0.99) return (EDGE);
	if (_space[z_idx + 1][y_idx - 1][x_idx + 1].tsdf > 0.99) return (EDGE);

	if (_space[z_idx + 0][y_idx + 0][x_idx + 0].tsdf < -0.99) return (EDGE);
	if (_space[z_idx + 1][y_idx + 0][x_idx + 0].tsdf < -0.99) return (EDGE);
	if (_space[z_idx + 0][y_idx - 1][x_idx + 0].tsdf < -0.99) return (EDGE);
	if (_space[z_idx + 1][y_idx - 1][x_idx + 0].tsdf < -0.99) return (EDGE);
	if (_space[z_idx + 0][y_idx + 0][x_idx + 1].tsdf < -0.99) return (EDGE);
	if (_space[z_idx + 1][y_idx + 0][x_idx + 1].tsdf < -0.99) return (EDGE);
	if (_space[z_idx + 0][y_idx - 1][x_idx + 1].tsdf < -0.99) return (EDGE);
	if (_space[z_idx + 1][y_idx - 1][x_idx + 1].tsdf < -0.99) return (EDGE);

	//get weights
	w_x = ((*coordinates)[0] - c_vxl_mp.x) / _voxeldimension;
	w_y = ((*coordinates)[1] - c_vxl_mp.y) / _voxeldimension;
	w_z = ((*coordinates)[2] - c_vxl_mp.z) / _voxeldimension;

	//Interpolate
	*tsdf = _space[z_idx + 0][y_idx + 0][x_idx + 0].tsdf * (1 - w_x) * (1 - w_y) * (1 - w_z)
        		+ _space[z_idx + 1][y_idx + 0][x_idx + 0].tsdf * (1 - w_x) * (1 - w_y) * w_z
        		+ _space[z_idx + 0][y_idx - 1][x_idx + 0].tsdf * (1 - w_x) * w_y * (1 - w_z)
        		+ _space[z_idx + 1][y_idx - 1][x_idx + 0].tsdf * (1 - w_x) * w_y * w_z
        		+ _space[z_idx + 0][y_idx + 0][x_idx + 1].tsdf * w_x * (1 - w_y) * (1 - w_z)
        		+ _space[z_idx + 1][y_idx + 0][x_idx + 1].tsdf * w_x * (1 - w_y) * w_z
        		+ _space[z_idx + 0][y_idx - 1][x_idx + 1].tsdf * w_x * w_y * (1 - w_z)
        		+ _space[z_idx + 1][y_idx - 1][x_idx + 1].tsdf * w_x * w_y * w_z;

	return (OK);
}

}
