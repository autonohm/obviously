/*
 * tsd_test2.cpp
 *
 *  Created on: 24.07.2012
 *      Author: phil
 */
#include <iostream>
#include "obgraphic/Obvious3D.h"
#include "obcore/base/tools.h"
#include "obcore/base/System.h"
#include "obvision/reconstruct/TsdSpace.h"
#include "obvision/reconstruct/TsdSpace_utils.h"
#include "obcore/base/Logger.h"

using namespace std;
using namespace obvious;

int main(void)
{
	LOGMSG_CONF("tsd_test.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_WARN);
	//Variables and Pointers
	const char *path={"pics/z_img/zpic.pgm"};
	double *depth_bfr;//[640*480];
	double *pcl=NULL;
	int rows,cols,size;

	// translation of sensor
	double tx = 0.5;
	double ty = 0.5;
	double tz = 0.0;

	// rotation about y-axis of sensor
	double theta = -10 * M_PI / 180;

	double tf[16]={cos(theta),  0, sin(theta), tx,
			0,           1, 0,          ty,
			-sin(theta), 0, cos(theta), tz,
			0,           0, 0,          1};

	VtkCloud *pre_cube,*post_cube;
	Obvious3D* viewer;
	unsigned int cl_ctr=0;
	double *inters=new double[3];
	double depth=0.0;

	double distZ[640*480];

	post_cube=new VtkCloud();

	//get data from kinect
	rows = 480;
	cols = 640;
	size = rows * cols;

	double*** buf;
	System<double>::allocate (cols, rows, 3, buf);

	// Setup synthetic perspective projection
	double su = 500;
	double sv = 500;
	double tu = 320;
	double tv = 240;
	double perspective[12]  = {su, 0, tu, 0, 0, sv, tv, 0, 0, 0, 1, 0};

	TsdSpace *my_space=new TsdSpace(1, 1, 1, 0.01, perspective);

	my_space->set_transformation(tf);

	my_space->set_max_truncation(0.1);

	// Background with distance = 0.8m -> s=0.8
	for(int u=0; u<cols; u++)
		for(int v=0; v<rows; v++)
		{
			double s = 0.8;
			double x = s*(((double)u) - tu) / su;
			double y = s*(((double)v) - tv) / sv;
			double z = s;
			buf[u][v][0] = x;
			buf[u][v][1] = y;
			buf[u][v][2] = z;
			distZ[v*640+u] = z;
		}

	// Centered square with distance = 0.5m -> s=0.5
	for(int u=cols/4; u<3*cols/4; u++)
		for(int v=rows/4; v<3*rows/4; v++)
		{
			double s = 0.5;
			double x = s*(((double)u) - tu) / su;
			double y = s*(((double)v) - tv) / sv;
			double z = s;
			buf[u][v][0] = x;
			buf[u][v][1] = y;
			buf[u][v][2] = z;
			distZ[v*640+u] = z;
		}

	my_space->_debug_on = false;
	my_space->push(distZ);

	//my_space->_debug_on = true;
	/**
	 * ToDo: check why generated clouds are completely wrong when applying the following test.
	 */
	//for(int u=160; u<180; u++)
		for(int u=0; u<640; u++)
			for(int v=50; v<150; v++)
		//for(int v=201; v<202; v++)
		{
			distZ[v*640+u] -= v * 0.0004;
		}
	my_space->push(distZ);

	my_space->gen_pcl(&pcl, &cl_ctr);
	cout<<"\nPCL generated! Generated cloud with "<<cl_ctr<<" Points!\n";

	unsigned char* buffer = new unsigned char[my_space->getXDimension()*my_space->getYDimension()*3];
	for(unsigned int i=0; i<my_space->getZDimension(); i++)
	{
		char path[64];
		sprintf(path, "/tmp/slice%04d.ppm", i);
		my_space->buildSliceImage(i, buffer);
		serializePPM(path, buffer, my_space->getXDimension(), my_space->getYDimension(), 0);
	}
	delete[] buffer;

	my_space->get_model(&pcl,&cl_ctr);
	cout<<"\nGetmodel returned with "<<cl_ctr<<" coordinates\n";

	post_cube->setCoords(pcl, cl_ctr/3, 3);

	//	viewer = new Obvious3D("TSD Cloud!");
	viewer = new Obvious3D("Model!");
	viewer->addCloud(post_cube);
	viewer->startRendering();

	delete post_cube;
	delete viewer;
	delete inters;

}


