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

double     _distZ[640*480];
TsdSpace*  _my_space;
Obvious3D* _viewer;
VtkCloud*  _cloud;
double *_normals;

void cbPushScene()
{
	double *pcl=NULL;
	unsigned int cl_ctr=0;
   _my_space->push(_distZ);
   _my_space->getModel(&pcl,_normals,&cl_ctr);
   _cloud->setCoords(pcl, cl_ctr/3, 3,_normals);
   _viewer->update();
}

int main(void)
{
	LOGMSG_CONF("tsd_test.log", Logger::file_off|Logger::screen_on, DBG_DEBUG, DBG_WARN);

	double     *pcl  = NULL;
	int         rows, cols, size;
	_normals=new double[640*480*3];

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

	unsigned int cl_ctr=0;

	_cloud = new VtkCloud();

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

	_my_space=new TsdSpace(1, 1, 1, 0.02, perspective);
	_my_space->setTransformation(tf);
	_my_space->setMaxTruncation(0.08);

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
			_distZ[v*640+u] = z;//sqrt(x*x+y*y+z*z);
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
			_distZ[v*640+u] = z;     //sqrt(x*x+y*y+z*z);
		}

	_my_space->push(_distZ);

	/**
	 * ToDo: check why generated clouds are completely wrong when applying the following test.
	 */
	for(int u=0; u<cols; u++)
	   for(int v=0; v<rows; v++)
	   {
	      _distZ[v*cols+u] -= v * 0.0004;
	   }

	/*_my_space->gen_pcl(&pcl, &cl_ctr);
	cout<<"\nPCL generated! Generated cloud with "<<cl_ctr<<" Points!\n";*/

	unsigned char* buffer = new unsigned char[_my_space->getXDimension()*_my_space->getYDimension()*3];
	for(unsigned int i=0; i<_my_space->getZDimension(); i++)
	{
		char path[64];
		sprintf(path, "/tmp/slice%04d.ppm", i);
		_my_space->buildSliceImage(i, buffer);
		serializePPM(path, buffer, _my_space->getXDimension(), _my_space->getYDimension(), 0);
	}
	delete[] buffer;

	_my_space->getModel(&pcl,_normals,&cl_ctr);

	cout << "\nGetmodel returned with " << cl_ctr << " coordinates\n";

	_cloud->setCoords(pcl, cl_ctr/3, 3,_normals);

	_viewer = new Obvious3D("TSD Cloud");
	_viewer->addCloud(_cloud);
	_viewer->registerKeyboardCallback("space", cbPushScene);
	_viewer->startRendering();

	delete _cloud;
	delete _viewer;
	delete _normals;
}


