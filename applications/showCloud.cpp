/*
 * showCloud.cpp
 *
 *  Created on: 31.10.2012
 *      Author: phil
 */

#include <iostream>
#include "obgraphic/Obvious3D.h"

using namespace obvious;
using namespace std;

/**
 * This application shows a given cloud in the Obvious3D Viewer.
 * Author: Philipp Koch
 * Date: 31.10.2012
 */

int main(int argc, char **argv)
{
	if(argc!=2)
		{
		cout<<"Usage: " << argv[0]  << " <****.vtp> \n";
		exit(1);
		}

	VtkCloud *vcloud;
	Obvious3D *viewer;

	vcloud = VtkCloud::load(argv[1], VTKCloud_AUTO);
	viewer=new Obvious3D("ShowCloud");
	viewer->addCloud(vcloud);
	viewer->startRendering();

	delete vcloud;
	delete viewer;

	return(0);

}

