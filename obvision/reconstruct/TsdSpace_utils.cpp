/*
 * Tsd_space_utils.cpp
 *
 *  Created on: 16.08.2012
 *      Author: phil
 */

#include <cmath>
#include "TsdSpace_utils.h"
#include "obcore/base/tools.h"
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <cstring>

namespace obvious
{

/*****************************************************************************************************************************/

double abs3D(obvious::Matrix *foot_point,obvious::Matrix *ref_point)
	{
	//Variables and pointers
	double abs=0;

	//detect abuse
	if(foot_point->getRows()!=4)
		{
		std::cout<<"\nError! Dimensions must be 4,1\n";
		return(0);
		}

	if(foot_point->getCols()!=1)
		{
		std::cout<<"\nError! Dimensions must be 4,1\n";
		return(0);
		}


	if(ref_point!=NULL)
		{
		if(ref_point->getRows()!=4)
			{
			std::cout<<"\nError! Dimensions must be 4,1\n";
			return(0);
			}

		if(ref_point->getCols()!=1)
			{
			std::cout<<"\nError! Dimensions must be 4,1\n";
			return(0);
			}
		}


	for(unsigned int i=0;i<3;i++)
		{
		if(ref_point==NULL)
			abs+=(*foot_point)[i][0]*(*foot_point)[i][0];
		else
			abs+=((*ref_point)[i][0]-(*foot_point)[i][0])*((*ref_point)[i][0]-(*foot_point)[i][0]);
		}
	return(std::sqrt(abs));
	}

/*****************************************************************************************************************************/

void z_img_to_gysc(const double max_depth,const char *path,const double *z_bfr,const unsigned int width,const unsigned int height,bool trace_on)
	{
	//Variables and Pointers
	unsigned char img_bfr[width*height];
	unsigned int idx=0;
	double depth_qt=0.0;

	//Detect idiots
	if((z_bfr==NULL)||(path==NULL))
		{
		std::cout<<"\nError! Ptr to z-buffer or path is missing!\n";
		std::exit(1);
		}

	//initialize
		//Calculate depth quantum
	depth_qt=max_depth/255.0;       //255=size of uchar

	/*
	* Main loop
	*/

	//Iterate over given z-buffer. Generate gayscale uchars from 0..255 = 0..max_depth
	for(unsigned int i=0;i<height*width;i++)
		{
		if(z_bfr[i]>max_depth)
			img_bfr[i]=(unsigned char)(255);
		else
			{
			idx=z_bfr[i]/depth_qt;
			if((img_bfr[i]=(unsigned char)(idx))<0)
				{
				std::cout<<"\nERROR GENERATING COLOR!!!\n";
				std::exit(2);
				}
			}
		//Trace output for debugging
		if(trace_on)
			{
			std::cout<<"\nqt = "<<depth_qt<<"  depth = " <<z_bfr[i]<<"  idx = "<<idx<<"  color = "<<255-idx<<"\n";
			}
		}

	//Burn it down to disk
	obvious::serializePGM(path,img_bfr,width,height,0);
	}

/*****************************************************************************************************************************/

}
