/*
 * Device3DData.cpp
 *
 *  Created on: 01.03.2013
 *      Author: phil
 */

#include "Device3DData.h"
#include <cstdlib>

namespace obvious
{

Device3DData::Device3DData(double* const pointCloud, bool* const mask,double* const depthMap) :
	          _pointCloud(pointCloud),
	          _mask(mask),
	          _depthMap(depthMap)
{
	_normals=NULL;
	_validPoints=0;
}

Device3DData::Device3DData(const Device3DData* const data) :
		                        _pointCloud(data->getPointCloud()),
		                        _normals(data->getNormals()),
		                        _mask(data->getMask()),
		                        _depthMap(data->getDepthMap())
{
	_validPoints=data->getValidPoints();
}


Device3DData::~Device3DData(void)
{
/*   if(_pointCloud)
      delete _pointCloud;
   if(_normals)
      delete _normals;
   if(_mask)
      delete _mask;
   if(_depthMap)
      delete _depthMap;*/
}

}

