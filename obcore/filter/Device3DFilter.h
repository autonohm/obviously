/*
 * Device3DFilter.h
 *
 *  Created on: 21.02.2013
 *      Author: phil
 */

#ifndef DEVICE3DFILTER_H_
#define DEVICE3DFILTER_H_

#include "Filter.h"
#include "obdevice/Kinect.h"

namespace obvious
{

class Device3DData
{
public:
	Device3DData(Kinect* kinect,double* normals);
	Device3DData(double* pointcloud,double* normals,bool* mask,double* depthMap);
	Device3DData(Device3DData* data);
	~Device3DData(void);
	double* getPointCloud(void){return(_pointCloud);}
	double* getNormals(void){return(_normals);}
	double* getDepthMap(void){return(_depthMap);}
	bool* getMask(void){return(_mask);}
	unsigned int getValidPoints(void){return(_validPoints);}
//	void setPointCloud(double* pointCloud){_pointCloud=pointCloud;}
//	void setNormals(double* normals);
//	void setDepthMap(double* depthMap){_depthMap=depthMap;}
//	void setMask(bool* mask){_mask=mask;}
	void setValidPoints(unsigned int validPoints){_validPoints=validPoints;}
private:
	double* _pointCloud;
	double* _normals;
	double* _depthMap;
	bool* _mask;
	unsigned int _validPoints;
};


class Device3DFilter : public IFilter
{
public:
	Device3DFilter(void);
	~Device3DFilter(void);
	void setInput(Device3DData* input){_input=input;}
	Device3DData* getOutput(void);
	FILRETVAL applyFilter(void);
	void setThreshold(double threshold){_threshold=threshold;}
private:
	Device3DData* _input;
	Device3DData* _output;
	double _threshold;
};

}
#endif /* DEVICE3DFILTER_H_ */
