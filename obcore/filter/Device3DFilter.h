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
	Device3DData(Kinect* const kinect,double* const normals);
	Device3DData(double* const pointCloud,double* const normals, bool* const mask,double* const depthMap);
	Device3DData(const Device3DData* const data);
	~Device3DData(void);
	double* getPointCloud(void)const{return(_pointCloud);}
	double* getNormals(void)const{return(_normals);}
	double* getDepthMap(void)const{return(_depthMap);}
	bool* getMask(void)const{return(_mask);}
	unsigned int getValidPoints(void)const{return(_validPoints);}
	void setValidPoints(unsigned int validPoints){_validPoints=validPoints;}
private:
	double* const _pointCloud;
	double* const _normals;
	bool* const _mask;
	double* const _depthMap;
	unsigned int _validPoints;
};


class Device3DFilter : public IFilter
{
public:
	Device3DFilter(void);
	~Device3DFilter(void);
	void setInput(Device3DData* const input){_input=input;}
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
