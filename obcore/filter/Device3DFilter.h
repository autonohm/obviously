/*
 * Device3DFilter.h
 *
 *  Created on: 21.02.2013
 *      Author: phil
 */

#ifndef DEVICE3DFILTER_H_
#define DEVICE3DFILTER_H_

#include "Filter.h"

namespace obvious
{

class Device3DData
{
public:
	Device3DData(void);
	~Device3DData(void);
	double* getPointCloud(void){return(_pointCloud);}
	double* getNormals(void){return(_normals);}
	double* getDepthMap(void){return(_depthMap);}
	bool* getMap(void){return(_mask);}
	void setPointCloud(double* pointCloud){_pointCloud=pointCloud;}
	void setNormals(double* pointCloud){_pointCloud=pointCloud;}
	void setDepthMap(double* depthMap){_depthMap=depthMap;}
private:
	double* _pointCloud;
	double* _normals;
	double* _depthMap;
	bool* _mask;
};


class Device3DFilter : public IFilter
{
public:
	Device3DFilter(void);
	~Device3DFilter(void);
	FILRETVAL setInput()
private:

};

}
#endif /* DEVICE3DFILTER_H_ */
