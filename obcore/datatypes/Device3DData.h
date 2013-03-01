/*
 * Device3DData.h
 *
 *  Created on: 01.03.2013
 *      Author: phil
 */

#ifndef DEVICE3DDATA_H_
#define DEVICE3DDATA_H_


namespace obvious
{

class Device3DData
{
public:
	Device3DData(double* const pointCloud, bool* const mask,double* const depthMap);
	Device3DData(const Device3DData* const data);
	~Device3DData(void);
	double* getPointCloud(void)const{return(_pointCloud);}
	double* getNormals(void)const{return(_normals);}
	double* getDepthMap(void)const{return(_depthMap);}
	bool* getMask(void)const{return(_mask);}
	unsigned int getValidPoints(void)const{return(_validPoints);}
	void setValidPoints(unsigned int validPoints){_validPoints=validPoints;}
	void setNormals(double* const normals){_normals=normals;}
private:
	double* const _pointCloud;
	double* _normals;
	bool* const _mask;
	double* const _depthMap;
	unsigned int _validPoints;
};

}


#endif /* DEVICE3DDATA_H_ */
