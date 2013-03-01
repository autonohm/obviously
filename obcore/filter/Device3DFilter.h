/*
 * Device3DFilter.h
 *
 *  Created on: 21.02.2013
 *      Author: phil
 */

#ifndef DEVICE3DFILTER_H_
#define DEVICE3DFILTER_H_

#include "Filter.h"
#include "obcore/datatypes/Device3DData.h"

namespace obvious
{

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
