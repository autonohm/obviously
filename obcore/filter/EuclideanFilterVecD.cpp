/*
 * EuclideanFilterVecD.cpp
 *
 *  Created on: 30.11.2012
 *      Author: phil
 */

#include "obcore/filter/EuclideanFilterVecD.h"
#include "obcore/math/mathbase.h"
#include <cmath>
#include <iostream>

using namespace obvious;

double absVector(vector<double>::iterator start);

FILRETVAL EuclideanFilterVecD::applyFilter(void)
{
	vector<double>::iterator inIter=_input->begin();
	double abs=0;
	unsigned int outCtr=0;

	for(unsigned int i=0;i<((unsigned int)_input->size())/3;i++)
	{
		abs=absVector(inIter);
//		abs=euklideanDistanceVecD(&inIter,NULL);
		if(abs<_threshold)
		{
			for(unsigned int j=0;j<3;j++)
			{
			_output->push_back(*inIter++);
			outCtr++;
			}
		}
		else
			inIter+=3;
	}

	return(FILTER_OK);
}

double absVector(vector<double>::iterator start)
{
	double abs=0;
	vector<double>::iterator iter=start;
	for(unsigned int i=0;i<3;i++)
	{
		abs+=*iter*(*iter);
		iter++;
	}
	return(std::sqrt(abs));
}



