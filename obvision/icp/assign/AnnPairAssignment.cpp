#include "AnnPairAssignment.h"
#include "obcore/base/System.h"
#include "obcore/base/tools.h"

namespace obvious
{

void AnnPairAssignment::init()
{
	_tree = NULL;
}

AnnPairAssignment::~AnnPairAssignment()
{
	if(_tree)
	{
		delete _tree;
		_tree = NULL;
		annClose();
	}
}

void AnnPairAssignment::setMaxVisitPoints(unsigned int unVisitPoints)
{
	annMaxPtsVisit(unVisitPoints);
}

void AnnPairAssignment::setModel(double** ppdModel, int nSize)
{
	
	if(_tree)
	{
		delete _tree;
		_tree = NULL;
	}
	_ppdModel = ppdModel;
	_tree = new ANNkd_tree(			// build search structure
				 			ppdModel,	      // the data points
				 			nSize,		      // number of points
				 			_nDimension);   // dimension of space
}

void AnnPairAssignment::determinePairs(double** ppdScene, bool* mask, int nSize)
{

	ANNidxArray	anIdx;					// near neighbor indices
	ANNdistArray adDists;				// near neighbor distances
	anIdx = new ANNidx[1];
	adDists = new ANNdist[1];
	
	double dErr = 0.0;

  for(int i = 0; i < nSize; i++)
  {
    if(mask[i]==1)
    {

      _tree->annkSearch(
            ppdScene[i],      // query point
            1,                // number of near neighbors to find
            anIdx,            // nearest neighbor array (modified)
            adDists,          // dist to near neighbors (modified)
            dErr);            // error bound
      addPair(anIdx[0], i, adDists[0]);
    }
    else
    {
      addNonPair(i);
    }
  }
	delete[] anIdx;
	delete[] adDists;
}

}
