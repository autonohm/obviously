#ifndef TSDGRIDLOCALIZATION_H_
#define TSDGRIDLOCALIZATION_H_

#include <iostream>
using namespace std;

#include "obcore/base/System.h"

#include "obcore/math/linalg/linalg.h"
#include "obvision/reconstruct/grid/TsdGrid.h"
#include "obvision/reconstruct/grid/RayCastPolar2D.h"

using namespace obvious;

namespace obvious
{


/**
 * @class TsdGridLocalization
 * @brief Represents an iterative scheme for localizing a sensor in TsdGrid
 * @author Stefan May
 **/
class TsdGridLocalization
{
public:
	/**
	 * Standard constructor
	 * @param
	 */
  TsdGridLocalization(TsdGrid* grid);
		 
	/**
	 * Destructor
	 */
	~TsdGridLocalization();

	Matrix localize(SensorPolar2D* sensor);

private:

	RayCastPolar2D _raycaster;

	TsdGrid* _grid;

};

}

#endif /*TSDGRIDLOCALIZATION_H_*/
