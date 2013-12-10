#include "VectorView.h"

using namespace Eigen;

namespace obvious
{
  VectorView::VectorView()
  {

  }

  VectorView::VectorView(double* data, unsigned int size)
  {
    //_V = Map<VectorXd>(data, size);
  }

  VectorView::VectorView(double* data, unsigned int size, unsigned int stride)
  {
    //_V = Map<VectorXd, size, InnerStride<stride> >(data);
  }

  VectorView::~VectorView()
  {

  }

	double& VectorView::operator [] (unsigned int i)
	{
	  //return _V(i);
	}

	double* VectorView::ptr()
	{
	  //return _V.data();
	}

	void VectorView::addConstant(const double val)
	{
	  //_V += val;
	}

  void VectorView::print()
  {
    //cout << _V;
  }

}
