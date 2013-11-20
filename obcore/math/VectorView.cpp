#include "VectorView.h"

namespace obvious
{
  VectorView::VectorView()
  {

  }

  VectorView::VectorView(double* data, unsigned int size)
  {
    _V = gsl_vector_view_array(data, size);
  }

  VectorView::VectorView(double* data, unsigned int size, unsigned int stride)
  {
    _V = gsl_vector_view_array_with_stride(data, stride, size);
  }

  VectorView::~VectorView()
  {

  }

	double VectorView::operator [] (unsigned int i)
	{
	  return gsl_vector_get(&(_V.vector), i);
	}

	double* VectorView::ptr()
	{
	  return _V.vector.data;
	}

	void VectorView::addConstant(const double val)
	{
	  gsl_vector_add_constant(&_V.vector, val);
	}

  void VectorView::print()
  {
    for(size_t i=0; i<_V.vector.size; i++)
    {
      cout << (*this)[i] << " ";
    }
    cout << endl;
  }

}
