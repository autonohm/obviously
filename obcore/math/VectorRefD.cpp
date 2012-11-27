#include "VectorRefD.h"

#include <gsl/gsl_vector.h>

#define GSL(x) (static_cast<gsl_vector*>(x))

namespace obvious {

VectorRefD::VectorRefD(std::vector<void*> vectors)
    : AbstractVectorRef(vectors)
{
    if (!_data.size())
        return;

    _size = GSL(_data.front())->size;
}

VectorRefD::~VectorRefD(void)
{
    this->deleteVectors();
}

VectorRefD::iterator VectorRefD::begin(const unsigned int channel)
{
    return iterator(gsl_vector_ptr(GSL(_data[channel]), 0), GSL(_data[channel])->stride);
}

VectorRefD::const_iterator VectorRefD::begin(const unsigned int channel) const
{
    return const_iterator(gsl_vector_ptr(GSL(_data[channel]), 0), GSL(_data[channel])->stride);
}

VectorRefD::iterator VectorRefD::end(const unsigned int channel)
{
    return iterator(gsl_vector_ptr(GSL(_data[channel]), _size - 1) + 1, GSL(_data[channel])->stride);
}

VectorRefD::const_iterator VectorRefD::end(const unsigned int channel) const
{
    return const_iterator(gsl_vector_ptr(GSL(_data[channel]) + 1, GSL(_data[channel])->stride));
}

double& VectorRefD::at(const unsigned int index, const unsigned int channel)
{
    return *gsl_vector_ptr(GSL(_data[channel]), index);
}

double VectorRefD::at(const unsigned int index, const unsigned int channel) const
{
    return gsl_vector_get(GSL(_data[channel]), index);
}

void VectorRefD::deleteVectors(void)
{
    for (unsigned int i = 0; i < _data.size(); i++)
        delete GSL(_data[i]);

    _data.clear();
}

} // end namepace obvious

std::ostream& operator<<(std::ostream& os, const obvious::VectorRefD& vec)
{
    os << "vector: size = " << vec.size() << std::endl;

    for (unsigned int i = 0; i < vec.size(); i++)
    {
        os << vec.at(i);
        if ((i + 1) < vec.size()) os << ", ";
    }

    os << std::endl;
}
