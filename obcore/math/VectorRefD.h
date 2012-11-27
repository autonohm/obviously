#ifndef __VECTOR_REF_D__
#define __VECTOR_REF_D__

#include "AbstractVectorRef.h"

namespace obvious {

class VectorRefD : public AbstractVectorRef<double>
{
public:
    VectorRefD(std::vector<void*> vectors);
    virtual ~VectorRefD(void);

    virtual iterator begin(const unsigned int channel);
    virtual const_iterator begin(const unsigned int channel) const;

    virtual iterator end(const unsigned int channel);
    virtual const_iterator end(const unsigned int channel) const;

    virtual double& at(const unsigned int index, const unsigned int channel = 0);
    virtual double  at(const unsigned int index, const unsigned int channel = 0) const;

private:
    void deleteVectors(void);
};

} // end namespace obvious

std::ostream& operator<<(std::ostream& os, const obvious::VectorRefD& ref);

#endif
