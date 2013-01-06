#ifndef __VEC_D_
#define __VEC_D_

#include "AbstractVector.h"

#include <ostream>

namespace xmlpp {
class Node;
}

namespace obvious {

class VecD : public AbstractVector<double>
{
public:
    VecD(const unsigned int size = 0, const unsigned int channels = 1);
    VecD(const VecD& vec);
    VecD(VecD& vec);
    VecD(const xmlpp::Node* node);
    ~VecD(void);

    void copyTo(VecD& vec) const;
    void createXml(xmlpp::Node* node) const;
    double& at(const unsigned int index, const unsigned int channel = 0);
    double at(const unsigned int index, const unsigned int channel = 0) const;

    iterator begin(const unsigned int channel);
    const_iterator begin(const unsigned int channel) const;

    iterator end(const unsigned int channel);
    const_iterator end(const unsigned int channel) const;

    VecD& operator=(VecD vec);

private:
    void freeData(void);
};

} // end namespace obvious

std::ostream& operator<<(std::ostream& os, const obvious::VecD& vec);

#endif
