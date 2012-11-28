#ifndef __VEC_D_
#define __VEC_D_

#include "AbstractVector.h"

namespace xmlpp {
class Node;
}

namespace obvious {

class VecD : public AbstractVector<double>
{
public:
    VecD(const unsigned int size = 0, const unsigned int channels = 1);
    VecD(const xmlpp::Node* node);
    ~VecD(void);

    void copyTo(VecD& vec) const;
    void createXml(xmlpp::Node* node) const;
    double& at(const unsigned int index, const unsigned int channel = 1);
    double at(const unsigned int index, const unsigned int channel = 1) const;

    iterator begin(const unsigned int channel);
    const_iterator begin(const unsigned int channel) const;

    iterator end(const unsigned int channel);
    const_iterator end(const unsigned int channel) const;

private:
    void freeData(void);
};

} // end namespace obvious

#endif
