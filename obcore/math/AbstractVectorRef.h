#ifndef __ABSTRACT_VECTOR_REF__
#define __ABSTRACT_VECTOR_REF__

#include <vector>
#include <iostream>

namespace obvious {

template <typename T>
class AbstractVectorRef
{
public:

    class iterator
    {
    public:
        iterator(T* data = 0, const unsigned int stride = 1) : _data(data), _stride(stride) { }

    private:
        T* _data;
        unsigned int _stride;
    };

    class const_iterator
    {
    public:
        const_iterator(const T* data = 0, const unsigned int stride = 1) : _data(data), _stride(stride) { }

    private:
        const T* _data;
        unsigned int _stride;
    };

    AbstractVectorRef(std::vector<void*> vectors = std::vector<void*>()) : _data(vectors), _size(0) { }
    virtual ~AbstractVectorRef(void) { }

    virtual iterator begin(const unsigned int channel = 0) = 0;
    virtual const_iterator begin(const unsigned int channel = 0) const = 0;

    virtual iterator end(const unsigned int channel = 0) = 0;
    virtual const_iterator end(const unsigned int channel = 0) const = 0;

    virtual T& at(const unsigned int index, const unsigned int channel = 0) = 0;
    virtual T  at(const unsigned int index, const unsigned int channel = 0) const = 0;

    unsigned int channels(void) const { return _data.size(); }
    unsigned int size(void) const { return _size; }

protected:
    std::vector<void*> _data;
    unsigned int _size;
};

} // end namespace obvious



#endif
