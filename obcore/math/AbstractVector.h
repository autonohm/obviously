#ifndef __ABSTRACT_VECTOR__
#define __ABRTRACT_VECTOR__

#include <vector>
#include <list>

namespace obvious {

template <typename T>
class AbstractVector
{
public:

    class iterator
    {
    public:
        iterator(T* data = 0, const unsigned int stride = 1) : _data(data), _stride(stride) { }
        iterator(const iterator& it) : _data(it._data), _stride(it._stride) { }

        iterator& operator=(const iterator& it)
        {
            _data = it._data;
            _stride = it._stride;
            return *this;
        }

        iterator& operator++(void)
        {
            _data += _stride;
            return *this;
        }

        iterator operator++(int)
        {
            iterator it(*this);
            _data += _stride;
            return it;
        }

        iterator& operator--(void)
        {
            _data -= _stride;
            return *this;
        }

        iterator operator--(int)
        {
            iterator it(*this);
            _data -= _stride;
            return it;
        }

        bool operator==(const iterator& it) const { return _data == it._data; }
        bool operator!=(const iterator& it) const { return _data != it._data; }
        bool operator< (const iterator& it) const { return _data <  it._data; }
        bool operator<=(const iterator& it) const { return _data <= it._data; }
        bool operator> (const iterator& it) const { return _data >  it._data; }
        bool operator>=(const iterator& it) const { return _data >= it._data; }

        double& operator*(void) const { return *_data; }

    private:
        T* _data;
        unsigned int _stride;
    };



    class const_iterator
    {
    public:
        const_iterator(const T* data = 0, const unsigned int stride = 1) : _data(data), _stride(stride) { }
        const_iterator(const const_iterator& it) : _data(it._data), _stride(it._stride) { }

        const_iterator& operator=(const const_iterator& it)
        {
            _data = it._data;
            _stride = it._stride;
            return *this;
        }

        const_iterator& operator++(void)
        {
            _data += _stride;
            return *this;
        }

        const_iterator operator++(int)
        {
            const_iterator it(*this);
            _data += _stride;
            return it;
        }

        const_iterator& operator--(void)
        {
            _data -= _stride;
            return *this;
        }

        const_iterator operator--(int)
        {
            const_iterator it(*this);
            _data -= _stride;
            return it;
        }

        bool operator==(const const_iterator& it) const { return _data == it._data; }
        bool operator!=(const const_iterator& it) const { return _data != it._data; }
        bool operator< (const const_iterator& it) const { return _data <  it._data; }
        bool operator<=(const const_iterator& it) const { return _data <= it._data; }
        bool operator> (const const_iterator& it) const { return _data >  it._data; }
        bool operator>=(const const_iterator& it) const { return _data >= it._data; }

        T operator*(void) const { return *_data; }

    private:
        const T* _data;
        unsigned int _stride;
    };



    //! default constructor
    AbstractVector(const unsigned size = 0) : _size(size) { }

    //! destructor
    ~AbstractVector(void) { }

public:
    virtual T& at(const unsigned int index, const unsigned int channel = 0) = 0;
    virtual T at(const unsigned int index, const unsigned int channel = 0) const = 0;

    virtual iterator begin(const unsigned int channel = 0) = 0;
    virtual const_iterator begin(const unsigned int channel = 0) const = 0;

    virtual iterator end(const unsigned int channel = 0) = 0;
    virtual const_iterator end(const unsigned int channel = 0) const = 0;

    unsigned int size(void) const { return _size; }

    //! assignment operator
    /*!
      he dosen't make a deep copy. Both Mats works on the same data. For a explicit deep copy use the copy constructor or the funciton copyTo()
    */
    virtual AbstractVector<T>& operator=(AbstractVector<T>& vec)
    {
        this->signOn(&vec);
        return *this;
    }

protected:
    bool haveToFreeData(void);

    std::vector<void*> _data;
    unsigned int _size;

private:
    void signOn (AbstractVector<T>* vec);
    void signOff(AbstractVector<T>* vec);

    std::list<AbstractVector<T>*> _refs;
};

template <typename T>
void AbstractVector<T>::signOn(AbstractVector<T>* vec)
{
    if (this == vec)
        return;

    _refs.push_back(vec);
    vec->_refs.push_back(this);
    _data = vec->_data;

    _size = vec->_size;
}

template <typename T>
void AbstractVector<T>::signOff(AbstractVector<T>* vec)
{
    if (this == vec)
        return;

    _refs.remove(vec);
    _data.clear();
    _size = 0;
}

template <typename T>
bool AbstractVector<T>::haveToFreeData(void)
{
    /* If list m_refs empty the destructor has to delete _data */
    if (!_refs.size())
        return true;

    while (!_refs.empty())
    {
        this->signOff(_refs.front());
    }

    return false;
}

} // end namespace obvious

#endif
