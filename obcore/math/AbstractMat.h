#ifndef __ABSTRACT_MAT__
#define __ABSTRACT_MAT__

#include <list>
#include <vector>
#include <ostream>

namespace obvious {

template <typename T>
class AbstractMat
{
public:
    //! default constructor
    AbstractMat(const unsigned int rows = 0, const unsigned int cols = 0) : _data(0), _rows(rows), _cols(cols) { }

    //! vritual destructor
    virtual ~AbstractMat(void) { }

    //! assignment operator
    /*!
      he dosen't make a deep copy. Both Mats works on the same data. For a explicit deep copy use the copy constructor or the funciton copyTo()
    */
    virtual AbstractMat<T>& operator=(AbstractMat<T>& mat)
    {
        this->signOn(&mat);
        return *this;
    }

    //! element access by col, row and channel
    virtual T& at(const unsigned int row, const unsigned int col, const unsigned int channel = 0) = 0;
    virtual T at(const unsigned int row, const unsigned int col, const unsigned int channel = 0) const = 0;

    //! get number of cols
    unsigned int cols(void) const { return _cols; }

    //! get number of rows
    unsigned int rows(void) const { return _rows; }

    //! get number of channels
    unsigned int channels(void) const { return _data.size(); }

protected:
    //! checks if the destructor has to delete _data
    /*!
      in classes that inherite this class the destructor must check if it has to delete _data.
      @return if ture the destructor of a subclass must delete m_data
    */
    bool haveToFreeData(void);


    std::vector<void*> _data;
    unsigned int       _rows;
    unsigned int       _cols;

private:
    void signOn (AbstractMat<T>* mat);
    void signOff(AbstractMat<T>* mat);

    std::list<AbstractMat<T>*> _refs;
};




template <typename T>
void AbstractMat<T>::signOn(AbstractMat<T>* mat)
{
    if (this == mat)
        return;

    _refs.push_back(mat);
    mat->_refs.push_back(this);
    _data = mat->_data;
    _cols = mat->_cols;
    _rows = mat->_rows;
}

template <typename T>
void AbstractMat<T>::signOff(AbstractMat<T>* mat)
{
    if (this == mat)
        return;

    _refs.remove(mat);
    mat->_refs.remove(this);
    _data.clear();
    _cols = 0;
    _rows = 0;
}

template <typename T>
bool AbstractMat<T>::haveToFreeData(void)
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

} //end namespace obvious

#endif
