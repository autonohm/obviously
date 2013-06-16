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
    /* iterator class */
    class iterator
    {
    public:
        //! default constructor
        iterator(T* data = 0) : _data(data) { }

        //! increment iterator (post)
        iterator& operator++(void)
        {
            _data++;
            return *this;
        }

        //! increment iterator (pre)
        iterator& operator++(int)
        {
            iterator it(_data);
            _data++;
            return *this;
        }

        //! decrement iterator (post)
        iterator& operator--(void)
        {
            _data--;
            return *this;
        }

        //! decrement iterator (pre)
        iterator& operator--(int)
        {
            iterator it(_data);
            _data--;
            return *this;
        }

        //! add number to iterator and return new iterator
        iterator& operator+(const int number) const
        {
            return iterator(_data + number);
        }
        iterator& operator+(const unsigned int number) const
        {
            return iterator(_data + number);
        }

        //! subtrate number from iterator and return new iterator
        iterator& operator-(const int number) const
        {
            return iterator(_data - number);
        }
        iterator& operator-(const unsigned int number) const
        {
            return iterator(_data - number);
        }

        //! add number to iterator
        iterator& operator+=(const int number)
        {
            _data += number;
            return *this;
        }
        iterator& operator+=(const unsigned int number)
        {
            _data += number;
            return *this;
        }

        //! subtrate number from iterator
        iterator& operator-=(const int number)
        {
            _data -= number;
            return *this;
        }
        iterator& operator-=(const unsigned int number)
        {
            _data -= number;
            return *this;
        }

        //! return a ref of data
        T& operator*(void)
        {
            return *_data;
        }

        bool operator<(iterator it)
        {
            return _data < it._data;
        }

    private:
        T* _data;
    };



    /* const_iterator class */
    class const_iterator
    {
    public:
        const_iterator(const T* data) : _data(data) { }

        //! increment iterator (post)
        const_iterator& operator++(void)
        {
            _data++;
            return *this;
        }

        //! increment iterator (pre)
        const_iterator& operator++(int)
        {
            iterator it(_data);
            _data++;
            return *this;
        }

        //! decrement iterator (post)
        const_iterator& operator--(void)
        {
            _data--;
            return *this;
        }

        //! decrement iterator (pre)
        const_iterator& operator--(int)
        {
            iterator it(_data);
            _data--;
            return *this;
        }

        //! add number to iterator and return new iterator
        const_iterator& operator+(const int number) const
        {
            return iterator(_data + number);
        }
        const_iterator& operator+(const unsigned int number) const
        {
            return iterator(_data + number);
        }

        //! subtrate number from iterator and return new iterator
        const_iterator& operator-(const int number) const
        {
            return iterator(_data - number);
        }
        const_iterator& operator-(const unsigned int number) const
        {
            return iterator(_data - number);
        }

        //! add number to iterator
        const_iterator& operator+=(const int number)
        {
            _data += number;
            return *this;
        }
        const_iterator& operator+=(const unsigned int number)
        {
            _data += number;
            return *this;
        }

        //! subtrate number from iterator
        const_iterator& operator-=(const int number)
        {
            _data -= number;
            return *this;
        }
        const_iterator& operator-=(const unsigned int number)
        {
            _data -= number;
            return *this;
        }

        bool operator<(const_iterator it)
        {
            return _data < it._data;
        }

        //! return a ref of data
        const T& operator*(void) const
        {
            return *_data;
        }

    private:
        const T* _data;
    };



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

    //! returns a iterator pointing to the begin of Mat
    virtual iterator begin(const unsigned int channel = 0) = 0;
    virtual const_iterator begin(const unsigned int channel = 0) const = 0;

    //! returns a iterator pointing to the end + 1 of Mat
    virtual iterator end(const unsigned int channel = 0) = 0;
    virtual const_iterator end(const unsigned int channel = 0) const = 0;

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
