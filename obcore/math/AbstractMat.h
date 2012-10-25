#ifndef __ABSTRACT_MAT__
#define __ABSTRACT_MAT__

#include <list>
#include <ostream>

#ifndef __GSL_INCLUDED__
struct gsl_matrix;
#endif

namespace obvious {

template <typename T>
class AbstractMat
{
public:
    //! default constructor
    AbstractMat(const unsigned int cols = 0, const unsigned int rows = 0) : m_data(0), m_cols(cols), m_rows(rows) { }

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

    //! element access by col, row
    virtual T& at(const unsigned int col, unsigned int row) = 0;
    virtual T at(const unsigned int col, unsigned int row) const = 0;

    //! get number of cols
    unsigned int cols(void) const { return m_cols; }

    //! get number of rows
    unsigned int rows(void) const { return m_rows; }

protected:
    //! checks if the destructor has to delete m_data
    /*!
      in classes that inherite this class the destructor must check if it has to delete m_data.
      @return if ture the destructor of a subclass must delete m_data
    */
    bool haveToFreeData(void);


    gsl_matrix*  m_data;
    unsigned int m_cols;
    unsigned int m_rows;

private:
    void signOn (AbstractMat<T>* mat);
    void signOff(AbstractMat<T>* mat);

    std::list<AbstractMat<T>*> m_refs;
};




template <typename T>
void AbstractMat<T>::signOn(AbstractMat<T>* mat)
{
    if (this == mat)
        return;

    m_refs.push_back(mat);
    mat->m_refs.push_back(this);
    m_data = mat->m_data;
    m_cols = mat->m_cols;
    m_rows = mat->m_rows;
}

template <typename T>
void AbstractMat<T>::signOff(AbstractMat<T>* mat)
{
    if (this == mat)
        return;

    m_refs.remove(mat);
    mat->m_refs.remove(this);
    m_data = 0;
    m_cols = 0;
    m_rows = 0;
}

template <typename T>
bool AbstractMat<T>::haveToFreeData(void)
{
    /* If list m_refs empty the destructor has to delete m_data */
    if (!m_refs.size())
        return true;

    while (!m_refs.empty())
    {
        this->signOff(m_refs.front());
    }

    return false;
}

} //end namespace obvious

#endif
