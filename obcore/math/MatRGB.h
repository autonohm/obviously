#ifndef __MAT_RGB__
#define __MAT_RGB__

#include "obcore/math/AbstractMat.h"

namespace xmlpp {
class Node;
}

namespace obvious {

class MatRGB : public AbstractMat<unsigned char>
{
public:
    //! Enumeration Channel
    enum Channel {
        Red   = 0,
        Green = 1,
        Blue  = 2
    };

    //! default constructor
    MatRGB(const unsigned int rows = 0, const unsigned int cols = 0);
    //! copy constructor
    /*!
      makes a deep copy.
    */
    MatRGB(const MatRGB& mat);
    //! copy constructor
    /*!
      takes a ref to the data of mat.
    */
    MatRGB(MatRGB& mat);
    //! destructor
    ~MatRGB(void);

    //! get a reference of element by row, col and channel
    unsigned char& at(const unsigned int row, const unsigned int col, const unsigned int channel = Red);
    //! get a reference of element by row, col and channel
    unsigned char at(const unsigned int row, const unsigned int col, const unsigned int channel = Red) const;

    //! reimplementation of begin()
    virtual iterator begin(const unsigned int channel);
    virtual const_iterator begin(const unsigned int channel) const;

    //! reimplementation of end()
    virtual iterator end(const unsigned int channel);
    virtual const_iterator end(const unsigned int channel) const;

    enum Orientation {
        X,
        Y
    };
    //! swap mat by orientation
    MatRGB& swap(const Orientation orientation);

    //! makes a deep
    void copyTo(MatRGB& mat) const;

    //! assignment operator
    /*!
      he dosen't make a deep copy. Both Mats works on the same data. For a explicit deep copy use the copy constructor or the funciton copyTo()
    */
//    MatRGB& operator=(MatRGB& mat);
    MatRGB& operator=(MatRGB mat);

    //! assignment opeator
    /*!
      makes a deep copy.
    */
//    MatRGB& operator=(const MatRGB& mat);

private:
    void freeData(void);
};

}

#endif
