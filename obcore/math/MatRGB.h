#ifndef __MAT_RGB__
#define __MAT_RGB__

#include "AbstractMat.h"

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
    MatRGB(const unsigned int cols = 0, const unsigned int rows = 0);
    //! copy constructor
    MatRGB(const MatRGB& mat);
    //! destructor
    ~MatRGB(void);

    //! get a reference of element by col, row and channel
    unsigned char& at(const unsigned int col, const unsigned int row, const unsigned int channel = Red);
    //! get a reference of element by col, row and channel
    unsigned char at(const unsigned int col, const unsigned int row, const unsigned int channel = Red) const;

    //! assignment operator
    /*!
      he dosen't make a deep copy. Both Mats works on the same data. For a explicit deep copy use the copy constructor or the funciton copyTo()
    */
    MatRGB& operator=(MatRGB& mat);
};

}

#endif
