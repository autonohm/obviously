#include "MatRGB.h"

#include <gsl/gsl_vector_uchar.h>
#include <gsl/gsl_matrix_uchar.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_permutation.h>
#include <iostream>

namespace {
const unsigned int CHANNELS = 3;

#define GSL(x) (static_cast<gsl_matrix_uchar*>(x))
}

namespace obvious {

MatRGB::MatRGB(const unsigned int rows, const unsigned int cols)
    : AbstractMat(rows, cols)
{
    if (!_rows || !_cols)
        return;

    for (unsigned int i = 0; i < CHANNELS; i++)
        _data.push_back(gsl_matrix_uchar_alloc(_rows, _cols));
}

MatRGB::MatRGB(const MatRGB& mat)
    : AbstractMat(mat._rows, mat._cols)
{
    if (!_rows || !_cols)
        return;

    for (unsigned int i = 0; i < CHANNELS; i++)
    {
        _data.push_back(gsl_matrix_uchar_alloc(_rows, _cols));
        gsl_matrix_uchar_memcpy(GSL(_data[i]), GSL(mat._data[i]));
    }
}

MatRGB::MatRGB(MatRGB& mat)
    : AbstractMat(mat._rows, mat._cols)
{
    AbstractMat<unsigned char>::operator=(mat);
}

MatRGB::~MatRGB(void)
{
    /* Check if m_data has to be deleted */
    this->freeData();
}

unsigned char& MatRGB::at(const unsigned int row, const unsigned int col, const unsigned int channel)
{
    return *gsl_matrix_uchar_ptr(GSL(_data[channel]), row, col);
}

unsigned char MatRGB::at(const unsigned int row, const unsigned int col, const unsigned int channel) const
{
    return gsl_matrix_uchar_get(GSL(_data[channel]), row, col);
}

RGBColor MatRGB::rgb(const unsigned int row, const unsigned int col) const
{
    return RGBColor(gsl_matrix_uchar_get(GSL(_data[Red])  , row, col),
                    gsl_matrix_uchar_get(GSL(_data[Green]), row, col),
                    gsl_matrix_uchar_get(GSL(_data[Blue]) , row, col));
}

void MatRGB::setRgb(const unsigned int row, const unsigned int col, const RGBColor& color)
{
    gsl_matrix_uchar_set(GSL(_data[Red])  , row, col, color.r());
    gsl_matrix_uchar_set(GSL(_data[Green]), row, col, color.g());
    gsl_matrix_uchar_set(GSL(_data[Blue]) , row, col, color.b());
}

//MatRGB& MatRGB::operator=(MatRGB& mat)
//{
//    /* Before take a reference to another Mat, delete m_data */
//    this->freeData();
//    AbstractMat<unsigned char>::operator=(mat);
//
//    return *this;
//}

MatRGB& MatRGB::operator=(MatRGB mat)
{
    /* Before take a reference to another Mat, delete m_data */
    this->freeData();
    AbstractMat<unsigned char>::operator=(mat);

    return *this;
}

//MatRGB& MatRGB::operator=(const MatRGB& mat)
//{
//    /* Before take a reference to another Mat, delete m_data */
//    this->freeData();
//    mat.copyTo(*this);
//
//    return *this;
//}

void MatRGB::copyTo(MatRGB& mat) const
{
    mat.freeData();
    mat._rows = _rows;
    mat._cols = _cols;

    if (!_rows || !_cols)
        return;

    for (unsigned int i = 0; i < _data.size(); i++)
    {
        mat._data.push_back(gsl_matrix_uchar_alloc(_rows, _cols));
        gsl_matrix_uchar_memcpy(GSL(mat._data[i]), GSL(_data[i]));
    }
}

MatRGB& MatRGB::swap(const Orientation orientation)
{
    for (unsigned int channel = 0; channel < _data.size(); channel++)
    {
        switch (orientation)
        {
        case X:
            for (unsigned int rowT = 0, rowB = _rows - 1; rowT < rowB; rowT++, rowB--)
                gsl_matrix_uchar_swap_rows(GSL(_data[channel]), rowT, rowB);

            break;

        case Y:
            for (unsigned int colL = 0, colR = _cols - 1; colL < colR; colL++, colR--)
                gsl_matrix_uchar_swap_columns(GSL(_data[channel]), colL, colR);

            break;
        }
    }

    return *this;
}

void MatRGB::freeData(void)
{
    if (this->haveToFreeData())
        for (unsigned int i = 0; i < _data.size(); i++)
            gsl_matrix_uchar_free(GSL(_data[i]));

    _data.clear();
}

}
