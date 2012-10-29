#include "MatRGB.h"

#include <gsl/gsl_vector_uchar.h>
#include <gsl/gsl_matrix_uchar.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_permutation.h>

namespace {
const unsigned int CHANNELS = 3;

#define GSL(x) (static_cast<gsl_matrix_uchar*>(x))
}

namespace obvious {

MatRGB::MatRGB(const unsigned int cols, const unsigned int rows)
    : AbstractMat(cols, rows)
{
    for (unsigned int i = 0; i < CHANNELS; i++)
        m_data.push_back(gsl_matrix_uchar_alloc(rows, cols));
}

MatRGB::MatRGB(const MatRGB& mat)
    : AbstractMat(mat.m_cols, mat.m_rows)
{
    for (unsigned int i = 0; i < CHANNELS; i++)
    {
        m_data.push_back(gsl_matrix_uchar_alloc(m_rows, m_cols));
        gsl_matrix_uchar_memcpy(GSL(m_data[i]), GSL(mat.m_data[i]));
    }
}

MatRGB::~MatRGB(void)
{
    /* Check if m_data has to be deleted */
    if (this->haveToFreeData())
        for (unsigned int i = 0; i < m_data.size(); i++)
            gsl_matrix_uchar_free(GSL(m_data[i]));
}

unsigned char& MatRGB::at(const unsigned int col, const unsigned int row, const unsigned int channel)
{
    return *gsl_matrix_uchar_ptr(GSL(m_data[channel]), row, col);
}

unsigned char MatRGB::at(const unsigned int col, const unsigned int row, const unsigned int channel) const
{
    return gsl_matrix_uchar_get(GSL(m_data[channel]), row, col);
}

MatRGB& MatRGB::operator=(MatRGB& mat)
{
    /* Before take a reference to another Mat, delete m_data */
    if (this->haveToFreeData())
        for (unsigned int i = 0; i < m_data.size(); i++)
            gsl_matrix_uchar_free(GSL(m_data[i]));

    AbstractMat<unsigned char>::operator=(mat);

    return *this;
}

}
