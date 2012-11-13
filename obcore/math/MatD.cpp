#include "MatD.h"

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_permutation.h>

#include <iostream>
#include <libxml++/libxml++.h>

namespace {
#define GSL(x) (static_cast<gsl_matrix*>(x))
}

namespace obvious {

MatD::MatD(const unsigned int rows, const unsigned int cols, const unsigned int channels)
    : AbstractMat(rows, cols)
{
    if (!_rows || !_cols)
        return;

    for (unsigned int channel = 0; channel < channels; channel++)
        _data.push_back(gsl_matrix_alloc(_rows, _cols));
}

MatD::MatD(const MatD& mat)
    : AbstractMat(mat._rows, mat._cols)
{
    if (!_rows || !_cols)
        return;

    for (unsigned int channel = 0; channel < mat._data.size(); channel++)
    {
        _data.push_back(gsl_matrix_alloc(_rows, _cols));
        gsl_matrix_memcpy(GSL(_data[channel]), GSL(mat._data[channel]));
    }
}


// Fix me !!! Can just save and create to/from xml file with one channel.
MatD::MatD(const xmlpp::Node* node)
    : AbstractMat(0, 0)
{
    /* Check if the XML tag has the name mat */
    if (node->get_name() != "mat")
    {
        throw "Invaild xml node for matrix initialization!";
        return;
    }

    /* Check if node is from type xmlpp::Element. If not throw an exeption and return */
    const xmlpp::Element* root = dynamic_cast<const xmlpp::Element*>(node);

    if (!root)
    {
        throw "Invaild xml node for matrix initialization!";
        return;
    }

    /* Read attributes cols and rows to know the dimension of the MatD */
    std::stringstream stream(root->get_attribute_value("rows"));
    unsigned int rows, cols;

    stream >> rows;
    stream.clear();
    stream.str(root->get_attribute_value("cols"));
    stream >> cols;

    /* If any of both has a null as value return */
    if (!rows || !cols)
    {
        return;
    }

    /* Allocate matrix */
    _data.push_back(gsl_matrix_alloc(rows, cols));
    _cols = cols;
    _rows = rows;

    /* Read values for the matrix */
    const xmlpp::Node::NodeList nodes = root->get_children();

    unsigned int row = 0;

    for (xmlpp::Node::NodeList::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        xmlpp::Element* elm = dynamic_cast<xmlpp::Element*>(*it);

        if (!elm || elm->get_name() != "row")
            continue;

        stream.clear();
        stream.str(elm->get_child_text()->get_content());

        for (unsigned int col = 0; col < cols; col++)
        {
            double value;

            stream >> value;
            gsl_matrix_set(GSL(_data[0]), row, col, value);
        }

        if (++row >= rows)
            break;
    }
}

MatD::~MatD(void)
{
    this->freeData();
}

void MatD::freeData(void)
{
    /* Check if _data has to be deleted */
    if (this->haveToFreeData())
        for (unsigned int channel = 0; channel < _data.size(); channel++)
            gsl_matrix_free(GSL(_data[channel]));

    _data.clear();
}

// Fix me !!!
void MatD::createXml(xmlpp::Node* node) const
{
    /* Create tag mat with attributes cols and rows */
    xmlpp::Element* root = node->add_child("mat");
    std::stringstream stream;
    stream << _rows;
    root->set_attribute("rows", stream.str());

    stream.str(std::string());
    stream << _cols;
    root->set_attribute("cols", stream.str());

    /* Write the data from matrix m_data to xml nodes */
    for (unsigned int row = 0; row < _rows; row++)
    {
        xmlpp::Element* elm = root->add_child("row");
        stream.str(std::string());

        for (unsigned int col = 0; col < _cols; col++)
            stream << gsl_matrix_get(GSL(_data[0]), row, col) << " ";

        elm->add_child_text(stream.str());
    }
}

double& MatD::at(const unsigned int row, const unsigned int col, const unsigned int channel)
{
    return *gsl_matrix_ptr(GSL(_data[channel]), row, col);
}

double MatD::at(const unsigned int row, const unsigned int col, const unsigned int channel) const
{
    return gsl_matrix_get(GSL(_data[channel]), row, col);
}

MatD& MatD::operator=(MatD& mat)
{
    /* Delete m_data before take a ref to another Mat */
    this->freeData();

    AbstractMat<double>::operator=(mat);

    return *this;
}

MatD& MatD::operator=(MatD mat)
{
    this->freeData();

    AbstractMat<double>::operator=(mat);

    return *this;
}

MatD MatD::operator*(const MatD& mat)
{
    MatD matN(GSL(_data[0])->size1, GSL(mat._data[0])->size2);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, GSL(_data[0]), GSL(mat._data[0]), 0.0, GSL(matN._data[0]));
    return matN;
}

}


std::ostream& operator<<(std::ostream& os, const obvious::MatD& mat)
{
    os << "Matrix (" << mat.cols() << "," << mat.rows() << ")" << std::endl;
    os << "----------------------------" << std::endl;

    for (unsigned int row = 0; row < mat.rows(); row++)
    {
        os << row << ": ";

        for (unsigned int col = 0; col < mat.cols(); col++)
        {
            os << mat.at(row, col) << " ";
        }

        os << std::endl;
    }

    return os;
}
