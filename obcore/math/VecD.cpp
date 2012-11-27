#include "VecD.h"

#include <gsl/gsl_vector.h>
#include <libxml++/libxml++.h>
#include <string>

#define GSL(x) (static_cast<gsl_vector*>(x))

namespace obvious {

VecD::VecD(const unsigned int size, const unsigned int channels)
    : AbstractVector(size)
{
    if (!_size)
        return;

    for (unsigned int channel = 0; channel < channels; channel++)
        _data.push_back(gsl_vector_alloc(_size));
}

VecD::VecD(const xmlpp::Node* node)
    : AbstractVector(0)
{
    /* Check if the XML tag has the name vec */
    if (node->get_name() != "vec")
    {
        throw "Invaild xml node for vector initialization!";
        return;
    }

    /* Check if node is from type xmlpp::Element. If not throw an exeption and return */
    const xmlpp::Element* root = dynamic_cast<const xmlpp::Element*>(node);

    if (!root)
    {
        throw "Invaild xml node for vector initialization!";
        return;
    }

    /* get size of vector */
    std::stringstream stream(root->get_attribute_value("size"));
    stream >> _size;

    /* get number of channels */
    stream.clear();
    stream.str(root->get_attribute_value("channels"));
    unsigned int channels;
    stream >> channels;

    /* allocate data for vector */
    for (unsigned int channel = 0; channel < channels; channels++)
        _data.push_back(gsl_vector_alloc(_size));

    /* copy data from xml to vector */
    const xmlpp::Node::NodeList nodes = root->get_children();
    unsigned int channel = 0;

    for (xmlpp::Node::NodeList::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        xmlpp::Element* elm = dynamic_cast<xmlpp::Element*>(*it);

        if (!elm || elm->get_name() != "channel")
            continue;

        stream.clear();
        stream.str(elm->get_child_text()->get_content());

        for (unsigned int i = 0; i < _size; i++)
        {
            double value;
            stream >> value;
            gsl_vector_set(GSL(_data[channel]), i, value);
        }

        if (++channel >= _data.size())
            break;
    }
}

VecD::~VecD(void)
{
    this->freeData();
}

void VecD::createXml(xmlpp::Node* node) const
{
    /* Create tag mat with attribute size and channels */
    xmlpp::Element* root = node->add_child("vec");
    std::stringstream stream;
    stream << _size;
    root->set_attribute("size", stream.str());

    stream.str(std::string());
    stream << _data.size();
    root->set_attribute("channels", stream.str());

    /* Write the data from vector _data to xml node */
    for (unsigned int channel = 0; channel < _data.size(); channel++)
    {
        xmlpp::Element* elm = root->add_child("channel");
        stream.str(std::string());

        for (unsigned int i = 0; i < _size; i++)
            stream << gsl_vector_get(GSL(_data[channel]), i) << " ";

        elm->add_child_text(stream.str());
    }
}

double& VecD::at(const unsigned int index, const unsigned int channel)
{
    return *gsl_vector_ptr(GSL(_data[channel]), index);
}

double VecD::at(const unsigned int index, const unsigned int channel) const
{
    return gsl_vector_get(GSL(_data[channel]), index);
}

VecD::iterator VecD::begin(const unsigned int channel)
{
    if (!_data.size())
        return iterator();

    return iterator(gsl_vector_ptr(GSL(_data[channel]), 0), GSL(_data[channel])->stride);
}

VecD::const_iterator VecD::begin(const unsigned int channel) const
{
    if (!_data.size())
        return const_iterator();

    return const_iterator(gsl_vector_ptr(GSL(_data[channel]), 0), GSL(_data[channel])->stride);
}

VecD::iterator VecD::end(const unsigned int channel)
{
    if (!_data.size())
        return iterator();

    return iterator(gsl_vector_ptr(GSL(_data[channel]), _size), GSL(_data[channel])->stride);
}

VecD::const_iterator VecD::end(const unsigned int channel) const
{
    if (!_data.size())
        return const_iterator();

    return const_iterator(gsl_vector_ptr(GSL(_data[channel]), _size), GSL(_data[channel])->stride);
}

void VecD::freeData(void)
{
    if (this->haveToFreeData())
        for (unsigned int channel = 0; channel < _data.size(); channel++)
            gsl_vector_free(GSL(_data[channel]));

    _data.clear();
}

} // end namespace obvious
