#include "CameraCalibration.h"

#include <iostream>
#include <libxml++/libxml++.h>

namespace {
const char* const TAG_CALIBRATION = "camera_calibration";
const char* const TAG_INTRINSIC   = "intrinsic";
const char* const TAG_DISTORTION  = "distortion";
}

namespace obvious {

CameraCalibration::CameraCalibration(const xmlpp::Node* node)
{
    /* Check if the XML tag has the name camera_calibration */
    if (node->get_name() != TAG_CALIBRATION)
    {
        throw "Invaild xml node for camera calibration initialization!";
        return;
    }

    /* Check if node is from type xmlpp::Element. If not throw an exeption and return */
    const xmlpp::Element* root = dynamic_cast<const xmlpp::Element*>(node);

    if (!root)
    {
        throw "Invaild xml node for camera calibration initialization!";
        return;
    }

    /* initialize intrinsic matrix and distortion coefficients */
    const xmlpp::Node::NodeList nodes = root->get_children();

    for (xmlpp::Node::NodeList::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        xmlpp::Element* elm = dynamic_cast<xmlpp::Element*>(*it);

        if (!elm)
            continue;

        if (elm->get_name() == TAG_INTRINSIC)
        {
            const xmlpp::Node* child = this->getChild(elm, "mat");
            std::cout << "do init MatD" << std::endl;
            if (child)
                _intrinsic = MatD(child);
        }
        else if (elm->get_name() == TAG_DISTORTION)
        {
            const xmlpp::Node* child = this->getChild(elm, "vec");
            std::cout << "do init VecD" << std::endl;
            if (child)
                _distortion = VecD(child);
        }
    }
}

bool CameraCalibration::valid(void) const
{
    return _intrinsic.rows() == 3 && _intrinsic.cols() == 3 && _distortion.size() == 5;
}

void CameraCalibration::createXml(xmlpp::Node* node) const
{
    /* create tags */
    xmlpp::Element* root = node->add_child(TAG_CALIBRATION);
    xmlpp::Element* tagIntrinsic = root->add_child(TAG_INTRINSIC);
    xmlpp::Element* tagDistortion = root->add_child(TAG_DISTORTION);

    _intrinsic.createXml(tagIntrinsic);
    _distortion.createXml(tagDistortion);
}

const xmlpp::Node* CameraCalibration::getChild(const xmlpp::Node* parent, const std::string& child)
{
    /* try to cast Node to Element */
    const xmlpp::Element* root = dynamic_cast<const xmlpp::Element*>(parent);

    if (!root)
        return 0;

    const xmlpp::Node::NodeList nodes = root->get_children();

    for (xmlpp::Node::NodeList::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        const xmlpp::Element* elm = dynamic_cast<const xmlpp::Element*>(*it);

        if (elm && elm->get_name() == child)
            return elm;
    }

    return 0;
}

void CameraCalibration::correctImage(MatRGB& image)
{

}

} // end namespace obvious
