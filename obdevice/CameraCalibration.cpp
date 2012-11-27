#include "CameraCalibration.h"

#include <libxml++/libxml++.h>

CameraCalibration::CamerCalibration(const xmlpp::Node* node)
{
    /* Check if the XML tag has the name camera_calibration */
    if (node->get_name() != "camera_calibration")
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

        if (elm->get_name() == "intrinsic")
            _intrinsic = MatD(elm);
        else if (elm->get_name() == "distortion")
            _distCoeffs = MatD(elm);
    }
}

void CameraCalibration::createXml(xmlpp::Node* node)
{

}

void CameraCalibration::correctImage(MatRGB& image)
{

}
