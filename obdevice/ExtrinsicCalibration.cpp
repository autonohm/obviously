#include "ExtrinsicCalibration.h"

#include <libxml++/libxml++.h>
#include <cmath>
#include <iostream>

namespace {
const char* TAG_EXTRINSIC   = "extrinsic_calibration";
const char* TAG_ROTATION    = "rotation";
const char* TAG_TRANSLATION = "translation";
}

namespace obvious {

ExtrinsicCalibration::ExtrinsicCalibration(const xmlpp::Node* node)
{
    if (node->get_name() != TAG_EXTRINSIC)
    {
        throw "Invaild xml node for extrinsic calibration.";
        return;
    }

    const xmlpp::Element* root = dynamic_cast<const xmlpp::Element*>(node);

    if (!root)
    {
        throw "xml node is not a element (dynamic_cast fails)";
        return;
    }

    const xmlpp::Node::NodeList nodes(root->get_children());

    for (xmlpp::Node::NodeList::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        const xmlpp::Element* elm(dynamic_cast<xmlpp::Element*>(*it));

        if (!elm)
            continue;

        if (elm->get_name() == TAG_ROTATION)
        {
            const xmlpp::Node* child(this->getChild(elm, "mat"));

            if (child)
                _R = MatD(child);
        }
        else if (elm->get_name() == TAG_TRANSLATION)
        {
            const xmlpp::Node* child(this->getChild(elm, "vec"));

            if (child)
                _t = VecD(child);
        }
    }
}

void ExtrinsicCalibration::createXml(xmlpp::Node* node) const
{
    xmlpp::Element* root(node->add_child(TAG_EXTRINSIC));
    xmlpp::Element* tagRot(root->add_child(TAG_ROTATION));
    xmlpp::Element* tagTrans(root->add_child(TAG_TRANSLATION));

    _R.createXml(tagRot);
    _t.createXml(tagTrans);
}

const xmlpp::Node* ExtrinsicCalibration::getChild(const xmlpp::Node* parent, const std::string& child)
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

bool ExtrinsicCalibration::valid(void) const
{
    if (_R.rows() != 3 || _R.cols() != 3 || _t.size() != 3)
        return false;

    if (_R.det() < 0.95)
        return false;

    return true;
}

void ExtrinsicCalibration::projectCoordsToPoints(const std::vector<Point3D>& coords, const CameraCalibration& cameraCali, std::vector<PointD>& points)
{
    std::cout << __PRETTY_FUNCTION__ << std::endl;
    /* check if calibrations are valid */
    if (!this->valid() || !cameraCali.valid())
    {
        throw "ExtrinsicCalibration : calibrations not valid.";
        return;
    }

    /* build one matrix for rotation and translation */
    MatD rt(3, 4);

    for (unsigned int row = 0; row < _R.rows(); row++)
        for (unsigned int col = 0; col < _R.cols(); col++)
            rt.at(row, col) = _R.at(row, col);

    for (unsigned int row = 0; row < _t.size(); row++)
        rt.at(row, 3) = _t.at(row);

    /* compute points */
    points.resize(coords.size());
    std::vector<Point3D>::const_iterator coord(coords.begin());
    std::vector<PointD>::iterator point(points.begin());

    const double k1(cameraCali.distortion().at(0));
    const double k2(cameraCali.distortion().at(1));
    const double k3(cameraCali.distortion().at(4));
    const double p1(cameraCali.distortion().at(2));
    const double p2(cameraCali.distortion().at(3));

    for (; coord < coords.end(); ++coord, ++point)
    {
        MatD p3D(4, 1);
        p3D.at(0, 0) = (*coord).x();
        p3D.at(1, 0) = (*coord).y();
        p3D.at(2, 0) = (*coord).z();
        p3D.at(3, 0) = 1.0;

        MatD p2D = cameraCali.intrinsic() * rt * p3D;
        p2D /= p2D.at(2, 0);
        double r(::sqrt(::pow(p2D.at(0, 0), 2) + ::pow(p2D.at(1, 0), 2)));
//        (*point).setX(p2D.at(0, 0) * (1.0 + k1 * ::pow(r, 2) + k2 * ::pow(r, 4) + k3 * ::pow(r, 6))  +  2 * p1 * p2D.at(1, 0) + p2 * (::pow(r, 2) + 2 * ::pow(p2D.at(0, 0), 2)));
//        (*point).setY(p2D.at(1, 0) * (1.0 + k1 * ::pow(r, 2) + k2 * ::pow(r, 4) + k3 * ::pow(r, 6))  +  p1 * (::pow(r, 2) + 2 * ::pow(p2D.at(1, 0), 2)) + 2 * p2 * p2D.at(0, 0));

//        if (p2D.at(0, 0) >= 0 && p2D.at(1, 0) >= 0)
//        {
//            std::cout << "Point3D = " << p3D << std::endl;
//            std::cout << "Point2D = " << p2D << std::endl;
//            std::cout << "x' = " << point->x() << ", y' = " << point->y() << std::endl;
//        }
    }
}

} // end namespace obvious
