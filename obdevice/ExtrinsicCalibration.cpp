#include "ExtrinsicCalibration.h"

#include <libxml++/libxml++.h>

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

    for (const xmlpp::Node::NodeList::const_iterator it = nodes.begin(); it < nodes.end(); ++it)
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

} // end namespace obvious
