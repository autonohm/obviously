#include "KinectThermoDevice.h"
#include "Kinect.h"

#include <libxml++/libxml++.h>
#include <iostream>

namespace obvious {

KinectThermoDevice::KinectThermoDevice(const std::string& configKinect, const std::string& configThermo, const xmlpp::Node* calibration)
    : KinectDevice(configKinect),
      ThermoDevice(configThermo)
{
    const xmlpp::Node::NodeList nodes = calibration->get_children();

    for (xmlpp::Node::NodeList::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
    {
        if ((*it)->get_name() == "kinect_camera")
        {
            std::cout << "found kinect" << std::endl;
            const xmlpp::Node::NodeList nodes = (*it)->get_children();

            for (xmlpp::Node::NodeList::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
            {
                if ((*it)->get_name() == "intrinsic")
                {
                    std::cout << "found intrinsic" << std::endl;
                    xmlpp::Node::NodeList nodes = (*it)->get_children();

                    for (xmlpp::Node::NodeList::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
                    {
                        std::cout << "found camera_matrix" << std::endl;

                        if((*it)->get_name() == "camera_matrix")
                        {
                            xmlpp::Node::NodeList nodes = (*it)->get_children();

                            for (xmlpp::Node::NodeList::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
                            {
                                if ((*it)->get_name() == "mat")
                                {
                                    std::cout << "fount mat" << std::endl;
                                    m_cameraMatrix = MatD(*it);
                                }
                            }
                        }
                    }
                }
            }
        }

        if ((*it)->get_name() == "stero_calibration")
        {
            const xmlpp::Node::NodeList nodes = (*it)->get_children();

            for (xmlpp::Node::NodeList::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
            {
                if ((*it)->get_name() == "rotation_matrix")
                {
                    const xmlpp::Node::NodeList nodes = (*it)->get_children();

                    for (xmlpp::Node::NodeList::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
                    {
                        if ((*it)->get_name() == "mat")
                            m_R = MatD(*it);
                    }
                }
                else if ((*it)->get_name() == "translation_vector")
                {
                    const xmlpp::Node::NodeList nodes = (*it)->get_children();

                    for (xmlpp::Node::NodeList::const_iterator it = nodes.begin(); it != nodes.end(); ++it)
                    {
                        if ((*it)->get_name() == "mat")
                            m_T = MatD(*it);
                    }
                }
            }
        }
    }

    std::cout << __PRETTY_FUNCTION__ << std::endl;
    std::cout << "cameraMatrix: " << m_cameraMatrix;
    std::cout << "R: " << m_R;
    std::cout << "T: " << m_T;
}

KinectThermoDevice::~KinectThermoDevice(void)
{

}

bool KinectThermoDevice::grab(void)
{
    bool state = ThermoDevice::grab() && this->device().grab();
//    bool state = ThermoDevice::grab() && KinectDevice::grab();

    MatD RT(4, 3);

    for (unsigned int row = 0; row < RT.rows(); row++)
        for (unsigned int col = 0; col < RT.cols() - 1; col++)
            RT.at(col, row) = m_R.at(col, row);

    for (unsigned int row = 0; row < RT.rows(); row++)
        RT.at(RT.cols() - 1, row) = m_T.at(0, row);

    std::cout << "RT: " << RT;

    return state;
}

} // end namespace obvious
