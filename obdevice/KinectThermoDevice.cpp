#include "KinectThermoDevice.h"
#include "Kinect.h"
#include "SuperPoint.h"

#include <libxml++/libxml++.h>
#include <iostream>

namespace obvious {

KinectThermoDevice::KinectThermoDevice(const std::string& configKinect, const std::string& configThermo, const xmlpp::Node* calibration)
    : KinectDevice(configKinect),
      ThermoDevice(configThermo),
      m_RT(3, 4)
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

    for (unsigned int row = 0; row < m_RT.rows(); row++)
        for (unsigned int col = 0; col < m_RT.cols() - 1; col++)
            m_RT.at(row, col) = m_R.at(row, col);

    for (unsigned int row = 0; row < m_RT.rows(); row++)
        m_RT.at(row, m_RT.cols() - 1) = m_T.at(row, 0);

    std::cout << "RT: " << m_RT;
}

KinectThermoDevice::~KinectThermoDevice(void)
{

}

bool KinectThermoDevice::grab(void)
{
    this->deletePoints();

    bool state = ThermoDevice::grab() && this->device().grab();
//    bool state = ThermoDevice::grab() && KinectDevice::grab();

    if (!state)
    {
        std::cout << __PRETTY_FUNCTION__  << std::endl;
        std::cout << "Can't grab Kinect or Thermo!" << std::endl;
        throw "Can't grab Kinect or Thermo!";

        return false;
    }

    KinectDevice::m_rgb = this->device().getMat();
    const double* coords = this->device().getCoords();
    MatD mat(m_cameraMatrix * m_RT);
    const unsigned int divU = KinectDevice::m_rgb.cols() / ThermoDevice::m_rgb.cols();
    const unsigned int divV = KinectDevice::m_rgb.rows() / ThermoDevice::m_rgb.rows();

    for (unsigned int row = 0, i = 0; row < KinectDevice::m_rgb.rows(); row++)
    {
        for (unsigned int col = 0; col < KinectDevice::m_rgb.cols(); col++, i += 3)
        {
            MatD point(4, 1);
            point.at(0, 0) = coords[i];
            point.at(1, 0) = coords[i + 1];
            point.at(2, 0) = coords[i + 2];
            point.at(3, 0) = 1.0;

//            std::cout << "MatD uv = mat * point;" << std::endl;
            MatD uv = mat * point;
            const unsigned int u = uv.at(0, 0) / divU;
            const unsigned int v = uv.at(1, 0) / divV;

            if (u < ThermoDevice::m_rgb.cols() && v < ThermoDevice::m_rgb.rows())
            {
//                std::cout << "m_points.push_back(new SuperPoint3D(coords[i], coords[i + 1], coords[i + 2], 0, ThermoDevice::m_rgb.rgb(u, v)));" << std::endl;
                m_points.push_back(new SuperPoint3D(coords[i], coords[i + 1], coords[i + 2], 0, ThermoDevice::m_rgb.rgb(v, u)));
            }
            else
            {
//                std::cout << "m_points.push_back(new RgbPoint3D(coords[i], coords[i + 1], coords[i + 2], KinectDevice::m_rgb.rgb(row, col)));" << std::endl;
                m_points.push_back(new RgbPoint3D(coords[i], coords[i + 1], coords[i + 2], KinectDevice::m_rgb.rgb(row, col)));
            }
        }
    }

    return state;
}

} // end namespace obvious
