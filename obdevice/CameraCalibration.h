#ifndef __CAMERA_CALIBRATION__
#define __CAMERA_CALIBRATION__

#include "obcore/math/MatD.h"
#include "obcore/math/MatRGB.h"
#include "obcore/math/VecD.h"

#include <string>

namespace xmlpp {
class Node;
}

namespace obvious {

class CameraCalibration
{
public:
    CameraCalibration(const MatD& intrinsic, const VecD& distortion) : _intrinsic(intrinsic), _distortion(distortion) { }
    CameraCalibration(const xmlpp::Node* node);

    void createXml(xmlpp::Node* node) const;

    const MatD& intrinsic(void) const          { return _intrinsic; }
    void setIntrinsic(const MatD& intrinsic)   { intrinsic.copyTo(_intrinsic); }
    const VecD& distortion(void) const         { return _distortion; }
    void setDistortion(const VecD& distCoeffs) { distCoeffs.copyTo(_distortion); }

    void correctImage(MatRGB& image);

private:
    const xmlpp::Node* getChild(const xmlpp::Node* parent, const std::string& child);

    MatD _intrinsic;
    VecD _distortion;
};

} // end namespace obvious

#endif
