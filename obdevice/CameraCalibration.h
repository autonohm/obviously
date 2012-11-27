#ifndef __CAMERA_CALIBRATION__
#define __CAMERA_CALIBRATION__

#include "obcore/math/MatD.h"
#include "obcore/math/MatRGB.h"
#include "obcore/math/VecD.h"

namespace xmlpp {
class Node;
}

namespace obvious {

class CameraCalibration
{
public:
    CameraCalibration(const MatD& intrinsic, const VecD& distCoeffs) : _intrinsic(intrinsic), _distCoeffs(distCoeffs) { }
    CameraCalibration(const xmlpp::Node* node);

    void createXml(xmlpp::Node* node);

    const MatD& intrinsic(void) const          { return _intrinsic; }
    void setIntrinsic(const MatD& intrinsic)   { intrinsic.copyTo(_intrinsic); }
    const VecD& distCoeffs(void) const         { return _distCoeffs; }
    void setDistCoeffs(const VecD& distCoeffs) { distCoeffs.copyTo(_distCoeffs); }

    void correctImage(MatRGB& image);

private:
    MatD _intrinsic;
    VecD _distCoeffs;
};

} // end namespace obvious

#endif
