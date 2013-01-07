#ifndef __EXTRINSIC_CALIBRATION__
#define __EXTRINSIC_CALIBRATION__

#include "obcore/math/MatD.h"
#include "obcore/math/VecD.h"
#include "obcore/Point.h"
#include "obcore/Point3D.h"
#include "obdevice/CameraCalibration.h"

#include <vector>

namespace xmlpp {
class Node;
}

namespace obvious {

class ExtrinsicCalibration
{
public:
    ExtrinsicCalibration(void) { }
    ExtrinsicCalibration(const MatD& R, const VecD& t) : _R(R), _t(t) { }
    ExtrinsicCalibration(const xmlpp::Node* node);

    void createXml(xmlpp::Node* node) const;

    const MatD& rotation(void) const { return _R; }
    void setRotation(const MatD& R) { R.copyTo(_R); }
    const VecD& translation(void) const { return _t; }
    void setTranslation(const VecD& t) { t.copyTo(_t); }
    bool valid(void) const;

    void projectCoordsToPoints(const std::vector<Point3D>& coords, const CameraCalibration& cameraCali, std::vector<PointD>& points);

private:
    const xmlpp::Node* getChild(const xmlpp::Node* parent, const std::string& child);

    MatD _R;
    VecD _t;
};

} // end namespace obvious

#endif
