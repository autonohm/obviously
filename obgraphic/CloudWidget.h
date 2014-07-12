#ifndef __CLOUD_WIDGET__
#define __CLOUD_WIDGET__

#include <QVTKWidget.h>
#include <vtkSmartPointer.h>
#include <vtkPlaneSource.h>

#include "obcore/base/PointCloud.h"

class vtkRenderer;
class vtkActor;

namespace obvious {

class CloudWidget : public QVTKWidget
{
    Q_OBJECT

public:

    enum Mode {
        Default = 0,
        Rgb,
        Thermal,
        CountMode
    };

    CloudWidget(QWidget* parent = 0);
    ~CloudWidget(void);

    static QString modeToString(const Mode mode);
    void setMode(const Mode mode) { _mode = mode; }
    Mode mode(void) const { return _mode; }
    void setMinTemperature(const float min) { _minTemperature = min; }
    float minTemperature(void) const { return _minTemperature; }
    void setMaxTemperature(const float max) { _maxTemperature = max; }
    float maxTemperature(void) const { return _maxTemperature; }
    void setAutoTemperatureRange(const bool enable) { _autoRange = enable; }
    bool autoTemperatureRange(void) const { return _autoRange; }

public slots:
    void setCloud(const PointCloud<Point>& cloud);

private:
    vtkSmartPointer<vtkRenderer> _renderer;
    vtkSmartPointer<vtkActor>    _actorCloud;
    vtkSmartPointer<vtkActor>    _actorLines;
    vtkSmartPointer<vtkActor>    _actorAxes;
    vtkSmartPointer<vtkActor>    _actorVector0;
    vtkSmartPointer<vtkActor>    _actorVector1;
    std::vector<vtkSmartPointer<vtkActor> > _actorPlane;

    Mode _mode;
    float _minTemperature;
    float _maxTemperature;
    bool _autoRange;
};

} // end namespace obvious

#endif
