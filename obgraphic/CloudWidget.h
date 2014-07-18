/*********************************************************************************
*
* Copyright (C) 2014 by TH-Nürnberg
* Written by Christian Merkl <christian.merkl@th-nuernberg.de>
* All Rights Reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along
* with this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*
*********************************************************************************/
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
    void setCloud(const PointCloud<PointRgb>& cloud);

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
