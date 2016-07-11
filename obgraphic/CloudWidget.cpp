#include "CloudWidget.h"
//#include "IronPalette.h"

#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCamera.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkArrowSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkMath.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkIdList.h>
#include <vtkTriangle.h>
#include <vtkLookupTable.h>
#include <vtkFloatArray.h>
#include <vtkCellData.h>

#include <QDebug>

namespace obvious {

CloudWidget::CloudWidget(QWidget* parent)
    : QVTKWidget(parent),
      _renderer(vtkRenderer::New()),
      _actorCloud(vtkSmartPointer<vtkActor>::New()),
      _actorLines(vtkSmartPointer<vtkActor>::New()),
      _actorAxes(vtkSmartPointer<vtkActor>::New()),
      _actorVector0(vtkSmartPointer<vtkActor>::New()),
      _actorVector1(vtkSmartPointer<vtkActor>::New()),
      _mode(Default),
      _minTemperature(0.0),
      _maxTemperature(40.0),
      _autoRange(true)
{
    _renderer->SetBackground(0.0f, 0.0f, 0.0f);
    _renderer->GetActiveCamera()->Yaw(180);
    this->GetRenderWindow()->AddRenderer(_renderer);

    _actorCloud->GetProperty()->SetPointSize(1);

    _renderer->AddActor(_actorCloud);
    _renderer->AddActor(_actorLines);
    _renderer->AddActor(_actorAxes);
    _renderer->AddActor(_actorVector0);
    _renderer->AddActor(_actorVector1);

    /*
    for (unsigned int i = 0; i < _planes; i++)
    {
        _actorPlane.push_back(vtkSmartPointer<vtkActor>::New());
        _renderer->AddActor(_actorPlane[i]);
    }
    */
    _renderer->ResetCamera();
}

CloudWidget::~CloudWidget(void)
{

}

void CloudWidget::setCloud(const PointCloud<Point>& cloud)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colours = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colours->SetNumberOfComponents(3);


    for (PointCloud<Point>::const_iterator point(cloud.begin()); point < cloud.end(); ++point)
    {
        unsigned char temp[3] = { 0xff, 0xff, 0xff };

        points->InsertNextPoint(point->x, point->y, point->z);
        colours->InsertNextTupleValue(temp);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->GetPointData()->SetNormals(NULL);
    polyData->GetPointData()->SetScalars(colours);

    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
#if VTK_MAJOR_VERSION <= 5
    glyphFilter->SetInputConnection(polyData->GetProducerPort());
#else
    glyphFilter->SetInputData(polyData);
#endif
    glyphFilter->Update();

    polyData->ShallowCopy(glyphFilter->GetOutput());
    points->Modified();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
    mapper->SetInputConnection(polyData->GetProducerPort());
#else
    mapper->SetInputData(polyData);
#endif

    _actorCloud->SetMapper(mapper);

    this->update();
}

void CloudWidget::setCloud(const PointCloud<PointRgb>& cloud)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colours = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colours->SetNumberOfComponents(3);


    for (PointCloud<PointRgb>::const_iterator point(cloud.begin()); point < cloud.end(); ++point)
    {
        unsigned char temp[3] = { point->r, point->g, point->b };

        points->InsertNextPoint(point->x, point->y, point->z);
        colours->InsertNextTupleValue(temp);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->GetPointData()->SetNormals(NULL);
    polyData->GetPointData()->SetScalars(colours);

    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();

#if VTK_MAJOR_VERSION <= 5
    glyphFilter->SetInputConnection(polyData->GetProducerPort());
#else
    glyphFilter->SetInputData(polyData);
#endif
    glyphFilter->Update();

    polyData->ShallowCopy(glyphFilter->GetOutput());
    points->Modified();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
    mapper->SetInputConnection(polyData->GetProducerPort());
#else
    mapper->SetInputData(polyData);
#endif

    _actorCloud->SetMapper(mapper);

    this->update();
}

/*
void CloudWidget::setCloud(pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud,
                           const std::vector<pcl::Vertices>& vertices)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colours = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colours->SetNumberOfComponents(3);

    // not the best code style here, but it works for the moment.
    float minTemperature;
    float maxTemperature;

    if (_mode == Thermal && _autoRange)
    {
        uint16_t extremums[2] = { 65534, 0 };

        for (pcl::PointCloud<pcl::PointXYZRGBL>::const_iterator it = cloud->begin(); it < cloud->end(); ++it)
        {
            if (it->label && it->label < extremums[0])
                extremums[0] = it->label;
            if (it->label > extremums[1])
                extremums[1] = it->label;
        }

        minTemperature = static_cast<float>(static_cast<int>(extremums[0]) - 1000) * 0.1f;
        maxTemperature = static_cast<float>(static_cast<int>(extremums[1]) - 1000) * 0.1f;
    }
    else
    {
        minTemperature = _minTemperature;
        maxTemperature = _maxTemperature;
    }

    const float factor = 240.0 / static_cast<float>(maxTemperature - minTemperature);

    for (pcl::PointCloud<pcl::PointXYZRGBL>::const_iterator it = cloud->begin(); it < cloud->end(); ++it)
    {
        unsigned char temp[3];

        points->InsertNextPoint(it->x, it->y, it->z);

        if (_mode == Thermal)
        {
            const float temperature = static_cast<float>(static_cast<int>(it->label) - 1000) * 0.1f;

            if (temperature < _minTemperature)
            {
                temp[0] = 0x00;
                temp[1] = 0x00;
                temp[2] = 0x00;
            }
            else if (temperature > _maxTemperature)
            {
                temp[0] = 0xff;
                temp[1] = 0xff;
                temp[2] = 0xff;
            }
            else
            {
                const unsigned int index = static_cast<unsigned int>((temperature - minTemperature) * factor);

                temp[0] = Iron[index][0];
                temp[1] = Iron[index][1];
                temp[2] = Iron[index][2];
            }
        }
        else
        {
            temp[0] = it->r;
            temp[1] = it->g;
            temp[2] = it->b;
        }

        colours->InsertNextTupleValue(temp);
    }


    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);

    if (vertices.size())
    {
        qDebug() << "set triangle to ploly data.";
        vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
        vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();

        vtkIdList* pIds = triangle->GetPointIds();
        pIds->SetId(0, 0);
        pIds->SetId(1, 0);
        pIds->SetId(2, 0);
        vtkIdType* pId0 = pIds->GetPointer(0);
        vtkIdType* pId1 = pIds->GetPointer(1);
        vtkIdType* pId2 = pIds->GetPointer(2);
        qDebug() << "got pointers.";
        for (std::vector<pcl::Vertices>::const_iterator vertice(vertices.begin());
             vertice < vertices.end();
             ++vertice)
        {
            *pId0 = vertice->vertices[0];
            *pId1 = vertice->vertices[1];
            *pId2 = vertice->vertices[2];

            triangles->InsertNextCell(triangle);
        }
        qDebug() << "before set polys.";
        polyData->SetPolys(triangles);
    }

    polyData->GetPointData()->SetNormals(NULL);
    polyData->GetPointData()->SetScalars(colours);

    if (!vertices.size())
    {
        vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
        glyphFilter->SetInputConnection(polyData->GetProducerPort());
        glyphFilter->Update();

        polyData->ShallowCopy(glyphFilter->GetOutput());
    }

    points->Modified();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(polyData->GetProducerPort());

    _actorCloud->SetMapper(mapper);

    this->update();
}
*/

QString CloudWidget::modeToString(const Mode mode)
{
    switch (mode)
    {
    case Default:
    default:
        return "Default";

    case Rgb:
        return "Rgb";

    case Thermal:
        return "Thermal";
    }
}

} // end namespace obvious
