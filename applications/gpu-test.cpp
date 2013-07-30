#include "obcore/base/PointCloud.h"
#include "obdevice/OpenNiDevice.h"
#include "obgpu/PointCloud.h"
#include "obgpu/filter/PassThrough.h"
#include "obgpu/features/NormalEstimator.h"
#include "obgraphic/Obvious3D.h"

#include <iostream>
#include <ctime>

using namespace obvious;

Obvious3D*     _viewer;
VtkCloud*      _cloudVtk;

class vtkTimerCallback : public vtkCommand
{
public:
    static vtkTimerCallback *New()
    {
        vtkTimerCallback *cb = new vtkTimerCallback;
        return cb;
    }

    vtkTimerCallback(void)
    : vtkCommand()
    {
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        _sensor.init();
        _cloud.resize(_sensor.width() * _sensor.height());
        _normals.resize(_cloud.size());
        _coords = new double[_cloud.size() * 3];
        _normalsD = new double[_cloud.size() * 3];
        _image = new unsigned char[_cloud.size() * 3];

        for (unsigned int i = 0; i < _cloud.size() * 3; i++)
            _image[i] = 0xff;

        _gpuNormals.upload(_normals);
    }

    virtual ~vtkTimerCallback(void)
    {
        delete [] _coords;
        delete [] _normalsD;
        delete [] _image;
    }

    virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId,  void *vtkNotUsed(callData))
    {
        if(_sensor.grab())
        {
            this->grab();

            const int clockBegin = clock();

            _gpuCloud.upload(_cloud);
            _estimator.setSource(&_gpuCloud);
            _estimator.estimate(_gpuNormals);
            _gpuNormals.download(_normals);

            this->copyToDouble(_cloud, _coords);
            this->copyToDouble(_normals, _normalsD);
            std::cout << "loop " << ": " << clock() - clockBegin << " clocks needed." << std::endl;

//            for (PointCloud<Normal>::const_iterator normal(_normals.begin()); normal < _normals.end(); ++normal)
//                std::cout << normal->x << ", " << normal->y << ", " << normal->z << "; ";
//            std::cout << std::endl;

            _cloudVtk->setCoords(_coords, _cloud.size(), 3);
            _cloudVtk->setNormals(_normalsD, _cloud.size(), 3);
            _cloudVtk->setColors(_image, _cloud.size(), 3);
            _viewer->update();
        }
    }

void grab(void)
{
    _sensor.grab();
    std::vector<float>::const_iterator coord(_sensor.coords().begin());

    for (PointCloud<PointXyz>::iterator point(_cloud.begin()); point < _cloud.end(); ++point)
    {
        point->x = *coord; ++coord;
        point->y = *coord; ++coord;
        point->z = *coord; ++coord;
    }
}

private:
    void copyToDouble(const PointCloud<PointXyz>& cloud, double* target)
    {
        for (PointCloud<PointXyz>::const_iterator point(cloud.begin()); point < cloud.end(); ++point)
        {
            *target++ = point->x;
            *target++ = point->y;
            *target++ = point->z;
        }
    }

    void copyToDouble(const PointCloud<Normal>& normals, double* target)
    {
        for (PointCloud<Normal>::const_iterator normal(normals.begin()); normal < normals.end(); ++normal)
        {
            *target++ = normal->x;
            *target++ = normal->y;
            *target++ = normal->z;
//            *target++ = normal->curvature;
        }
    }

    OpenNiDevice _sensor;
    PointCloud<PointXyz> _cloud;
    PointCloud<Normal> _normals;
    gpu::PointCloud _gpuCloud;
    gpu::PointCloud _gpuNormals;
    gpu::filter::PassThrough _filter;
    gpu::features::NormalEstimator _estimator;
    double* _coords;
    double* _normalsD;
    unsigned char* _image;
};

namespace {
const int LOOPS = 1000;
}

using namespace obvious;

int main(void)
{
    _cloudVtk  = new VtkCloud();
    _viewer = new Obvious3D("OpenNI Stream 3D", 1024, 768, 0, 0);



    _viewer->addCloud(_cloudVtk);

    vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
    vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer->getWindowInteractor();
    interactor->AddObserver(vtkCommand::TimerEvent, cb);

    interactor->CreateRepeatingTimer(30);
    _viewer->startRendering();

    delete _cloudVtk;
    delete _viewer;
}
