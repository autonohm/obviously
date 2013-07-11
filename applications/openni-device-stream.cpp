#include <iostream>
#include "obgraphic/Obvious3D.h"
#include "obdevice/OpenNiDevice.h"

using namespace std;
using namespace obvious;

Obvious3D*     _viewer;
VtkCloud*      _cloud;
OpenNiDevice*  _device;
bool           _pause       = false;
bool           _showNormals = false;
double*        _coords;
unsigned char* _image;

class vtkTimerCallback : public vtkCommand
{
public:
    static vtkTimerCallback *New()
    {
        vtkTimerCallback *cb = new vtkTimerCallback;
        return cb;
    }

    virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId,  void *vtkNotUsed(callData))
    {
        if(!_pause)
        {
            if(_device->grab())
            {
                double* dst = _coords;

                for (std::vector<float>::const_iterator coord(_device->coords().begin()); coord < _device->coords().end(); ++coord, ++dst)
                    *dst = static_cast<double>(*coord);

                unsigned char* color = _image;
                MatRGB::const_iterator red  (_device->image().begin(MatRGB::Red  ));
                MatRGB::const_iterator green(_device->image().begin(MatRGB::Green));
                MatRGB::const_iterator blue (_device->image().begin(MatRGB::Blue ));

                for (int row = 0; row < _device->height(); row++)
                {
                    for (int col = 0; col < _device->width(); col++, ++red, ++green, ++blue)
                    {
                        *color++ = *red;
                        *color++ = *green;
                        *color++ = *blue;
                    }
                }

                _cloud->setCoords(_coords, _device->width() * _device->height(), 3);
                _cloud->setColors(_image,  _device->width() * _device->height(), 3);
                _viewer->update();
            }
        }
    }
};


int main(void)
{
    _device = new OpenNiDevice;
    _device->init();
    _coords = new double[_device->width() * _device->height() * 3];
    _image  = new unsigned char[_device->width() * _device->height() * 3];
    _cloud  = new VtkCloud();
    _viewer = new Obvious3D("OpenNI Stream 3D", 1024, 768, 0, 0);

    _viewer->addCloud(_cloud);

    vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
    vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer->getWindowInteractor();
    interactor->AddObserver(vtkCommand::TimerEvent, cb);

    interactor->CreateRepeatingTimer(30);
    _viewer->startRendering();

    delete _cloud;
    delete _device;
    delete [] _coords;
    delete [] _image;
}
