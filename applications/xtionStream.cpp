#include <iostream>
#include "obgraphic/Obvious3D.h"
#include "obdevice/OpenNiDevice.h"

using namespace std;
using namespace obvious;

Obvious3D*        _viewer;
VtkCloud*         _cloud;
OpenNiDevice*     _xtion;
bool              _pause       = false;
bool              _showNormals = false;

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
      if(_xtion->grab())
      {
    	std::vector<float> coords =  _xtion->coords();
    	double* coordsD = new double[_xtion->width()* _xtion->height() * 3];

    	for(int i=0 ; i<_xtion->width()*_xtion->height()*3 ; i++)
    		coordsD[i] = coords[i];

        _cloud->setCoords(coordsD, _xtion->width()*_xtion->height(), 3);
        _cloud->setColors(_xtion->rgb(),    _xtion->width()*_xtion->height(), 3);
        _viewer->update();
      }
    }
  }

private:

};


int main(int argc, char* argv[])
{
  _xtion      = new OpenNiDevice();
  _cloud      = new VtkCloud();
  _viewer     = new Obvious3D("Xtion Stream 3D", 1024, 768, 0, 0);

  _viewer->addCloud(_cloud);

  _xtion->init();

  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);

  interactor->CreateRepeatingTimer(30);
  _viewer->startRendering();

  delete _cloud;
  delete _xtion;

  return 0;
}
