#include <iostream>
#include "obgraphic/Obvious3D.h"
#include "obdevice/Xtion.h"
#include "obdevice/XtionDevice.h"

using namespace std;
using namespace obvious;

Obvious3D*        _viewer;
VtkCloud*         _cloud;
Xtion*            _xtion;
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
        _cloud->setCoords(_xtion->getCoords(), _xtion->getCols()*_xtion->getRows(), 3);
        _cloud->setColors(_xtion->getRGB(),    _xtion->getCols()*_xtion->getRows(), 3);
        _viewer->update();
      }
    }
  }

private:

};


int main(int argc, char* argv[])
{
  _xtion      = new Xtion(argv[1]);
  _cloud      = new VtkCloud();
  _viewer     = new Obvious3D("Xtion Stream 3D", 1024, 768, 0, 0);

  _viewer->addCloud(_cloud);

  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);

  interactor->CreateRepeatingTimer(30);
  _viewer->startRendering();

  delete _cloud;
  delete _xtion;

  return 0;
}
