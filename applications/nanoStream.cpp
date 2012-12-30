#include <iostream>
#include "obgraphic/Obvious3D.h"
#include "obdevice/CamNano.h"

using namespace std;
using namespace obvious;

Obvious3D*        _viewer;
VtkCloud*         _cloud;
CamNano*          _nano;
bool             _pause       = false;
bool             _showNormals = false;

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
      if(_nano->grab())
      {
        std::cout << "Frame rate: \t\t" << _nano->getFrameRate() << std::endl;
        _cloud->setCoords(_nano->getCoords(), _nano->getValidSize(), 3);
        _viewer->update();
      }
    }
  }

private:

};


int main(int argc, char* argv[])
{
  _nano       = new CamNano();
  _cloud      = new VtkCloud();
  _viewer     = new Obvious3D("Nano Stream 3D", 1024, 768, 0, 0);

  _nano->showParameters();
  _nano->setIntegrationTime(600);
  _viewer->addCloud(_cloud);

  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);

  interactor->CreateRepeatingTimer(30);
  _viewer->startRendering();

  delete _cloud;
  delete _nano;

  return 0;
}
