#include <iostream>
#include "obgraphic/Obvious3D.h"
#include "obdevice/KinectPlayback.h"

using namespace std;
using namespace obvious;

Obvious3D* _viewer;
VtkCloud*  _cloud;
KinectPlayback*    _kinect;

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
    if(!_kinect->eof())
    {
      if(_kinect->grab())
      {
        int size = _kinect->getRows() * _kinect->getCols();
        _cloud->setCoords(_kinect->getCoords(), size, 3);
        _cloud->setColors(_kinect->getRGB(),    size, 3);
        _viewer->update();
      }
    }
  }

private:

};

void resetFilestream()
{
  _kinect->reset();
}

int main(int argc, char* argv[])
{
  if(argc!=2)
  {
    cout << "usage: " << argv[0] << " <playback.dat>" << endl;
    return -1;
  }

  _kinect = new KinectPlayback(argv[1]);
  _cloud  = new VtkCloud();
  _viewer = new Obvious3D();
  _viewer->registerKeyboardCallback("space", resetFilestream);
  _viewer->addCloud(_cloud);

  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);

  interactor->CreateRepeatingTimer(30);
  _viewer->startRendering();

  delete _cloud;
  delete _kinect;

  return 0;
}
