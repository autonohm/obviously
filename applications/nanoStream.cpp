#include <iostream>
//#include "obgraphic/Obvious3D.h"
#include "obgraphic/Obvious2D.h"
#include "obdevice/CamNano.h"

using namespace std;
using namespace obvious;

//Obvious3D*        _viewer;
Obvious2D*        _viewer2D;
//VtkCloud*         _cloud;
CamNano*          _nano;
bool             _pause       = false;
bool             _showNormals = false;

//class vtkTimerCallback : public vtkCommand
//{
//public:
//  static vtkTimerCallback *New()
//  {
//    vtkTimerCallback *cb = new vtkTimerCallback;
//    return cb;
//  }
//
//  virtual void Execute(vtkObject *vtkNotUsed(caller), unsigned long eventId,  void *vtkNotUsed(callData))
//  {
//    if(!_pause)
//    {
//      if(_nano->grab())
//      {
////        std::cout << "Frame rate: \t\t" << _nano->getFrameRate() << std::endl;
//        _cloud->setCoords(_nano->getValidCoords(), _nano->getValidSize(), 3);
//        std::cout << "valid size: " << _nano->getValidSize() << std::endl;
////        _cloud->setColors(_nano->getRGB(),    _nano->getCols()*_nano->getRows(), 3);
//        _viewer->update();
//      }
//    }
//  }
//
//private:
//
//};


int main(int argc, char* argv[])
{
  _nano       = new CamNano();
//  _cloud      = new VtkCloud();
//  _viewer     = new Obvious3D("Nano Stream 3D", 1024, 768, 0, 0);
  _viewer2D   = new Obvious2D(1024, 768, "Depth Image");

  _nano->showParameters();
  unsigned char* _img = new unsigned char[_nano->getCols()*_nano->getRows()*3];
  while(_viewer2D->isAlive())
  {
    _img = _nano->getImage();
    _viewer2D->draw(_img, _nano->getCols(), _nano->getRows(), 3);
  }

//
//  //_nano->setIntegrationTime(200);
//  _viewer->addCloud(_cloud);
//
//  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
//  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer->getWindowInteractor();
//  interactor->AddObserver(vtkCommand::TimerEvent, cb);
//
//  interactor->CreateRepeatingTimer(1);
//  _viewer->startRendering();

  delete _img;
//  delete _cloud;
  delete _nano;

  return 0;
}
