
#include <string>
#include <sstream>

#include "obgraphic/VtkCloud.h"
#include "obgraphic/Obvious3D.h"
#include "obdevice/Kinect.h"
#include "obdevice/PclCloudInterface.h"

using namespace obvious; 

Obvious3D* _viewer3D;
unsigned int fileCounter = 0; 
bool updateImage = false; 
VtkCloud* _scene = NULL; 

std::vector<double*>        coords;
std::vector<unsigned char*> rgb;
std::vector<size_t>         width;
std::vector<size_t>         height;
std::vector<int>::size_type i = 0; 

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
      if(updateImage)
      {
        _scene->setCoords(coords[i],  width[i]*height[i], 3);
        _scene->setColors(rgb[i],     width[i]*height[i], 3);
        _viewer3D->update();
        updateImage = false;
      }
  }

private:

};

void _load(void)
{
  fileCounter++; 
  i++; 
  updateImage = true; 
}

int main(int argc, char* argv[])
{
  _viewer3D = new Obvious3D("3DMapper");

  PclCloudInterface::loadAllCloudsFromDirectory(argv[1], coords, rgb, width, height); 


  std::cout << coords.size() << std::endl; 
  _scene = new VtkCloud();

  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer3D->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);
  interactor->CreateRepeatingTimer(30);
  _viewer3D->registerKeyboardCallback("space", _load, 	"Load new Image");
  _viewer3D->addCloud(_scene); 
  _viewer3D->startRendering();

  delete _viewer3D; 
}
