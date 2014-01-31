
#include <string>
#include <sstream>

#include "obgraphic/VtkCloud.h"
#include "obgraphic/Obvious3D.h"
#include "obdevice/Kinect.h"
#include "obdevice/PclCloudInterface.h"

using namespace obvious; 

Kinect* _kinect = NULL; 
Obvious3D* _viewer3D;
unsigned int fileCounter = 0; 
bool saveToFile = false; 
VtkCloud* _scene = NULL; 

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
    if (_kinect->grab())
    {
      double* coords      = _kinect->getCoords();  
      const unsigned char* rgb  = _kinect->getRGB();  
      const unsigned int width  = _kinect->getCols(); 
      const unsigned int height = _kinect->getRows();  

      _scene->setCoords(coords,  width*height, 3);
      _scene->setColors(rgb, width*height, 3);
      _viewer3D->update();
      
      
      if(saveToFile)
      {
        std::string folder = "/tmp/hbs_";
        std::ostringstream fileNumber;
        fileNumber << fileCounter++; 
        std::string type   = ".pcd"; 
        std::string filename = folder + fileNumber.str() + type; 

        PclCloudInterface::save(coords, rgb, width, height, filename); 
        saveToFile = false; 
      }
    }
  }

private:

};

void _save(void)
{
  saveToFile = true; 
}

int main(int argc, char* argv[])
{
  _kinect = new Kinect(argv[1]);

  _kinect->grab(); 
  _kinect->grab(); 
  _kinect->grab(); 
  _kinect->grab(); 
  _kinect->grab(); 

  _viewer3D = new Obvious3D("3DMapper");

  _scene = new VtkCloud();

  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> interactor = _viewer3D->getWindowInteractor();
  interactor->AddObserver(vtkCommand::TimerEvent, cb);
  interactor->CreateRepeatingTimer(30);
  _viewer3D->registerKeyboardCallback("space", _save, 	"Save new Image");
  _viewer3D->addCloud(_scene); 
  _viewer3D->startRendering();

  delete _viewer3D; 


  delete _kinect; 
}
