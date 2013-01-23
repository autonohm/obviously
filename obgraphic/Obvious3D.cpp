#include "Obvious3D.h"

#include <vtkPointData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkCallbackCommand.h>
#include <vtkAlgorithmOutput.h>
#include <vtkDelaunay2D.h>
#include <vtkCleanPolyData.h>
#include <vtkCamera.h>
#include <vtkPlanes.h>
#include <vtkProperty.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>

#include <iostream>
#include <string>

#include <map>

namespace obvious
{

struct StrIncVariable
{
	double* ptr;
	double inc;
};

std::map<std::string, fptrKeyboardCallback> _mCallback;
std::map<std::string, bool*> _mFlipVariable;
std::map<std::string, StrIncVariable> _mIncVariable;

void keypressCallback ( vtkObject* caller, long unsigned int eventId, void* clientData, void* callData )
{
  vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(caller);
  fptrKeyboardCallback fptr = _mCallback[std::string(iren->GetKeySym())];
  if(fptr!=NULL)(*fptr)();

  //cout << std::string(iren->GetKeySym()) << endl;

  bool* flip = _mFlipVariable[std::string(iren->GetKeySym())];
  if(flip!=NULL)
    *flip = !*flip;

  StrIncVariable inc = _mIncVariable[std::string(iren->GetKeySym())];
  if(inc.ptr != NULL)
  {
	  *(inc.ptr) += inc.inc;
  }
}

void cloudCallback ( vtkObject* caller, long unsigned int eventId, void* clientData, void* callData )
{
  vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(caller);
  VtkCloud* cloud  = static_cast<VtkCloud*>(clientData);
  int size = cloud->getActor()->GetProperty()->GetPointSize();
  std::string key = iren->GetKeySym();
  if(key == "plus")
  {
    cloud->getActor()->GetProperty()->SetPointSize(++size);
    iren->Render();
  }
  else if(key == "minus")
  {
    if(size>1)
    {
      cloud->getActor()->GetProperty()->SetPointSize(--size);
      iren->Render();
    }
  }
}

Obvious3D::Obvious3D(const char* windowName, unsigned int sizx, unsigned int sizy, unsigned int posx, unsigned int posy)
{
  _renderer = vtkSmartPointer<vtkRenderer>::New();
  _renderer->SetBackground(.0, .0, .0);
  _renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  _renderWindow->AddRenderer(_renderer);
  _renderWindow->SetSize(sizx, sizy);
  _renderWindow->SetPosition(posx,posy);

  if(windowName == NULL) _renderWindow->SetWindowName("Obvious3D");
  else _renderWindow->SetWindowName(windowName);
  _renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  _renderWindowInteractor->SetRenderWindow(_renderWindow);
  vtkSmartPointer<vtkInteractorStyleSwitch> stlye = vtkInteractorStyleSwitch::SafeDownCast(_renderWindowInteractor->GetInteractorStyle());
  stlye->SetCurrentStyleToTrackballCamera();

  vtkSmartPointer<vtkCallbackCommand> cb = vtkSmartPointer<vtkCallbackCommand>::New();
  cb->SetCallback(keypressCallback);
  _renderWindowInteractor->AddObserver (vtkCommand::KeyPressEvent, cb);
  _renderWindowInteractor->Initialize();

  _renderer->GetActiveCamera()->Yaw(180);
}

Obvious3D::~Obvious3D()
{

}

void Obvious3D::addCloud(VtkCloud* cloud, bool pickable, unsigned int pointsize)
{
  vtkSmartPointer<vtkPolyData> polyData = cloud->getPolyData();

  vtkSmartPointer<vtkCallbackCommand> cb = vtkSmartPointer<vtkCallbackCommand>::New();
  cb->SetCallback(cloudCallback);
  cb->SetClientData(cloud);
  _renderWindowInteractor->AddObserver (vtkCommand::KeyPressEvent, cb);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(polyData->GetProducerPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  if(!pickable)actor->PickableOff();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetPointSize(pointsize);

  cloud->setActor(actor);

  _renderer->AddActor(actor);
  _renderer->ResetCamera();
}

void Obvious3D::setFrustum(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
  vtkSmartPointer<vtkPlanes> planes = vtkSmartPointer<vtkPlanes>::New();
  planes->SetBounds(xmin, xmax, ymin, ymax, zmin, zmax);
  _frust->SetFrustum(planes);
  _frust->Update();
}

void Obvious3D::registerKeyboardCallback(const char key[], fptrKeyboardCallback fptr)
{
  std::string skey = std::string(key);
  _mCallback[skey] = fptr;
}

void Obvious3D::registerFlipVariable(const char key[], bool* flip)
{
  std::string skey = std::string(key);
  _mFlipVariable[skey] = flip;
}

void Obvious3D::registerIncVariable(const char key[], double* var, double inc)
{
  std::string skey = std::string(key);
  StrIncVariable sinc;
  sinc.ptr = var;
  sinc.inc = inc;
  _mIncVariable[skey] = sinc;
}

void Obvious3D::update()
{
  _renderWindow->Render();
}

void Obvious3D::startRendering()
{
  update();
  _renderWindowInteractor->Start();
}

void Obvious3D::showAxes(bool show )
{
  vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
	if(show)
	{
	  _renderer->AddActor(axes);
	}
	else
	{
	  _renderer->RemoveActor(axes);
	}
}

vtkSmartPointer<vtkRenderer> Obvious3D::getRenderer()
{
  return _renderer;
}

vtkSmartPointer<vtkRenderWindowInteractor> Obvious3D::getWindowInteractor()
{
  return _renderWindowInteractor;
}

}
