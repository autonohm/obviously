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
#include <vtkLineSource.h>
#include <vtkLine.h>
#include <vtkTransform.h>
#include <vtkPlaneSource.h>
#include <vtkSphereSource.h>
#include <vtkLight.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

#include <iostream>
#include <string>
#include <map>
#include <ctime>

#include "obcore/math/mathbase.h"
#include "obcore/base/Logger.h"

namespace obvious
{

Obvious3D* _this;

struct StrIncVariable
{
  double* ptr;
  double inc;
};

std::map<std::string, fptrKeyboardCallback> _mCallback;
std::map<std::string, std::string> _mDesc;
std::map<std::string, bool*> _mFlipVariable;
std::map<std::string, StrIncVariable> _mIncVariable;

void showHelp()
{
  cout << "Default key table" << endl;
  cout << "-----------------" << endl;
  cout << "h: show (h)elp" << endl;
  cout << "r: (r)eset camera" << endl;
  cout << "s: (s)ave screenshot (to /tmp directory)" << endl;
  cout << "q: (q)uit application" << endl;
  cout << "+: increase point size of added clouds" << endl;
  cout << "-: decrease point size of added clouds" << endl << endl;


  std::map<std::string, fptrKeyboardCallback>::iterator iter = _mCallback.begin();
  std::map<std::string, std::string>::iterator iter2 = _mDesc.begin();

  if(iter != _mCallback.end())
  {
    cout << "Custom keys" << endl;
    cout << "----------------" << endl;

    while(iter != _mCallback.end())
    {
        cout << iter->first << ": " << iter2->second << endl;
        ++iter;
        ++iter2;
    }
    cout << endl;
  }
}

void keypressCallback (vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
{
  vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(caller);
  //cout << "Key pressed: " << std::string(iren->GetKeySym()) << endl;

  if(std::string(iren->GetKeySym()).compare(std::string("h"))==0)
  {
    showHelp();
  }
  else if(std::string(iren->GetKeySym()).compare(std::string("s"))==0)
  {
    _this->screenshot();
  }
  else
  {
    std::string key = std::string(iren->GetKeySym());
    if(_mCallback.find(key)!=_mCallback.end())
    {
      fptrKeyboardCallback fptr = _mCallback[key];
      if(fptr!=NULL)(*fptr)();
    }

    if(_mFlipVariable.find(key)!=_mFlipVariable.end())
    {
      bool* flip = _mFlipVariable[key];
      if(flip!=NULL)
        *flip = !*flip;
    }

    if(_mIncVariable.find(key)!=_mIncVariable.end())
    {
      StrIncVariable inc = _mIncVariable[key];
      if(inc.ptr != NULL)
      {
        *(inc.ptr) += inc.inc;
      }
    }
  }
}

void cloudCallback (vtkObject* caller, long unsigned int eventId, void* clientData, void* callData)
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

Obvious3D::Obvious3D(const char* windowName, unsigned int sizx, unsigned int sizy, unsigned int posx, unsigned int posy, double rgb[])
{
  _this = this;

  _renderer = vtkSmartPointer<vtkRenderer>::New();
  if(rgb)
    _renderer->SetBackground(rgb[0], rgb[1], rgb[2]);
  else
    _renderer->SetBackground(0.0, 0.0, 0.0);
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

  _sensor_axes = NULL;

  _renderer->GetActiveCamera()->Yaw(180);

  _mCallback.clear();
  _mDesc.clear();
}

Obvious3D::~Obvious3D()
{

}

void Obvious3D::addCloud(VtkCloud* cloud, bool pickable, unsigned int pointsize, double opacity)
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
  actor->GetProperty()->SetOpacity(opacity);

  cloud->setActor(actor);

  _renderer->AddActor(actor);
  _renderer->ResetCamera();
}

void Obvious3D::addLines(double** coordsStart, double** coordsEnd, unsigned int size, double rgb[])
{
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
  for(unsigned int i=0; i<size; i++)
  {
    pts->InsertNextPoint(coordsStart[i][0], coordsStart[i][1], coordsStart[i][2]);
    pts->InsertNextPoint(coordsEnd[i][0], coordsEnd[i][1], coordsEnd[i][2]);
    vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
    line->GetPointIds()->SetId(0,2*i);
    line->GetPointIds()->SetId(1,2*i+1);
    lines->InsertNextCell(line);
  }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(pts);
  polydata->SetLines(lines);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput(polydata);

  vtkActor* actor = vtkActor::New();
  actor->SetMapper(mapper);
  if(rgb)
    actor->GetProperty()->SetColor(rgb[0], rgb[1], rgb[2]);
  else
    actor->GetProperty()->SetColor(0.0, 0.0, 0.0);
  actor->GetProperty()->SetLineWidth(2.0);
  actor->GetProperty()->SetInterpolationToPhong();
  _renderer->AddActor(actor);
}

void Obvious3D::addPlane(double origin[3], double axis1[3], double axis2[3], unsigned int resX, unsigned int resY, double rgb[])
{
  vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
  planeSource->SetOrigin(origin[0], origin[1], origin[2]);
  planeSource->SetPoint1(axis1[0], axis1[1], axis1[2]);
  planeSource->SetPoint2(axis2[0], axis2[1], axis2[2]);
  planeSource->SetResolution(resX, resY);
  planeSource->Update();

  vtkPolyData* plane = planeSource->GetOutput();

  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
#if VTK_MAJOR_VERSION <= 5
  mapper->SetInput(plane);
#else
  mapper->SetInputData(plane);
#endif

  vtkProperty* prop = vtkProperty::New();
  //prop->SetRepresentation(VTK_WIREFRAME);
  if(rgb)
    prop->SetColor(rgb[0], rgb[1], rgb[2]);
  else
    prop->SetColor(0.8, 0.8, 0.95);
  prop->SetAmbient(0.8);
  prop->SetDiffuse(0.0);
  prop->SetSpecular(0.0);
  prop->SetOpacity(1.0);

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->SetProperty(prop);
  _renderer->AddActor(actor);
}

void Obvious3D::addSphere(double center[3], double radius, double rgb[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(center[0], center[1], center[2]);
  sphereSource->SetRadius(radius);
  sphereSource->SetStartPhi(-135.0);
  sphereSource->SetEndPhi(135.0);
  sphereSource->SetThetaResolution(360);
  sphereSource->SetPhiResolution(270);
  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkProperty* prop = vtkProperty::New();
  //prop->SetRepresentation(VTK_WIREFRAME);
  if(rgb)
    prop->SetColor(rgb[0], rgb[1], rgb[2]);
  else
    prop->SetColor(0.8, 0.8, 0.95);
  //prop->SetAmbient(0.3);
  prop->SetDiffuse(0.8);
  //prop->SetSpecular(0.4);
  prop->SetOpacity(1.0);


  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);
  actor->SetProperty(prop);
  _renderer->AddActor(actor);
}

void Obvious3D::addLight(double pos[3], double focal[3])
{
  vtkSmartPointer<vtkLight> light = vtkSmartPointer<vtkLight>::New();
  light->SetFocalPoint(focal[0], focal[1], focal[2]);
  light->SetPosition(pos[0], pos[1], pos[2]);
  light->PositionalOff();
  _renderer->AddLight(light);
}

void Obvious3D::setFrustum(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
  vtkSmartPointer<vtkPlanes> planes = vtkSmartPointer<vtkPlanes>::New();
  planes->SetBounds(xmin, xmax, ymin, ymax, zmin, zmax);
  _frust->SetFrustum(planes);
  _frust->Update();
}

void Obvious3D::addAxisAlignedCube(double xmin, double xmax, double ymin, double ymax, double zmin, double zmax)
{
  /**
   * Lines of frustum bounding box
   */
  vtkSmartPointer<vtkLineSource> cube[12];

  for(int i=0; i<12; i++)
    cube[i] = vtkSmartPointer<vtkLineSource>::New();

  cube[0]->SetPoint1(xmin, ymin, zmin);
  cube[0]->SetPoint2(xmax, ymin, zmin);

  cube[1]->SetPoint1(xmin, ymax, zmin);
  cube[1]->SetPoint2(xmax, ymax, zmin);

  cube[2]->SetPoint1(xmin, ymin, zmax);
  cube[2]->SetPoint2(xmax, ymin, zmax);

  cube[3]->SetPoint1(xmin, ymax, zmax);
  cube[3]->SetPoint2(xmax, ymax, zmax);

  cube[4]->SetPoint1(xmin, ymin, zmin);
  cube[4]->SetPoint2(xmin, ymax, zmin);

  cube[5]->SetPoint1(xmin, ymin, zmax);
  cube[5]->SetPoint2(xmin, ymax, zmax);

  cube[6]->SetPoint1(xmax, ymin, zmin);
  cube[6]->SetPoint2(xmax, ymax, zmin);

  cube[7]->SetPoint1(xmax, ymin, zmax);
  cube[7]->SetPoint2(xmax, ymax, zmax);

  cube[8]->SetPoint1(xmin, ymin, zmin);
  cube[8]->SetPoint2(xmin, ymin, zmax);

  cube[9]->SetPoint1(xmin, ymax, zmin);
  cube[9]->SetPoint2(xmin, ymax, zmax);

  cube[10]->SetPoint1(xmax, ymin, zmin);
  cube[10]->SetPoint2(xmax, ymin, zmax);

  cube[11]->SetPoint1(xmax, ymax, zmin);
  cube[11]->SetPoint2(xmax, ymax, zmax);

  for(int i=0; i<12; i++)
  {
    vtkSmartPointer<vtkPolyDataMapper> map = vtkSmartPointer<vtkPolyDataMapper>::New();
    map->SetInput(cube[i]->GetOutput());
    vtkActor* actor = vtkActor::New();
    actor->SetMapper(map);
    actor->GetProperty()->SetColor(255, 255, 255);
    actor->GetProperty()->SetLineWidth(1.0);
    _renderer->AddActor(actor);
  }
}

bool Obvious3D::checkVariableRegistration(std::string key)
{
  bool found = (key.compare("h")==0 ||
                key.compare("r")==0 ||
                key.compare("s")==0 ||
                key.compare("q")==0 ||
                key.compare("plus")==0 ||
                key.compare("minus")==0);

  if(found)
  {
    LOGMSG(DBG_WARN, "Key " << key << " is already registered ... ignoring");
    return false;
  }

  return true;
}

bool Obvious3D::registerKeyboardCallback(const char key[], fptrKeyboardCallback fptr, const char desc[])
{
  std::string skey = std::string(key);
  if(!checkVariableRegistration(skey)) return false;
  _mCallback[skey] = fptr;
  _mDesc[skey] = std::string(desc);
  return true;
}

bool Obvious3D::registerFlipVariable(const char key[], bool* flip)
{
  std::string skey = std::string(key);
  if(!checkVariableRegistration(skey)) return false;
  _mFlipVariable[skey] = flip;
  return true;
}

bool Obvious3D::registerIncVariable(const char key[], double* var, double inc)
{
  std::string skey = std::string(key);
  if(!checkVariableRegistration(skey)) return false;
  StrIncVariable sinc;
  sinc.ptr = var;
  sinc.inc = inc;
  _mIncVariable[skey] = sinc;
  return true;
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

void Obvious3D::showAxes(bool show)
{
  vtkSmartPointer<vtkAxesActor> axes = vtkSmartPointer<vtkAxesActor>::New();
  axes->SetXAxisLabelText("Axes");
  axes->AxisLabelsOff();
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->Scale(0.2, 0.2, 0.2);
  axes->SetUserTransform(transform);
  if(show)
    _renderer->AddActor(axes);
  else
    _renderer->RemoveActor(axes);
}

void Obvious3D::showSensorPosition(const double* position)
{
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->Translate(position);
  transform->Scale(0.2, 0.2, 0.2);

  if (_sensor_axes == NULL)
  {
    _sensor_axes = vtkSmartPointer<vtkAxesActor>::New();
    _sensor_axes->SetUserTransform(transform);
    _sensor_axes->SetXAxisLabelText("Sensor");
    _sensor_axes->AxisLabelsOff();
    _renderer->AddActor(_sensor_axes);
  }
  else
  {
    _sensor_axes->SetUserTransform(transform);
  }
}

void Obvious3D::showSensorPose(const double* T)
{
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->SetMatrix(T);
  transform->Scale(0.2, 0.2, 0.2);

  if (_sensor_axes == NULL)
  {
    _sensor_axes = vtkSmartPointer<vtkAxesActor>::New();
    _sensor_axes->SetUserTransform(transform);
    _sensor_axes->SetXAxisLabelText("Sensor");
    _sensor_axes->AxisLabelsOff();
    _renderer->AddActor(_sensor_axes);
  }
  else
  {
    _sensor_axes->SetUserTransform(transform);
  }
}

void Obvious3D::showSensorPose(Matrix& T)
{
  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  double T_tmp[16];
  T.getData(T_tmp);
  transform->SetMatrix(T_tmp);
  transform->Scale(0.2, 0.2, 0.2);

  if (_sensor_axes == NULL)
  {
    _sensor_axes = vtkSmartPointer<vtkAxesActor>::New();
    _sensor_axes->SetUserTransform(transform);
    _sensor_axes->SetXAxisLabelText("Sensor");
    _sensor_axes->AxisLabelsOff();
    _renderer->AddActor(_sensor_axes);
  }
  else
  {
    _sensor_axes->SetUserTransform(transform);
  }
}

void Obvious3D::screenshot()
{
  vtkSmartPointer<vtkWindowToImageFilter> windowToImageFilter =  vtkSmartPointer<vtkWindowToImageFilter>::New();
  windowToImageFilter->SetInput(_renderWindow);
  windowToImageFilter->SetMagnification(2); //set the resolution of the output image (3 times the current resolution of vtk render window)
  windowToImageFilter->SetInputBufferTypeToRGBA(); //also record the alpha (transparency) channel
  windowToImageFilter->Update();

  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();

  std::string filename = "/tmp/screenshot";

  time_t rawtime;
  struct tm * timeinfo;
  char buffer [80];
  time (&rawtime);
  timeinfo = localtime (&rawtime);

  strftime(buffer, 80, " %c", timeinfo);
  filename.append(buffer);
  filename.append(".png");
  writer->SetFileName(filename.c_str());
  writer->SetInputConnection(windowToImageFilter->GetOutputPort());
  writer->Write();

  LOGMSG(DBG_DEBUG, "Screenshot saved to " << filename);
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
