#include <iostream>
#include "obgraphic/Obvious3D.h"
#include "obdevice/Kinect.h"
#include "obvision/normals/NormalsEstimator.h"
#include "obvision/mesh/TriangleMesh.h"

#include "obcore/base/Timer.h"

#include <vtkPolyDataMapper.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <vtkPointData.h>
#include <vtkCamera.h>

using namespace std;
using namespace obvious;

Kinect* _kinect;

vtkSmartPointer<vtkPolyData>          _polyData;
vtkSmartPointer<vtkPoints>            _points;
vtkSmartPointer<vtkCellArray>         _triangles;
vtkSmartPointer<vtkUnsignedCharArray> _colors;
vtkSmartPointer<vtkRenderWindow>      _renderWindow;

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
    if(_kinect->grab())
    {
      Timer t;
      int rows           = _kinect->getRows();
      int cols           = _kinect->getCols();
      int size           = rows * cols;
      double* coords     = _kinect->getCoords();
      unsigned char* rgb = _kinect->getRGB();
      bool* mask         = _kinect->getMask();

      TriangleMesh* mesh   = new TriangleMesh(size);
      mesh->createMeshFromOrganizedCloud(coords, rows, cols, rgb, mask);
      cout << "Surface: " << mesh->computeSurface() << endl;

      double** mCoords        = mesh->getCoords();
      unsigned char** mRGB    = mesh->getRGB();
      unsigned int** mIndices = mesh->getIndices();

      _points->Reset();
      _triangles->Reset();
      _colors->Reset();

      for(unsigned int i=0; i<mesh->getNumberOfPoints(); i++)
      {
        _points->InsertNextPoint(mCoords[i][0], mCoords[i][1], mCoords[i][2]);
        _colors->InsertNextTupleValue(mRGB[i]);
      }

      vtkSmartPointer<vtkTriangle> triangle  = vtkSmartPointer<vtkTriangle>::New();
      vtkIdList* pIds = triangle->GetPointIds();
      pIds->SetId(0, 0);
      pIds->SetId(1, 0);
      pIds->SetId(2, 0);
      vtkIdType* pId0 = pIds->GetPointer(0);
      vtkIdType* pId1 = pIds->GetPointer(1);
      vtkIdType* pId2 = pIds->GetPointer(2);
      for(unsigned int i=0; i<mesh->getNumberOfTriangles(); i++)
      {
        unsigned int* idx = mIndices[i];
        *pId0 = idx[0];
        *pId1 = idx[1];
        *pId2 = idx[2];
        _triangles->InsertNextCell(triangle);
      }

      _points->Modified();

      _renderWindow->Render();

      delete mesh;
    }
  }

private:

};

int main(int argc, char* argv[])
{
  if(argc!=2)
  {
    cout << "usage: " << argv[0] << " <config.xml>" << endl;
    return -1;
  }

  _kinect     = new Kinect(argv[1]);

  _polyData  = vtkSmartPointer<vtkPolyData>::New();
  _points    = vtkSmartPointer<vtkPoints>::New();
  _triangles = vtkSmartPointer<vtkCellArray>::New();
  _colors    = vtkSmartPointer<vtkUnsignedCharArray>::New();
  _colors->SetNumberOfComponents(3);
  _colors->SetName("Colors");

  _polyData->SetPoints(_points);
  _polyData->SetPolys(_triangles);
  _polyData->GetPointData()->SetScalars(_colors);

  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->SetBackground(.5, .5, .5);
  _renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  _renderWindow->SetSize(1024, 768);
  _renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInput(_polyData);
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  renderer->AddActor(actor);
  renderer->GetActiveCamera()->Yaw(180);
  renderer->ResetCamera();

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(_renderWindow);
  renderWindowInteractor->Initialize();

  vtkSmartPointer<vtkTimerCallback> cb =  vtkSmartPointer<vtkTimerCallback>::New();
  renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);
  renderWindowInteractor->CreateRepeatingTimer(30);

  _renderWindow->Render();
  renderWindowInteractor->Start();

  delete _kinect;

  return 0;
}
