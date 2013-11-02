#include "VtkCloud.h"
#include "ohm_logo.h"

#include <vtkVertexGlyphFilter.h>
#include <vtkPointData.h>
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkLookupTable.h>
#include <vtkIVWriter.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkPLYWriter.h>
//#include <vtkMNIObjectWriter.h>
#include <vtkPLYReader.h>
#include <vtkCleanPolyData.h>
#include <vtkIdList.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <vtkPointSource.h>

#include "obcore/math/mathbase.h"
#include "obcore/base/Logger.h"

#include <string>

using namespace std;

namespace obvious
{

VtkCloud::VtkCloud()
{
  _polyData = vtkSmartPointer<vtkPolyData>::New();

  _points   = vtkSmartPointer<vtkPoints>::New();

  _normals = vtkSmartPointer<vtkDoubleArray>::New();
  _normals->SetNumberOfComponents(3);

  _colors  = vtkSmartPointer<vtkUnsignedCharArray>::New();
  _colors->SetNumberOfComponents(3);
  _colors->SetName ("Colors");

  _triangles = vtkSmartPointer<vtkCellArray>::New();

  _actor = NULL;
}

VtkCloud::~VtkCloud()
{

}

void VtkCloud::setCoords(double* coords, int size, int tda, double* ndata)
{
  _polyData->Reset();
  _points->Reset();

  for(int i=0; i<size*tda; i+=tda)
  {
    if(coords != NULL)
      _points->InsertNextPoint(coords[i], coords[i + 1], coords[i + 2]);
    else
      _points->InsertNextPoint(0, 0, 0);
  }

  _normals->Reset();
  if(ndata)
  {
    _normals->SetNumberOfTuples(size);
    for(int i=0; i<size; i++)
      _normals->SetTuple((vtkIdType)i, &ndata[tda*i]);
    _polyData->GetPointData()->SetNormals(_normals);
  }
  else
  {
    _polyData->GetPointData()->SetNormals(NULL);
  }

  _polyData->SetPoints(_points);

  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(_polyData->GetProducerPort());
  glyphFilter->Update();

  _polyData->ShallowCopy(glyphFilter->GetOutput());
}

void VtkCloud::setNormals(double* ndata, int size, int tda)
{
  _normals->Reset();

  _normals->SetNumberOfTuples(size);
  for(int i=0; i<size; i++)
    _normals->SetTuple((vtkIdType)i, &ndata[tda*i]);

  _polyData->GetPointData()->SetNormals(_normals);

  _points->Modified();

  /*vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(_polyData->GetProducerPort());
  glyphFilter->Update();

  _polyData->ShallowCopy(glyphFilter->GetOutput());*/
}

void VtkCloud::setTriangles(double** coords, unsigned char** rgb, unsigned int points, unsigned int** indices, unsigned int triangles)
{
  _polyData->Reset();

  _points->Reset();
  _triangles->Reset();
  _colors->Reset();

  for(unsigned int i=0; i<points; i++)
  {
    _points->InsertNextPoint(coords[i][0], coords[i][1], coords[i][2]);
    _colors->InsertNextTupleValue(rgb[i]);
  }

  vtkSmartPointer<vtkTriangle> triangle  = vtkSmartPointer<vtkTriangle>::New();
  vtkIdList* pIds = triangle->GetPointIds();
  pIds->SetId(0, 0);
  pIds->SetId(1, 0);
  pIds->SetId(2, 0);
  vtkIdType* pId0 = pIds->GetPointer(0);
  vtkIdType* pId1 = pIds->GetPointer(1);
  vtkIdType* pId2 = pIds->GetPointer(2);
  for(unsigned int i=0; i<triangles; i++)
  {
    unsigned int* idx = indices[i];
    *pId0 = idx[0];
    *pId1 = idx[1];
    *pId2 = idx[2];
    _triangles->InsertNextCell(triangle);
  }

  _polyData->SetPoints(_points);
  _polyData->GetPointData()->SetScalars(_colors);
  _polyData->SetPolys(_triangles);
  _points->Modified();

 /* vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(_polyData->GetProducerPort());
  glyphFilter->Update();

  _polyData->ShallowCopy(glyphFilter->GetOutput());*/
}

void VtkCloud::addCoords(double* coords, unsigned char* rgb, int size, int tda)
{
  _polyData->Reset();

  vtkSmartPointer<vtkPoints> points       = _polyData->GetPoints();
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkUnsignedCharArray::SafeDownCast(_polyData->GetPointData()->GetScalars());

  for(int i=0; i<size*tda; i+=tda)
  {
    points->InsertNextPoint(coords[i], coords[i + 1], coords[i + 2]);
    colors->InsertNextTupleValue(&rgb[i]);
  }

  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(_polyData->GetProducerPort());
  glyphFilter->Update();

  _polyData->ShallowCopy(glyphFilter->GetOutput());
}

void VtkCloud::setColors(const unsigned char* data, int points, int channels)
{
  _colors->Reset();
  _colors->SetNumberOfComponents(channels);

  for(int i=0; i<points*channels; i+=channels)
  {
    _colors->InsertNextTupleValue(&data[i]);
  }

  _polyData->GetPointData()->SetScalars(_colors);
  _points->Modified();

  /*vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(_polyData->GetProducerPort());
  glyphFilter->Update();

  _polyData->ShallowCopy(glyphFilter->GetOutput());*/
}

void VtkCloud::removeInvalidPoints()
{
  vtkSmartPointer<vtkPoints>            newPoints  = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkUnsignedCharArray> newColors  = vtkSmartPointer<vtkUnsignedCharArray>::New();
  vtkSmartPointer<vtkDoubleArray>       newNormals = vtkSmartPointer<vtkDoubleArray>::New();

  vtkSmartPointer<vtkPoints> points                = _polyData->GetPoints();
  vtkSmartPointer<vtkUnsignedCharArray> colors     = vtkUnsignedCharArray::SafeDownCast(_polyData->GetPointData()->GetScalars());
  vtkSmartPointer<vtkDoubleArray>       normals    = vtkDoubleArray::SafeDownCast(_polyData->GetPointData()->GetNormals());

  if(normals==NULL)
  {
    LOGMSG(DBG_WARN, "Cloud has no normals");
    return;
  }

  unsigned int size = points->GetNumberOfPoints();
  newColors->SetNumberOfComponents(3);
  newNormals->SetNumberOfComponents(3);

  double buf[3];
  double nbuf[3];
  unsigned char cbuf[3];

  for(unsigned int i=0; i<size; i++)
  {
    points->GetPoint(i, buf);
    normals->GetTupleValue(i, nbuf);
    double len = sqrt(nbuf[0]*nbuf[0] + nbuf[1]*nbuf[1] + nbuf[2]*nbuf[2]);
    if(buf[2]>10e-6 && len>10e-6)
    {
      newPoints->InsertNextPoint(buf);
      if(colors)
      {
        colors->GetTupleValue(i, cbuf);
        newColors->InsertNextTupleValue(cbuf);
      }
      norm3(nbuf);
      newNormals->InsertNextTuple(nbuf);
    }
  }

  points->ShallowCopy(newPoints);
  if(colors) colors->DeepCopy(newColors);
  normals->DeepCopy(newNormals);

  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(_polyData->GetProducerPort());
  glyphFilter->Update();

  _polyData->ShallowCopy(glyphFilter->GetOutput());
}

unsigned int VtkCloud::getSize()
{
  vtkSmartPointer<vtkPointSet> pointsScene  = vtkPointSet::SafeDownCast(_polyData);
  return pointsScene->GetNumberOfPoints();
}

void VtkCloud::copyCoords(double* dst, unsigned int subsampling)
{
  vtkSmartPointer<vtkPointSet> points  = vtkPointSet::SafeDownCast(_polyData);
  int j=0;
  for(int i=0; i<points->GetNumberOfPoints(); i+=subsampling, j++)
      points->GetPoint(i, &dst[j*3]);
}

void VtkCloud::copyColors(unsigned char* dst, unsigned int subsampling)
{
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkUnsignedCharArray::SafeDownCast(_polyData->GetPointData()->GetScalars());
  int j=0;
  for(int i=0; i<colors->GetNumberOfTuples(); i+=subsampling, j++)
    colors->GetTupleValue(i, &dst[3*j]);
}

void VtkCloud::copyNormals(double* dst, unsigned int subsampling)
{
  vtkSmartPointer<vtkDoubleArray> normals  = vtkDoubleArray::SafeDownCast(_polyData->GetPointData()->GetNormals());

  if(normals==NULL)
  {
    LOGMSG(DBG_WARN, "Cloud has no normals");
    return;
  }

  int j=0;
  for(int i=0; i<normals->GetNumberOfTuples(); i+=subsampling, j++)
      normals->GetTuple(i, &dst[j*3]);
}

void VtkCloud::copyData(gsl_matrix* C, gsl_matrix* N, unsigned char* rgb)
{
  vtkSmartPointer<vtkPointSet> points          = vtkPointSet::SafeDownCast(_polyData);
  vtkSmartPointer<vtkDoubleArray> normals      = vtkDoubleArray::SafeDownCast(_polyData->GetPointData()->GetNormals());
  vtkSmartPointer<vtkUnsignedCharArray> colors = vtkUnsignedCharArray::SafeDownCast(_polyData->GetPointData()->GetScalars());
  for(int i=0; i<points->GetNumberOfPoints(); i++)
  {
    double* dst = gsl_matrix_ptr(C, i, 0);
    points->GetPoint(i, dst);
    if(colors) colors->GetTupleValue(i, &rgb[3*i]);
  }

  if(normals)
  {
    for(int i=0; i<points->GetNumberOfPoints(); i++)
    {
      double* dst = gsl_matrix_ptr(N, i, 0);
      normals->GetTuple(i, dst);
    }
  }
  else
  {
    LOGMSG(DBG_WARN, "Cloud has no normals");
  }
}

void VtkCloud::transform(double* T)
{
  if(!_actor) return;
  vtkSmartPointer<vtkTransformFilter> transf = vtkSmartPointer<vtkTransformFilter>::New();
  vtkSmartPointer<vtkTransform> trans    = vtkSmartPointer<vtkTransform>::New();
  trans->SetMatrix(T);
  transf->SetTransform(trans);
  transf->SetInputConnection(_polyData->GetProducerPort());
  transf->Update();
  _polyData->DeepCopy(transf->GetOutput());
  _polyData->Modified();
  _actor->SetPosition(0, 0, 0);
  _actor->GetMatrix()->Identity();
}

void VtkCloud::lowlight(std::vector<unsigned int>* indices)
{
  unsigned int size = _polyData->GetPoints()->GetNumberOfPoints();
  vtkSmartPointer<vtkLookupTable> lut = vtkSmartPointer<vtkLookupTable>::New();
  lut->SetTableRange(0, 1);
  lut->Build();
  unsigned char* colortbl = new unsigned char[3*size];
  double c[3];
  for(unsigned int i=0; i<size; i++)
  {
    lut->GetColor(0, c);
    colortbl[3*i]   = 255.0*c[0];
    colortbl[3*i+1] = 255.0*c[1];
    colortbl[3*i+2] = 255.0*c[2];
  }
  for(unsigned int i=0; i<indices->size(); i++)
  {
    unsigned int idx = (*indices)[i];
    lut->GetColor(1, c);
    colortbl[3*idx]   = 255.0*c[0];
    colortbl[3*idx+1] = 255.0*c[1];
    colortbl[3*idx+2] = 255.0*c[2];
  }

  setColors(colortbl, size, 3);
  delete [] colortbl;
}

vtkSmartPointer<vtkPolyData> VtkCloud::getPolyData()
{
  return _polyData;
}

void VtkCloud::serialize(char* filename, EnumVtkCloudFileFormat format)
{
  switch(format)
  {
  case VTKCloud_XML:
  {
    vtkSmartPointer<vtkXMLPolyDataWriter> w = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
    w->SetFileName(filename);
    w->SetInput(_polyData);
    w->Write();
    break;
  }
  case VTKCloud_PLY:
  {
    vtkSmartPointer<vtkPLYWriter> w = vtkSmartPointer<vtkPLYWriter>::New();
    w->SetColorModeToDefault();
    w->SetArrayName("Colors");
    w->SetFileName(filename);
    w->SetInput(_polyData);
    w->Write();
    break;
  }
  /*case VTKCloud_MNI:
  {
    vtkSmartPointer<vtkMNIObjectWriter> w = vtkSmartPointer<vtkMNIObjectWriter>::New();
    w->SetFileName(filename);
    w->SetInput(_polyData);
    w->Write();
    break;
  }*/
  default:
  {
    LOGMSG(DBG_WARN, "Format not specified properly");
    return;
  }
  }
}

VtkCloud* VtkCloud::load(char* filename, EnumVtkCloudFileFormat format)
{
  if(format == VTKCloud_AUTO)
  {
    string fn = filename;
    if(fn.substr(fn.find_last_of(".") + 1) == "ply")
    {
      format = VTKCloud_PLY;
    }
    else if(fn.substr(fn.find_last_of(".") + 1) == "vtp")
    {
      format = VTKCloud_XML;
    }
    else
    {
      LOGMSG(DBG_WARN, "file type could not be determined.");
      return NULL;
    }
  }
  vtkSmartPointer<vtkPolyData> polyData = NULL;
  switch(format)
  {
  case VTKCloud_XML:
  {
    vtkSmartPointer<vtkXMLPolyDataReader> r = vtkSmartPointer<vtkXMLPolyDataReader>::New();
    r->SetFileName(filename);
    r->Update();
    polyData = r->GetOutput();
    break;
  }
  case VTKCloud_PLY:
  {
    vtkSmartPointer<vtkPLYReader> r = vtkSmartPointer<vtkPLYReader>::New();
    r->SetFileName(filename);
    r->Update();
    polyData = r->GetOutput();
    break;
  }
  default:
  {

  }
  }
  VtkCloud* cloud = new VtkCloud();//NULL, polyData->GetPoints()->GetNumberOfPoints(), 3);

  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(polyData->GetProducerPort());
  glyphFilter->Update();

  cloud->getPolyData()->ShallowCopy(glyphFilter->GetOutput());
  return cloud;
}

void VtkCloud::setActor(vtkSmartPointer<vtkActor> actor)
{
  _actor = actor;
}

vtkSmartPointer<vtkActor> VtkCloud::getActor()
{
  return _actor;
}

void VtkCloud::getTransformation(double T[16])
{
  if(!_actor) return;
  _actor->GetMatrix(T);
}

void VtkCloud::getTransformation(double R[9], double t[3])
{
  if(!_actor) return;
  double T[16];
  _actor->GetMatrix(T);
  R[0] = T[0]; R[1] = T[1]; R[2] = T[2];  t[0] = T[3];
  R[3] = T[4]; R[4] = T[5]; R[5] = T[6];  t[1] = T[7];
  R[6] = T[8]; R[7] = T[9]; R[8] = T[10]; t[2] = T[11];
}

void VtkCloud::applyActorTransformation()
{
  if(!_actor) return;
  vtkSmartPointer<vtkTransformFilter> tf = vtkSmartPointer<vtkTransformFilter>::New();
  vtkSmartPointer<vtkTransform> t        = vtkSmartPointer<vtkTransform>::New();
  t->SetMatrix(_actor->GetMatrix());
  tf->SetTransform(t);
  tf->SetInputConnection(_polyData->GetProducerPort());
  tf->Update();
  _polyData->DeepCopy(tf->GetOutput());
  _polyData->Modified();
  _actor->SetPosition(0, 0, 0);
  _actor->GetMatrix()->Identity();
}

void VtkCloud::resetActorTransformation()
{
  if(!_actor) return;
  vtkMatrix4x4* T = _actor->GetMatrix();
  T->Identity();
}

VtkCloud* VtkCloud::createExample()
{
  VtkCloud* cloud;

  int rows = ohm_logo.height;
  int cols  = ohm_logo.width;
  const unsigned char* pixel_data = ohm_logo.pixel_data;

  double* data          = new double[rows * cols * 3];
  unsigned char* colors = new unsigned char[rows * cols * 3];

  int cnt = 0;
  for(int r=0; r<rows; r++)
  {
    for(int c=0; c<cols; c++)
    {
      int idx = (r*cols+c)*4;
      if((int)pixel_data[idx+3]>0)
      {
        data[3*cnt]       = c;
        data[3*cnt + 1]   = r;
        data[3*cnt + 2]   = 0;

        colors[3*cnt]     = pixel_data[idx];
        colors[3*cnt+1]   = pixel_data[idx+1];
        colors[3*cnt+2]   = pixel_data[idx+2];
        cnt++;
      }
    }
  }

  cloud = new VtkCloud();
  cloud->setCoords(data, cnt, 3);
  cloud->setColors(colors, cnt, 3);

  delete [] data;
  delete [] colors;

  return cloud;
}

VtkCloud* VtkCloud::createRandom(unsigned int nrPoints, double radius)
{
  VtkCloud* cloud = new VtkCloud();

  // Create a point cloud
  vtkSmartPointer<vtkPointSource> pointSource =
    vtkSmartPointer<vtkPointSource>::New();
  pointSource->SetCenter(0.0, 0.0, 0.0);
  pointSource->SetNumberOfPoints(nrPoints);
  pointSource->SetRadius(radius);
  pointSource->Update();

  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =  vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputConnection(pointSource->GetOutput()->GetProducerPort());
  glyphFilter->Update();

  cloud->getPolyData()->ShallowCopy(glyphFilter->GetOutput());

  return cloud;
}

}

