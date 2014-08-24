#include "Ndt.h"

#include "obcore/base/tools.h"
#include "obcore/base/Timer.h"
#include "obcore/math/mathbase.h"

namespace obvious
{

const char* g_ndt_states[] = {"NDT_IDLE", "NDT_PROCESSING", "NDT_NOTMATCHABLE", "NDT_MAXITERATIONS", "NDT_TIMEELAPSED", "NDT_SUCCESS", "NDT_ERROR"};

Ndt::Ndt(int minX, int maxX, int minY, int maxY)
{

  _minX = minX;
  _maxX = maxX;
  _minY = minY;
  _maxY = maxY;

  System<NdtCell>::allocate(_maxY-_minY, _maxX-_minX, _model);
  for(int y=0; y<_maxY-_minY; y++)
  {
    for(int x=0; x<_maxX-_minX; x++)
    {
      _model[y][x].centroid = new double[2];
      _model[y][x].cov = new Matrix(2, 2);
    }
  }

  _maxIterations       = 3;
  _dim                 = 2;
  _scene               = NULL;
  _sceneTmp            = NULL;
  _sizeSceneBuf        = 0;
  _sizeScene           = 0;

  _Tfinal4x4           = new Matrix(4, 4);
  _Tlast               = new Matrix(4, 4);
  _Tfinal4x4->setIdentity();
  _Tlast->setIdentity();

  this->reset();
}

Ndt::~Ndt()
{
  System<NdtCell>::deallocate(_model);
  if(_scene != NULL)     System<double>::deallocate(_scene);
  if(_sceneTmp != NULL)  System<double>::deallocate(_sceneTmp);
  delete _Tlast;
}

const char* Ndt::state2char(EnumNdtState eState)
{
  return g_ndt_states[eState];
};

bool* createSubsamplingMask(unsigned int* size, double probability)
{
  unsigned int sizeOut = 0;
  bool* mask = new bool[*size];
  memset(mask, 0, *size * sizeof(*mask));
  if(probability>1.0) probability = 1.0;
  if(probability<0.0) probability = 0.0;
  int probability_thresh = (int)(probability * 1000.0 + 0.5);
  srand(time(NULL));
  for(unsigned int i=0; i<*size; i++)
  {
    if((rand()%1000)<probability_thresh)
    {
      mask[i] = 1;
      sizeOut++;
    }
  }
  *size = sizeOut;
  return mask;
}

void Ndt::setModel(Matrix* coords, double probability)
{
  if(coords->getCols()!=(size_t)_dim)
  {
    cout << "WARNING: Model is not of correct dimensionality. Needed: " << _dim << endl;
    return;
  }

  unsigned int sizeSource = coords->getRows();
  unsigned int sizeModel = sizeSource;
  bool* mask = createSubsamplingMask(&sizeModel, probability);

  for(unsigned int i=0; i<sizeSource; i++)
  {
    if(mask[i])
    {
      double* coord = new double[2];
      coord[0] = (*coords)(i, 0);
      coord[1] = (*coords)(i, 1);
      int x = floor(coord[0]) - _minX;
      int y = floor(coord[1]) - _minY;
      _model[y][x].coords.push_back(coord);
    }
  }

  for(int y=0; y<_maxY-_minY; y++)
  {
    for(int x=0; x<_maxX-_minX; x++)
    {
      NdtCell cell = _model[y][x];
      vector<double*> v = cell.coords;
      double sumX = 0.0;
      double sumY = 0.0;
      for(int i=0; i<v.size(); i++)
      {
        cout << x << " " << y << " " << v[i][0] << " " << v[i][1] << endl;
        sumX += v[i][0];
        sumY += v[i][1];
      }
      cell.centroid[0] = sumX / (double)v.size();
      cell.centroid[1] = sumY / (double)v.size();
      cout << "centroid: " << cell.centroid[0] << " " << cell.centroid[1] << endl << endl;;

      Matrix* cov = cell.cov;
      (*cov)(0,0) = 0.0;
      (*cov)(0,1) = 0.0;
      (*cov)(1,0) = 0.0;
      (*cov)(1,1) = 0.0;
      for(int i=0; i<v.size(); i++)
      {
        double c[2];
        c[0] = v[i][0] - cell.centroid[0];
        c[1] = v[i][1] - cell.centroid[1];
        (*cov)(0,0) += c[0] * c[0];
        (*cov)(0,1) += c[0] * c[1];
        (*cov)(1,0) += c[1] * c[0];
        (*cov)(1,1) += c[1] * c[1];
      }
      (*cov)(0,0) /= v.size();
      (*cov)(0,1) /= v.size();
      (*cov)(1,0) /= v.size();
      (*cov)(1,1) /= v.size();
      cov->print();
    }
  }

  delete [] mask;
}

void Ndt::setScene(double* coords, const unsigned int size, double probability)
{
  if(size==0)
  {
    cout << "Scene of size 0 passed ... ignoring" << endl;
    return;
  }

  _sizeScene = size;
  bool* mask = createSubsamplingMask(&_sizeScene, probability);

  checkMemory(_sizeScene, _dim, _sizeSceneBuf, _scene);
  unsigned int idx = 0;
  for(unsigned int i=0; i<size; i++)
  {
    if(mask[i])
    {
      for(unsigned int j=0; j<(unsigned int)_dim; j++)
        _scene[idx][j] = coords[_dim*i+j];
      idx++;
    }
  }
  if(_sceneTmp) System<double>::deallocate(_sceneTmp);
  System<double>::allocate(_sizeScene, _dim, _sceneTmp);
  System<double>::copy(_sizeScene, _dim, _scene, _sceneTmp);

  delete [] mask;
}

void Ndt::reset()
{
  _Tfinal4x4->setIdentity();
  if(_sceneTmp) System<double>::copy(_sizeScene, _dim, _scene, _sceneTmp);
}

void Ndt::setMaxIterations(unsigned int iterations)
{
  _maxIterations = iterations;
}

unsigned int Ndt::getMaxIterations()
{
  return _maxIterations;
}

void Ndt::applyTransformation(double** data, unsigned int size, unsigned int dim, Matrix* T)
{
  // Apply rotation
  Matrix R(*T, 0, 0, dim, dim);
  Matrix::multiply(R, *data, size, dim);

  // Apply translation
  if(_dim < 3)
  {

#pragma omp parallel
{
#pragma omp for
  for(unsigned int i=0; i<size; i++)
  {
    data[i][0] += (*T)(0,3);
    data[i][1] += (*T)(1,3);
  }
}

  }
  else
  {

#pragma omp parallel
{
#pragma omp for
  for(unsigned int i=0; i<size; i++)
  {
    data[i][0] += (*T)(0,3);
    data[i][1] += (*T)(1,3);
    data[i][2] += (*T)(2,3);
  }
}

  } // end if

}

EnumNdtState Ndt::iterate(double* rms, unsigned int* iterations, Matrix* Tinit)
{
  _Tfinal4x4->setIdentity();

  if(Tinit)
  {
    applyTransformation(_sceneTmp, _sizeScene, _dim, Tinit);
    (*_Tfinal4x4) = (*Tinit) * (*_Tfinal4x4);
  }

  EnumNdtState eRetval = NDT_PROCESSING;
  unsigned int iter = 0;
  //double rms_prev = 10e12;

  *iterations = iter;

  return eRetval;
}	

Matrix Ndt::getFinalTransformation4x4()
{
  Matrix T = *_Tfinal4x4;
  return T;
}

Matrix Ndt::getFinalTransformation()
{
  Matrix T(_dim+1, _dim+1);
   for(int r=0; r<_dim; r++)
   {
      for(int c=0; c<_dim; c++)
      {
         T(r,c) = (*_Tfinal4x4)(r,c);
      }
      T(r,_dim) = (*_Tfinal4x4)(r,3);
   }

   for(int c=0; c<_dim; c++)
      T(_dim,c) = 0;

   T(_dim,_dim) = 1;

  return T;
}

Matrix Ndt::getLastTransformation()
{
  Matrix T = *_Tlast;
  return T;
}

void Ndt::checkMemory(unsigned int rows, unsigned int cols, unsigned int &memsize, double** &mem)
{
  // first instantiation of buffer
  if(mem == NULL)
  {
    memsize = rows;
    System<double>::allocate(rows, cols, mem);
  }
  // resize buffer, if needed
  if(rows > memsize)
  {
    memsize = rows;
    if(mem != NULL) System<double>::deallocate(mem);
    System<double>::allocate(rows, cols, mem);
  }
}
}
