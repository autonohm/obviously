#include "Icp.h"
#include "obcore/base/tools.h"
#include "obcore/base/Timer.h"
#include "obcore/math/mathbase.h"

namespace obvious
{

const char* g_icp_states[] = {"ICP_IDLE", "ICP_PROCESSING", "ICP_NOTMATCHABLE", "ICP_MAXITERATIONS", "ICP_TIMEELAPSED", "ICP_SUCCESS", "ICP_ERROR"};

Icp::Icp(PairAssignment* assigner, IRigidEstimator* estimator)
{
  _assigner  = assigner;
  _estimator = estimator;

  _maxIterations       = 3;
  _dim                 = _assigner->getDimension();
  _maxRMS              = 0.1;
  _model               = NULL;
  _scene               = NULL;
  _normalsM            = NULL;
  _normalsS            = NULL;
  _sizeModelBuf        = 0;
  _sizeSceneBuf        = 0;
  _sizeModel           = 0;
  _sizeScene           = 0;

  _Tfinal4x4           = new Matrix(4, 4);
  _Tfinal              = new Matrix(_dim+1, _dim+1);
  _Tlast               = new Matrix(4, 4);
  _Tfinal4x4->setIdentity();
  _Tlast->setIdentity();
  _reset = true;
  _convCnt = 5;

  this->reset();

  _trace = NULL;
}

Icp::~Icp()
{
  if(_model != NULL) System<double>::deallocate(_model);
  if(_scene != NULL) System<double>::deallocate(_scene);
  delete(_Tlast);
  delete(_Tfinal);
  if(_trace)
  {
    delete _trace;
    _trace = NULL;
  }
}

const char* Icp::state2char(EnumIcpState eState)
{
  return g_icp_states[eState];
};

void Icp::activateTrace()
{
  _trace = new IcpTrace(_dim);
}

void Icp::deactivateTrace()
{
  delete _trace;
  _trace = NULL;
}

PairAssignment* Icp::getPairAssigner()
{
  return _assigner;
}

IRigidEstimator* Icp::getRigidEstimator()
{
  return _estimator;
}

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

void Icp::setModel(double* coords, double* normals, const unsigned int size, double probability)
{
  _sizeModel = size;
  bool* mask = createSubsamplingMask(&_sizeModel, probability);

  unsigned int sizeNormalsBuf = _sizeModelBuf;
  checkMemory(_sizeModel, _dim, _sizeModelBuf, _model);
  unsigned int idx = 0;
  for(unsigned int i=0; i<size; i++)
  {
    if(mask[i])
    {
      for(unsigned int j=0; j<(unsigned int)_dim; j++)
        _model[idx][j] = coords[_dim*i+j];
      idx++;
    }
  }

  if(normals)
  {
    checkMemory(_sizeModel, _dim, sizeNormalsBuf, _normalsM);
    idx = 0;
    for(unsigned int i=0; i<size; i++)
    {
      if(mask[i])
      {
        for(unsigned int j=0; j<(unsigned int)_dim; j++)
          _normalsM[idx][j] = normals[_dim*i+j];
        idx++;
      }
    }
  }

  _assigner->setModel(_model, idx);
  _estimator->setModel(_model, idx, _normalsM);

  delete [] mask;
}

void Icp::setModel(Matrix* coords, Matrix* normals, double probability)
{
  if(coords->getCols()!=(size_t)_dim)
  {
    cout << "WARNING: Model is not of correct dimensionality. Needed: " << _dim << endl;
    return;
  }

  unsigned int sizeSource = coords->getRows();
  _sizeModel = sizeSource;
  bool* mask = createSubsamplingMask(&_sizeModel, probability);

  unsigned int sizeNormals = _sizeModelBuf;

  checkMemory(_sizeModel, _dim, _sizeModelBuf, _model);

  unsigned int idx = 0;
  for(unsigned int i=0; i<sizeSource; i++)
  {
    if(mask[i])
    {
      for(unsigned int j=0; j<(unsigned int)_dim; j++)
        _model[idx][j] = (*coords)(i,j);
      idx++;
    }
  }

  if(normals)
  {
    checkMemory(_sizeModel, _dim, sizeNormals, _normalsM);
    idx = 0;
    for(unsigned int i=0; i<sizeSource; i++)
    {
      if(mask[i])
      {
        for(unsigned int j=0; j<(unsigned int)_dim; j++)
          _normalsM[idx][j] = (*normals)(i,j);
        idx++;
      }
    }
  }

  _assigner->setModel(_model, idx);
  _estimator->setModel(_model, idx, _normalsM);

  delete [] mask;
}

void Icp::setScene(double* coords, double* normals, const unsigned int size, double probability)
{
  if(size==0)
  {
    cout << "Scene of size 0 passed ... ignoring" << endl;
    return;
  }

  _sizeScene = size;
  bool* mask = createSubsamplingMask(&_sizeScene, probability);

  unsigned int sizeNormalsBuf = _sizeSceneBuf;
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

  if(normals)
  {
    checkMemory(_sizeScene, _dim, sizeNormalsBuf, _normalsS);
    idx = 0;
    for(unsigned int i=0; i<size; i++)
    {
      if(mask[i])
      {
        for(unsigned int j=0; j<(unsigned int)_dim; j++)
          _normalsS[idx][j] = normals[_dim*i+j];
        idx++;
      }
    }
  }

  // if final matrix is not identity
  if(!_reset)
  {
    applyTransformation(_scene, idx, _dim, _Tfinal4x4);
    if(normals) applyTransformation(_normalsS, idx, _dim, _Tfinal4x4);
  }

  delete [] mask;
}

void Icp::setScene(Matrix* coords, Matrix* normals, double probability)
{
  if(coords->getCols()!=(size_t)_dim) {
    cout << "WARNING: Scene is not of correct dimensionality " << _dim << endl;
    return;
  }

  unsigned int sizeSource = coords->getRows();
  _sizeScene = sizeSource;
  bool* mask = createSubsamplingMask(&_sizeScene, probability);

  unsigned int sizeNormals = _sizeSceneBuf;

  checkMemory(_sizeScene, _dim, _sizeSceneBuf, _scene);
  unsigned int idx = 0;
  for(unsigned int i=0; i<sizeSource; i++)
  {
    if(mask[i])
    {
      for(unsigned int j=0; j<(unsigned int)_dim; j++)
        _scene[idx][j] = (*coords)(i,j);
      idx++;
    }
  }

  if(normals)
  {
    checkMemory(_sizeScene, _dim, sizeNormals, _normalsS);
    idx = 0;
    for(unsigned int i=0; i<sizeSource; i++)
    {
      if(mask[i])
      {
        for(unsigned int j=0; j<(unsigned int)_dim; j++)
          _normalsS[idx][j] = (*normals)(i,j);
        idx++;
      }
    }
  }

  // if final matrix is not identity
  if(!_reset)
  {
    applyTransformation(_scene, idx, _dim, _Tfinal4x4);
    if(normals) applyTransformation(_normalsS, idx, _dim, _Tfinal4x4);
  }

  delete [] mask;
}

void Icp::checkMemory(unsigned int rows, unsigned int cols, unsigned int &memsize, double** &mem)
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

void Icp::reset()
{
  _Tfinal4x4->setIdentity();
  _assigner->reset();
  _reset = true;
}

void Icp::setMaxRMS(double rms)
{
  _maxRMS = rms;
}

double Icp::getMaxRMS()
{
  return _maxRMS;
}

void Icp::setMaxIterations(unsigned int iterations)
{
  _maxIterations = iterations;
}

unsigned int Icp::getMaxIterations()
{
  return _maxIterations;
}

void Icp::setConvergenceCounter(unsigned int convCnt)
{
  _convCnt = convCnt;
}

unsigned int Icp::getConvergenceCounter()
{
  return _convCnt;
}

void Icp::applyTransformation(double** data, unsigned int size, unsigned int dim, Matrix* T)
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

EnumIcpState Icp::step(double* rms, unsigned int* pairs)
{
  Timer t;
  if(_model==NULL || _scene == NULL) return ICP_ERROR;

  _reset = false;

  vector<StrCartesianIndexPair>* pvPairs;
  _estimator->setScene(_scene, _sizeScene, _normalsS);
  _assigner->determinePairs(_scene, _sizeScene);
  pvPairs = _assigner->getPairs();
  *pairs = pvPairs->size();

  if(_trace) _trace->addAssignment(_model, _sizeModel, _scene, _sizeScene, *pvPairs);

  if(pvPairs->size()>2)
  {
    // Estimate transformation
    _estimator->setPairs(pvPairs);

    // get mapping error
    *rms = _estimator->getRMS();

    // estimate transformation
    _estimator->estimateTransformation(_Tlast);

    applyTransformation(_scene, _sizeScene, _dim, _Tlast);
    if(_normalsS)
      applyTransformation(_normalsS, _sizeScene, _dim, _Tlast);

    // update overall transformation
    (*_Tfinal4x4) = (*_Tlast) * (*_Tfinal4x4);
  }
  else
  {
    return ICP_NOTMATCHABLE;
  }

  return ICP_PROCESSING;
}

EnumIcpState Icp::iterate(double* rms, unsigned int* pairs, unsigned int* iterations)
{
  if(_trace) _trace->reset();

  EnumIcpState eRetval = ICP_PROCESSING;
  unsigned int iter = 0;
  double rms_prev = 10e12;
  unsigned int conv_cnt = 0;
  while( eRetval == ICP_PROCESSING )
  {
    eRetval = step(rms, pairs);
    iter++;

    if(fabs(*rms-rms_prev) < 10e-10)
      conv_cnt++;
    else
      conv_cnt = 0;
    if((*rms <= _maxRMS || conv_cnt>=_convCnt ))
      eRetval = ICP_SUCCESS;
    else if(iter >= _maxIterations)
      eRetval = ICP_MAXITERATIONS;

    rms_prev = *rms;
  }
  *iterations = iter;

  return eRetval;
}	

void Icp::serializeTrace(char* folder)
{
  if(_trace)
  {
    _trace->serialize(folder);
  }
}

Matrix* Icp::getFinalTransformation4x4()
{
  return _Tfinal4x4;
}

Matrix* Icp::getFinalTransformation()
{
   for(int r=0; r<_dim; r++)
   {
      for(int c=0; c<_dim; c++)
      {
         (*_Tfinal)(r,c) = (*_Tfinal4x4)(r,c);
      }
      (*_Tfinal)(r,_dim) = (*_Tfinal4x4)(r,3);
   }

   for(int c=0; c<_dim; c++)
      (*_Tfinal)(_dim,c) = 0;

   (*_Tfinal)(_dim,_dim) = 1;

  return _Tfinal;
}

Matrix* Icp::getLastTransformation()
{
  return _Tlast;
}

}
