#include "CartesianCloud.h"

#include <string.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_vector.h>
#include <obcore/math/mathbase.h>

namespace obvious
{
CartesianCloud3D::CartesianCloud3D(unsigned int size, double* coords, unsigned char* rgb, double* normals)
{
  _coords  = NULL;
  _colors  = NULL;
  _normals = NULL;
  _size    = size;
  bool withInfo = (rgb!=NULL);
  init(size, withInfo);

  gsl_matrix_view vcoords = gsl_matrix_view_array(coords, size, 3);
  gsl_matrix_memcpy(_coords, &vcoords.matrix);

  if(normals)
  {
    gsl_matrix_view vnormals = gsl_matrix_view_array(normals, size, 3);
    gsl_matrix_memcpy(_normals, &vnormals.matrix);
    _hasNormals = 1;
  }

  if(rgb)
  {
    memcpy(_colors, rgb, 3*size*sizeof(*_colors));
    _hasColors = true;
  }

}

CartesianCloud3D::CartesianCloud3D(unsigned int size, bool withInfo)
{
  init(size, withInfo);
}

CartesianCloud3D::CartesianCloud3D(CartesianCloud3D* cloud)
{
  init(cloud->size(), cloud->hasInfo());
  gsl_matrix_memcpy(_coords, cloud->_coords);
  gsl_matrix_memcpy(_normals, cloud->_normals);
  if(cloud->hasInfo())
  {
    memcpy(_colors, cloud->_colors, cloud->size()*3*sizeof(*_colors));
    memcpy(_attributes, cloud->_attributes, cloud->size()*sizeof(*_attributes));
    memcpy(_indices, cloud->_indices, cloud->size()*sizeof(*_indices));
  }
}

void CartesianCloud3D::init(unsigned int size, bool withInfo)
{
  _hasInfo    = 0;
  _hasNormals = 0;
  _coords   = gsl_matrix_alloc(size, 3);
  _normals  = gsl_matrix_alloc(size, 3);

  if (withInfo)
  {
    _hasInfo   = 1;
    _colors     = new unsigned char[size*3];
    _attributes = new int[size];
    _indices    = new int[size];

    for (unsigned int i = 0; i < size; i++)
    {
      _colors[i*3]   = 255;
      _colors[i*3+1] = 255;
      _colors[i*3+2] = 255;

      _attributes[i] = ePointAttrValid;

      _indices[i]    = i;
    }
  }
}

CartesianCloud3D::~CartesianCloud3D()
{
  _mSourceInfo.clear();

  if(_hasInfo)
  {
    delete[] _colors;
    delete[] _attributes;
    delete[] _indices;
  }

  gsl_matrix_free(_coords);
  gsl_matrix_free(_normals);
}

double* CartesianCloud3D::operator [](unsigned int i)
{
  return gsl_matrix_ptr(_coords, i, 0);
}

gsl_matrix* CartesianCloud3D::getCoords()
{
  return _coords;
}

void CartesianCloud3D::setNormals(gsl_matrix* normals)
{
  gsl_matrix_memcpy(_normals, normals);
}

gsl_matrix* CartesianCloud3D::getNormals()
{
  return _normals;
}

unsigned char* CartesianCloud3D::getColors()
{
  return _colors;
}

int* CartesianCloud3D::getAttributes()
{
  return _attributes;
}

int* CartesianCloud3D::getIndices()
{
  return _indices;
}

int CartesianCloud3D::hasInfo()
{
  return _hasInfo;
}

int CartesianCloud3D::hasSourceInfo()
{
  return (!_mSourceInfo.empty());
}

void CartesianCloud3D::addSourceInfo(EnumSourceInfo eSourceInfo, long lValue)
{
  _mSourceInfo[eSourceInfo] = lValue;
}

int CartesianCloud3D::getSourceInfo(EnumSourceInfo eSourceInfo, long* plValue)
{
  int nRetval = 0;
  if (_mSourceInfo.find(eSourceInfo) != _mSourceInfo.end())
  {
    nRetval = 1;
    *plValue = _mSourceInfo[eSourceInfo];
  }
  return nRetval;
}

void CartesianCloud3D::clearSourceInfo()
{
  _mSourceInfo.clear();
}

void CartesianCloud3D::maskPoints(bool* mask)
{
  for (unsigned int i=0; i<_coords->size1; i++)
  {
    if(!mask[i])_attributes[i] &= ~ePointAttrValid;
  }
}

void CartesianCloud3D::maskEmptyNormals()
{
  for (unsigned int i=0; i<_coords->size1; i++)
  {
    double bufN[3];
    bufN[0] = gsl_matrix_get(_normals, i, 0);
    bufN[1] = gsl_matrix_get(_normals, i, 1);
    bufN[2] = gsl_matrix_get(_normals, i, 2);
    double len = bufN[0]*bufN[0] + bufN[1]*bufN[1] + bufN[2]*bufN[2];
    if(len<10e-6)_attributes[i] &= ~ePointAttrValid;
  }
}

void CartesianCloud3D::removeInvalidPoints()
{

  if(!_hasInfo) return;

  gsl_matrix* bufC = gsl_matrix_alloc(_coords->size1, 3);
  gsl_matrix* bufN = gsl_matrix_alloc(_coords->size1, 3);

  unsigned int i;
  int cnt = 0;

  for (i=0; i<_coords->size1; i++)
  {
    if(_attributes[i] & ePointAttrValid)
    {
      gsl_matrix_set(bufC, cnt, 0, gsl_matrix_get(_coords, i, 0));
      gsl_matrix_set(bufC, cnt, 1, gsl_matrix_get(_coords, i, 1));
      gsl_matrix_set(bufC, cnt, 2, gsl_matrix_get(_coords, i, 2));
      gsl_matrix_set(bufN, cnt, 0, gsl_matrix_get(_normals, i, 0));
      gsl_matrix_set(bufN, cnt, 1, gsl_matrix_get(_normals, i, 1));
      gsl_matrix_set(bufN, cnt, 2, gsl_matrix_get(_normals, i, 2));
      _colors[3*cnt]   = _colors[3*i];
      _colors[3*cnt+1] = _colors[3*i+1];
      _colors[3*cnt+2] = _colors[3*i+2];
      _indices[cnt]    = _indices[i];
      _attributes[cnt] = _attributes[i];
      cnt++;
    }
  }

  gsl_matrix_free(_coords);
  gsl_matrix_free(_normals);
  _coords  = gsl_matrix_alloc(cnt, 3);
  _normals = gsl_matrix_alloc(cnt, 3);
  gsl_matrix_view vbuf = gsl_matrix_submatrix (bufC, 0, 0, cnt, 3);
  gsl_matrix_memcpy(_coords, &vbuf.matrix);
  vbuf = gsl_matrix_submatrix (bufN, 0, 0, cnt, 3);
  gsl_matrix_memcpy(_normals, &vbuf.matrix);

  gsl_matrix_free(bufC);
  gsl_matrix_free(bufN);
}

void CartesianCloud3D::subsample(unsigned int step)
{
  for (unsigned int i=0; i<_coords->size1; i++)
  {
    if(i%step!=0)
      _attributes[i] &= ~ePointAttrValid;
  }
  removeInvalidPoints();
}

unsigned int CartesianCloud3D::size()
{
  return _coords->size1;
}

void CartesianCloud3D::transform(gsl_matrix* T)
{
  gsl_matrix* buf = gsl_matrix_alloc(_coords->size1, 3);

  gsl_matrix_view R = gsl_matrix_submatrix(T, 0, 0, 3, 3);

  gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, _coords, &R.matrix, 0.0, buf);
  gsl_matrix_memcpy(_coords, buf);

  gsl_vector_view x = gsl_matrix_column(_coords, 0);
  gsl_vector_view y = gsl_matrix_column(_coords, 1);
  gsl_vector_view z = gsl_matrix_column(_coords, 2);

  gsl_vector_add_constant(&x.vector, gsl_matrix_get(T,0,3));
  gsl_vector_add_constant(&y.vector, gsl_matrix_get(T,1,3));
  gsl_vector_add_constant(&z.vector, gsl_matrix_get(T,2,3));

  gsl_matrix_free(buf);
}

void CartesianCloud3D::transform(Matrix* T)
{
  transform(T->getBuffer());
}

void CartesianCloud3D::transform(double T[16])
{
  gsl_matrix_view VT = gsl_matrix_view_array(T, 4, 4);
  transform(&VT.matrix);
}

void CartesianCloud3D::createProjection(unsigned char* pImage, unsigned char* pMask, gsl_matrix* P, int nW, int nH)
{
  gsl_vector* xi = gsl_vector_alloc(4);
  gsl_vector* ni = gsl_vector_alloc(3);
  gsl_vector_set(xi, 3, 1.0);

  memset(pImage, 0, 3*nW*nH*sizeof(unsigned char));
  memset(pMask, 0, nW*nH*sizeof(unsigned char));

  for(unsigned int i=0; i<_coords->size1; i++)
  {
    double* point = gsl_matrix_ptr(_coords, i, 0);

    if(_attributes[i] & ePointAttrValid)
    {
      gsl_vector_set(xi, 0, point[0]);
      gsl_vector_set(xi, 1, point[1]);
      gsl_vector_set(xi, 2, point[2]);

      gsl_blas_dgemv(CblasNoTrans, 1.0, P, xi, 0.0, ni);
      double du = gsl_vector_get(ni,0);
      double dv = gsl_vector_get(ni,1);
      double dw = gsl_vector_get(ni,2);
      if(dw > 10e-6)
      {
        int u = nW-1-(int)( du / dw + 0.5);
        int v = (int)(dv / dw + 0.5);

        if((u>=0) && (u<nW) && (v>=0) && (v<nH))
        {
          pImage[(v*nW+u)*3]   = _colors[3*i];
          pImage[(v*nW+u)*3+1] = _colors[3*i+1];
          pImage[(v*nW+u)*3+2] = _colors[3*i+2];;
          pMask[v*nW+u]        = 255;
        }
      }
    }
  }

  gsl_vector_free(ni);
  gsl_vector_free(xi);
}

void CartesianCloud3D::createZBuffer(unsigned char* pImage, double* zbuffer, gsl_matrix* P, int nW, int nH)
{
  if(!_hasInfo) return;

  gsl_vector* xi = gsl_vector_alloc(4);
  gsl_vector* ni  = gsl_vector_alloc(3);
  gsl_vector_set(xi, 3, 1.0);

  memset(pImage, 0, 3*nW*nH*sizeof(unsigned char));
  for(int i=0; i<nW*nH; i++)
    zbuffer[i] = 0.0;

  for(unsigned int i=0; i<_coords->size1; i++)
  {
    double* point = gsl_matrix_ptr(_coords, i, 0);

    if(_attributes[i] & ePointAttrValid)
    {
      gsl_vector_set(xi, 0, point[0]);
      gsl_vector_set(xi, 1, point[1]);
      gsl_vector_set(xi, 2, point[2]);

      gsl_blas_dgemv(CblasNoTrans, 1.0, P, xi, 0.0, ni);
      double du = gsl_vector_get(ni,0);
      double dv = gsl_vector_get(ni,1);
      double dw = gsl_vector_get(ni,2);
      if(dw > 10e-6)
      {
        int u = (int)( du / dw + 0.5);
        int v = nH-1-(int)(dv / dw + 0.5);

        if((u>=0) && (u<nW) && (v>=0) && (v<nH))
        {
          pImage[(v*nW+u)*3]   = _colors[3*i];
          pImage[(v*nW+u)*3+1] = _colors[3*i+1];
          pImage[(v*nW+u)*3+2] = _colors[3*i+2];
          zbuffer[v*nW+u]      = point[2];
        }
      }
    }
  }

  gsl_vector_free(ni);
  gsl_vector_free(xi);
}

void CartesianCloud3D::setData(const unsigned int size, double* coords,
    double* normals, const unsigned char* const rgb)
{
  //new content == old content -> overwrite content
  if((size == _size) && ((normals == NULL) && (_hasNormals == 0)) && ((rgb == NULL) && (_hasColors == false)))
  {
    for(unsigned int i = 0; i < size * 3; i++)
    {
      _coords->data[i] = coords[i];
      if(normals)
        _normals->data[i] = normals[i];
      if(rgb)
        _colors[i] = rgb[i];
//      else
//        _colors[i] = 255;
    }
  }
  else  //new content differs old content -> reallocate matrices
  {
    _mSourceInfo.clear();
    if(_hasInfo)
    {
      delete[] _colors;
      delete[] _attributes;
      delete[] _indices;
    }
    gsl_matrix_free(_coords);
    gsl_matrix_free(_normals);
    bool withInfo = (rgb!=NULL);
    init(size, withInfo);
    _size = size;
    gsl_matrix_view vcoords = gsl_matrix_view_array(coords, size, 3);
    gsl_matrix_memcpy(_coords, &vcoords.matrix);

    if(normals)
    {
      gsl_matrix_view vnormals = gsl_matrix_view_array(normals, size, 3);
      gsl_matrix_memcpy(_normals, &vnormals.matrix);
      _hasNormals = 1;
    }
    if(rgb)
    {
      memcpy(_colors, rgb, 3*size*sizeof(*_colors));
      _hasColors = true;
    }

  }
}

}
