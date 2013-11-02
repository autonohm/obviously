#include "TriangleMesh.h"

#include "obcore/base/System.h"
#include <gsl/gsl_matrix.h>
#include "obcore/math/mathbase.h"
#include "obcore/math/Matrix.h"
#include <string.h>

using namespace std;
using namespace obvious;

namespace obvious
{

TriangleMesh::TriangleMesh(unsigned int maxsize, double maxDiscontinuity)
{
  System<double>::allocate(maxsize, 3, _coords);
  System<double>::allocate(maxsize, 3, _normals);
  System<unsigned char>::allocate(maxsize, 3, _rgb);
  System<unsigned int>::allocate(2*maxsize, 3, _indices);
  System<unsigned int>::allocate(2*maxsize, 3, _inputIndices);
  _size                = 0;
  _triangles           = 0;
  _validIndices        = new unsigned int[maxsize];
  _maxDiscontinuitySqr = maxDiscontinuity*maxDiscontinuity;
}

TriangleMesh::~TriangleMesh()
{
  System<double>::deallocate(_coords);
  System<double>::deallocate(_normals);
  System<unsigned char>::deallocate(_rgb);
  System<unsigned int>::deallocate(_indices);
  System<unsigned int>::deallocate(_inputIndices);
  delete [] _validIndices;
}

double** TriangleMesh::getCoords()
{
  return _coords;
}

double** TriangleMesh::getNormals()
{
  return _coords;
}

unsigned char** TriangleMesh::getRGB()
{
  return _rgb;
}

unsigned int** TriangleMesh::getIndices()
{
  return _indices;
}

unsigned int** TriangleMesh::getInputIndices()
{
  return _inputIndices;
}

unsigned int TriangleMesh::getNumberOfPoints()
{
  return _size;
}

unsigned int TriangleMesh::getNumberOfTriangles()
{
  return _triangles;
}

double TriangleMesh::computeSurface()
{
  double surface = 0.0;
  for(unsigned int i=0; i<_triangles; i++)
  {
    double* p1 = _coords[_indices[i][0]];
    double* p2 = _coords[_indices[i][1]];
    double* p3 = _coords[_indices[i][2]];
    double v1[3];
    double v2[3];
    // normal vector
    double n[3];
    v1[0] = p2[0] - p1[0];
    v1[1] = p2[1] - p1[1];
    v1[2] = p2[2] - p1[2];
    v2[0] = p3[0] - p1[0];
    v2[1] = p3[1] - p1[1];
    v2[2] = p3[2] - p1[2];
    cross3<double>(n, v1, v2);
    surface += 0.5*sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
  }
  return surface;
}

void TriangleMesh::createMeshFromOrganizedCloud(double* coords, unsigned int rows, unsigned cols, unsigned char* rgb, bool* mask, double* normals)
{
  _size = 0;
  _triangles = 0;

  // 1) Create hash table for valid points
  // 2) Copy only valid points to coordinate and rgb array
  for(unsigned int i=0; i<rows*cols; i++)
  {
    _validIndices[i] = _size;
    if(mask[i])
    {
      memcpy(_coords[_size], &coords[3*i], 3*sizeof(*coords));
      if(rgb) memcpy(_rgb[_size],    &rgb[3*i],    3*sizeof(*rgb));
      if(normals) memcpy(_normals[_size],&normals[3*i],3*sizeof(*normals));
      _size++;
    }
  }

  // Create triangles in local neighborhood
  for(unsigned int r=0; r<rows-1; r++)
  {
    for(unsigned int c=0; c<cols-1; c++)
    {
      unsigned int idx    = r*cols+c;
      unsigned int idx_r  = idx+1;
      unsigned int idx_d  = idx+cols;
      unsigned int idx_dr = idx_d+1;

      bool isValid1 = mask[idx]    && mask[idx_d] && mask[idx_r];
      bool isValid2 = mask[idx_r]  && mask[idx_d] && mask[idx_dr];
      bool isValid3 = mask[idx]    && mask[idx_d] && mask[idx_dr];
      bool isValid4 = mask[idx]    && mask[idx_r] && mask[idx_dr];

      if(isValid1)
      {
        double distSqr = distSqr3D(&coords[idx*3], &coords[idx_d*3]);
        if(distSqr < _maxDiscontinuitySqr)
        {
          distSqr = distSqr3D(&coords[idx_d*3], &coords[idx_r*3]);
          if(distSqr < _maxDiscontinuitySqr)
          {
            _indices[_triangles][0]      = _validIndices[idx];
            _indices[_triangles][1]      = _validIndices[idx_d];
            _indices[_triangles][2]      = _validIndices[idx_r];
            _inputIndices[_triangles][0] = idx;
            _inputIndices[_triangles][1] = idx_d;
            _inputIndices[_triangles][2] = idx_r;
            _triangles++;
          }
        }
      } // isValid

      if(isValid2)
      {
        double distSqr = distSqr3D(&coords[idx_r*3], &coords[idx_d*3]);
        if(distSqr < _maxDiscontinuitySqr)
        {
          distSqr = distSqr3D(&coords[idx_d*3], &coords[idx_dr*3]);
          if(distSqr < _maxDiscontinuitySqr)
          {
            _indices[_triangles][0]      = _validIndices[idx_r];
            _indices[_triangles][1]      = _validIndices[idx_d];
            _indices[_triangles][2]      = _validIndices[idx_dr];
            _inputIndices[_triangles][0] = idx_r;
            _inputIndices[_triangles][1] = idx_d;
            _inputIndices[_triangles][2] = idx_dr;
            _triangles++;
          }
        }
      } // isValid2

      if(isValid3 && !isValid1 && !isValid2)
      {
        double distSqr = distSqr3D(&coords[idx*3], &coords[idx_d*3]);
        if(distSqr < _maxDiscontinuitySqr)
        {
          distSqr = distSqr3D(&coords[idx_d*3], &coords[idx_dr*3]);
          if(distSqr < _maxDiscontinuitySqr)
          {
            _indices[_triangles][0]      = _validIndices[idx];
            _indices[_triangles][1]      = _validIndices[idx_d];
            _indices[_triangles][2]      = _validIndices[idx_dr];
            _inputIndices[_triangles][0] = idx;
            _inputIndices[_triangles][1] = idx_d;
            _inputIndices[_triangles][2] = idx_dr;
            _triangles++;
          }
        }
      } // isValid3

      if(isValid4 && !isValid1 && !isValid2)
      {
        double distSqr = distSqr3D(&coords[idx*3], &coords[idx_r*3]);
        if(distSqr < _maxDiscontinuitySqr)
        {
          distSqr = distSqr3D(&coords[idx_r*3], &coords[idx_dr*3]);
          if(distSqr < _maxDiscontinuitySqr)
          {
            _indices[_triangles][0]      = _validIndices[idx];
            _indices[_triangles][1]      = _validIndices[idx_r];
            _indices[_triangles][2]      = _validIndices[idx_dr];
            _inputIndices[_triangles][0] = idx;
            _inputIndices[_triangles][1] = idx_r;
            _inputIndices[_triangles][2] = idx_dr;
            _triangles++;
          }
        }
      } // isValid4

    }
  }

}

} // end namespace

