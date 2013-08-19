#include "obcore/base/System.h"
#include "obcore/base/Logger.h"
#include "obcore/base/Timer.h"
#include "obcore/math/Matrix.h"
#include "obcore/math/mathbase.h"
#include "TsdSpace.h"

#include <cstring>
#include <omp.h>

namespace obvious
{

#define MAXWEIGHT 128.0        //1.6e308     // maximum weight (near end of double)
#define RGB_MAX 255

TsdSpace::TsdSpace(const unsigned int height, const unsigned int width, const unsigned int depth, const unsigned int cols, const unsigned int rows, const double voxelSize, Matrix P)
{
   _voxelSize = voxelSize;
   _invVoxelSize = 1.0 / _voxelSize;
   // determine number of voxels in each dimension
   _xDim = ((double)width * _invVoxelSize + 0.5);
   _yDim = ((double)height * _invVoxelSize + 0.5);
   _zDim = ((double)depth * _invVoxelSize + 0.5);
   _sizeOfSpace = _zDim * _yDim * _xDim;

   _cols = cols;
   _rows = rows;


   _height = height;
   _width = width;
   _depth = depth;
   _maxTruncation = 2*voxelSize;
   _P = new Matrix(P);

   LOGMSG(DBG_DEBUG, "Dimensions are (x/y/z) (" << _xDim << "/" << _yDim << "/" << _zDim << ")");
   LOGMSG(DBG_DEBUG, "Creating TsdVoxel Space...");

   System<TsdVoxel>::allocate(_zDim, _yDim, _xDim, _space);

   _voxelCoords = new Matrix(_sizeOfSpace, 4);

   int i=0;
   for (int z = 0; z < _zDim; z++)
   {
      for (int y = 0; y < _yDim; y++)
      {
         for (int x = 0; x < _xDim; x++, i++)
         {
            _space[z][y][x].tsdf   = 1.0;
            _space[z][y][x].weight = 0.0;
            (*_voxelCoords)[i][0] = ((double)x + 0.5) * _voxelSize;
            (*_voxelCoords)[i][1] = ((double)y + 0.5) * _voxelSize;
            (*_voxelCoords)[i][2] = ((double)z + 0.5) * _voxelSize;
            (*_voxelCoords)[i][3] = 1.0;
         }
      }
   }

   _depthImage = new double[_cols * _rows];

   _T = new Matrix(4, 4);
   _T->setIdentity();
   (*_T)[0][3] = ((double)width) * 0.5;
   (*_T)[1][3] = ((double)height) * 0.5;
   (*_T)[2][3] = 0.0;

   _Tinv = new Matrix(4, 4);
   (*_Tinv) = _T->getInverse();

   _tr[0] = (*_T)[0][3];
   _tr[1] = (*_T)[1][3];
   _tr[2] = (*_T)[2][3];
}

TsdSpace::~TsdSpace(void)
{
   delete _Tinv;
   delete _T;
   delete [] _depthImage;
   delete _voxelCoords;
   delete [] _space;
   delete _P;
}

unsigned int TsdSpace::getXDimension()
{
   return _xDim;
}

unsigned int TsdSpace::getYDimension()
{
   return _yDim;
}

unsigned int TsdSpace::getZDimension()
{
   return _zDim;
}

double TsdSpace::getVoxelSize()
{
   return _voxelSize;
}

void TsdSpace::setMaxTruncation(double val)
{
   if(val < 2 * _voxelSize)
   {
      LOGMSG(DBG_WARN, "Truncation radius must be at 2 x voxel dimension. Setting minimum size.");
      val = 2 * _voxelSize;
   }

   _maxTruncation = val;
}

double TsdSpace::getMaxTruncation()
{
   return _maxTruncation;
}

void TsdSpace::setTransformation(double *TData)
{
   Matrix T(4, 4, TData);
   (*_T) *= T;

   (*_Tinv) = _T->getInverse();

   _tr[0] = (*_T)[0][3];
   _tr[1] = (*_T)[1][3];
   _tr[2] = (*_T)[2][3];
}

double* TsdSpace::getTransformation()
{
   return _T->getBuffer()->data;
}

void TsdSpace::push(double* depthImage, bool* mask, unsigned char* rgbImage)
{
   Timer t;

   for(unsigned int i=0; i<_cols * _rows; i++)
      _depthImage[i] = depthImage[i] * 0.001;

   _rgbImage = rgbImage;

   gsl_matrix* Pgen = gsl_matrix_alloc(3, 4);
   gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, _P->getBuffer(), _Tinv->getBuffer(), 0.0, Pgen);

   // coords2D = P * Tinv * voxelCoords
   gsl_matrix* coords2D = gsl_matrix_alloc(3, _sizeOfSpace);

   // Sequential version
   // gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, Pgen, _voxelCoords->getBuffer(), 0.0, coords2D);


   // OMP version
#pragma omp parallel
   {
#pragma omp for schedule(dynamic)
      for(int i=0; i<4; i++)
      {
         //gsl_matrix_view viewVoxel = gsl_matrix_submatrix (_voxelCoords->getBuffer(), 0, i*_sizeOfSpace/4, 4, _sizeOfSpace/4);
         gsl_matrix_view viewVoxel = gsl_matrix_submatrix (_voxelCoords->getBuffer(), i*_sizeOfSpace/4, 0, _sizeOfSpace/4, 4);
         gsl_matrix_view viewCoords2D = gsl_matrix_submatrix (coords2D, 0, i*_sizeOfSpace/4, 3, _sizeOfSpace/4);

         //gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Pgen, &(viewVoxel.matrix), 0.0, &(viewCoords2D.matrix));
         gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1.0, Pgen, &(viewVoxel.matrix), 0.0, &(viewCoords2D.matrix));
      }
   }

#pragma omp parallel
   {
#pragma omp for schedule(dynamic)
      for(int z=0; z<_zDim; z++)
      {
         int iz = z*_yDim*_xDim;
         for(int y=0; y<_yDim; y++)
         {
            int i = iz + y*_xDim;
            for(int x=0; x<_xDim; x++, i++)
            {
               const double dw = gsl_matrix_get(coords2D, 2, i);

               if(dw > 0.0)
               {
                  const double inv_dw = 1.0 / dw;
                  const double du = gsl_matrix_get(coords2D, 0, i) * inv_dw;
                  const double dv = gsl_matrix_get(coords2D, 1, i) * inv_dw;

                  const unsigned int u = static_cast<unsigned int>(du + 0.5);
                  const unsigned int v = static_cast<unsigned int>(dv + 0.5);

                  if(u < _cols && v < _rows && mask[((_rows - 1) - v) * _cols + u])
                  {
                     addTsdfValue((*_voxelCoords)[i], u, v, x, y, z);
                  }
               }
            }
         }
      }
   }

   gsl_matrix_free(Pgen);
   gsl_matrix_free(coords2D);

   LOGMSG(DBG_DEBUG, "Elapsed push: " << t.getTime() << "ms");
}

  bool TsdSpace::interpolateNormal(const double* coord, double* normal)
  {
    double neighbor[3];
    double depthVarInc = 0;
    double depthVarDec = 0;

    neighbor[0] = coord[0] + _voxelSize;        //interpolate around Voxel in x+1 direction
    neighbor[1] = coord[1];
    neighbor[2] = coord[2];

    if(!interpolateTrilinear(neighbor, &depthVarInc))
      return false;

    neighbor[0] = coord[0] - _voxelSize;        //interpolate around Voxel in x-1 direction
//    neighbor[1] = coord[1];
//    neighbor[2] = coord[2];

    if(!interpolateTrilinear(neighbor, &depthVarDec))
      return false;

    normal[0] = depthVarInc - depthVarDec;     //x_coordinate of the normal

    neighbor[0] = coord[0];
    neighbor[1] = coord[1] + _voxelSize;    //interpolate around Voxel in y+1 direction
//    neighbor[2] = coord[2];

    if(!interpolateTrilinear(neighbor, &depthVarInc))
      return false;

//    neighbor[0] = coord[0];
    neighbor[1] = coord[1] - _voxelSize;    //interpolate around Voxel in y-1 direction
//    neighbor[2] = coord[2];

    if(!interpolateTrilinear(neighbor, &depthVarDec))
      return false;

    normal[1] = depthVarInc - depthVarDec;     //y_coordinate of the normal

//    neighbor[0] = coord[0];
    neighbor[1] = coord[1];
    neighbor[2] = coord[2] + _voxelSize;         //interpolate around Voxel in z+1 direction

    if(!interpolateTrilinear(neighbor, &depthVarInc))
      return false;

//    neighbor[0] = coord[0];
//    neighbor[1] = coord[1];
    neighbor[2] = coord[2] - _voxelSize;         //interpolate around Voxel in z-1 direction

    if(!interpolateTrilinear(neighbor, &depthVarDec))
      return false;

    normal[2] = depthVarInc - depthVarDec;     //z_coordinate of the normal

    norm3<double>(normal);

    return true;
  }

  bool TsdSpace::interpolateTrilinear(double coord[3], double* tsdf)
  {
    int x, y, z;
    Point p;
    if(!coord2Voxel(coord, &x, &y, &z, &p)) return false;

    // get weights
    double wX = (coord[0] - p.x) * _invVoxelSize;
    double wY = (coord[1] - p.y) * _invVoxelSize;
    double wZ = (coord[2] - p.z) * _invVoxelSize;

    // Interpolate
    *tsdf =   _space[z + 0][y + 0][x + 0].tsdf * (1. - wX) * (1. - wY) * (1. - wZ)
                  + _space[z + 1][y + 0][x + 0].tsdf * (1. - wX) * (1. - wY) * wZ
                  + _space[z + 0][y - 1][x + 0].tsdf * (1. - wX) * wY * (1. - wZ)
                  + _space[z + 1][y - 1][x + 0].tsdf * (1. - wX) * wY * wZ
                  + _space[z + 0][y + 0][x + 1].tsdf * wX * (1. - wY) * (1. - wZ)
                  + _space[z + 1][y + 0][x + 1].tsdf * wX * (1. - wY) * wZ
                  + _space[z + 0][y - 1][x + 1].tsdf * wX * wY * (1. - wZ)
                  + _space[z + 1][y - 1][x + 1].tsdf * wX * wY * wZ;

    return true;
  }

  bool TsdSpace::interpolateTrilinearRGB(double coord[3], unsigned char rgb[3])
  {
    int x, y, z;
    Point p;
    if(!coord2Voxel(coord, &x, &y, &z, &p)) return false;

    // get weights
    double wX = (coord[0] - p.x) * _invVoxelSize;
    double wY = (coord[1] - p.y) * _invVoxelSize;
    double wZ = (coord[2] - p.z) * _invVoxelSize;

    unsigned char pRGB[8][3];

    pRGB[0][0] = _space[z + 0][y + 0][x + 0].rgb[0];
    pRGB[0][1] = _space[z + 0][y + 0][x + 0].rgb[1];
    pRGB[0][2] = _space[z + 0][y + 0][x + 0].rgb[2];

    pRGB[1][0] = _space[z + 1][y + 0][x + 0].rgb[0];
    pRGB[1][1] = _space[z + 1][y + 0][x + 0].rgb[1];
    pRGB[1][2] = _space[z + 1][y + 0][x + 0].rgb[2];

    pRGB[2][0] = _space[z + 0][y - 1][x + 0].rgb[0];
    pRGB[2][1] = _space[z + 0][y - 1][x + 0].rgb[1];
    pRGB[2][2] = _space[z + 0][y - 1][x + 0].rgb[2];

    pRGB[3][0] = _space[z + 1][y - 1][x + 0].rgb[0];
    pRGB[3][1] = _space[z + 1][y - 1][x + 0].rgb[1];
    pRGB[3][2] = _space[z + 1][y - 1][x + 0].rgb[2];

    pRGB[4][0] = _space[z + 0][y - 0][x + 1].rgb[0];
    pRGB[4][1] = _space[z + 0][y - 0][x + 1].rgb[1];
    pRGB[4][2] = _space[z + 0][y - 0][x + 1].rgb[2];

    pRGB[5][0] = _space[z + 1][y - 0][x + 1].rgb[0];
    pRGB[5][1] = _space[z + 1][y - 0][x + 1].rgb[1];
    pRGB[5][2] = _space[z + 1][y - 0][x + 1].rgb[2];

    pRGB[6][0] = _space[z + 0][y - 1][x + 1].rgb[0];
    pRGB[6][1] = _space[z + 0][y - 1][x + 1].rgb[1];
    pRGB[6][2] = _space[z + 0][y - 1][x + 1].rgb[2];

    pRGB[7][0] = _space[z + 1][y - 1][x + 1].rgb[0];
    pRGB[7][1] = _space[z + 1][y - 1][x + 1].rgb[1];
    pRGB[7][2] = _space[z + 1][y - 1][x + 1].rgb[2];

    double pw[8];
    pw[0] = (1. - wX) * (1. - wY) * (1. - wZ);
    pw[1] = (1. - wX) * (1. - wY) * wZ;
    pw[2] = (1. - wX) * wY * (1. - wZ);
    pw[3] = (1. - wX) * wY * wZ;
    pw[4] = wX * (1. - wY) * (1. - wZ);
    pw[5] = wX * (1. - wY) * wZ;
    pw[6] = wX * wY * (1. - wZ);
    pw[7] = wX * wY * wZ;

    memset(rgb,0,3);
    for(int i=0; i<8; i++)
    {
       rgb[0] += pRGB[i][0] * pw[i];
       rgb[1] += pRGB[i][1] * pw[i];
       rgb[2] += pRGB[i][2] * pw[i];
    }

/*
    // Interpolate
    rgb[0] =  _space[z + 0][y + 0][x + 0].rgb[0] * (1. - wX) * (1. - wY) * (1. - wZ)
                  + _space[z + 1][y + 0][x + 0].rgb[0] * (1. - wX) * (1. - wY) * wZ
                  + _space[z + 0][y - 1][x + 0].rgb[0] * (1. - wX) * wY * (1. - wZ)
                  + _space[z + 1][y - 1][x + 0].rgb[0] * (1. - wX) * wY * wZ
                  + _space[z + 0][y + 0][x + 1].rgb[0] * wX * (1. - wY) * (1. - wZ)
                  + _space[z + 1][y + 0][x + 1].rgb[0] * wX * (1. - wY) * wZ
                  + _space[z + 0][y - 1][x + 1].rgb[0] * wX * wY * (1. - wZ)
                  + _space[z + 1][y - 1][x + 1].rgb[0] * wX * wY * wZ;

    rgb[1] =  _space[z + 0][y + 0][x + 0].rgb[1] * (1. - wX) * (1. - wY) * (1. - wZ)
                  + _space[z + 1][y + 0][x + 0].rgb[1] * (1. - wX) * (1. - wY) * wZ
                  + _space[z + 0][y - 1][x + 0].rgb[1] * (1. - wX) * wY * (1. - wZ)
                  + _space[z + 1][y - 1][x + 0].rgb[1] * (1. - wX) * wY * wZ
                  + _space[z + 0][y + 0][x + 1].rgb[1] * wX * (1. - wY) * (1. - wZ)
                  + _space[z + 1][y + 0][x + 1].rgb[1] * wX * (1. - wY) * wZ
                  + _space[z + 0][y - 1][x + 1].rgb[1] * wX * wY * (1. - wZ)
                  + _space[z + 1][y - 1][x + 1].rgb[1] * wX * wY * wZ;

    rgb[2] =  _space[z + 0][y + 0][x + 0].rgb[2] * (1. - wX) * (1. - wY) * (1. - wZ)
                  + _space[z + 1][y + 0][x + 0].rgb[2] * (1. - wX) * (1. - wY) * wZ
                  + _space[z + 0][y - 1][x + 0].rgb[2] * (1. - wX) * wY * (1. - wZ)
                  + _space[z + 1][y - 1][x + 0].rgb[2] * (1. - wX) * wY * wZ
                  + _space[z + 0][y + 0][x + 1].rgb[2] * wX * (1. - wY) * (1. - wZ)
                  + _space[z + 1][y + 0][x + 1].rgb[2] * wX * (1. - wY) * wZ
                  + _space[z + 0][y - 1][x + 1].rgb[2] * wX * wY * (1. - wZ)
                  + _space[z + 1][y - 1][x + 1].rgb[2] * wX * wY * wZ;
*/
    return true;
  }

  void TsdSpace::addTsdfValue(const double coordVoxel[3], const unsigned int u, const unsigned int v, const unsigned int col, const unsigned int row, const unsigned int z)
  {
    double distance;
    double xl;
    double yl;
    double lambdaInv;
    double tsdf;
    double sdf;
    TsdVoxel *voxel;

    // calculate distance of current voxel to kinect
    distance = euklideanDistance<double>(_tr, (double*)coordVoxel, 3);

    xl = (u - (*_P)[0][2]) / (*_P)[0][0];
    yl = (v - (*_P)[1][2]) / (*_P)[1][1];
    lambdaInv = 1. / sqrt(xl * xl + yl * yl + 1.);
    unsigned int idx = ((_rows - 1) - v) * _cols + u;
    sdf = _depthImage[idx] - distance * lambdaInv; //mm

    if(sdf >= -_maxTruncation) // Voxel is in front of an object
    {
      voxel = &_space[z][(_yDim - 1) - row][col];

      // determine whether sdf/max_truncation = ]-1;1[
      tsdf = sdf / _maxTruncation;
      tsdf = min(tsdf, 1.0);


      voxel->weight += 1.0;
      const double invWeight = 1.0 / voxel->weight;
      const unsigned char* rgbImage = &_rgbImage[idx * 3];
      voxel->tsdf   = (voxel->tsdf * (voxel->weight - 1.0) + tsdf) * invWeight;
      voxel->rgb[0] = voxel->rgb[0] + static_cast<unsigned char>(static_cast<double>(*rgbImage++ - voxel->rgb[0]) * invWeight);
      voxel->rgb[1] = voxel->rgb[1] + static_cast<unsigned char>(static_cast<double>(*rgbImage++ - voxel->rgb[1]) * invWeight);
      voxel->rgb[2] = voxel->rgb[2] + static_cast<unsigned char>(static_cast<double>(*rgbImage   - voxel->rgb[2]) * invWeight);

/*
      voxel->tsdf   = (voxel->tsdf * voxel->weight + 1.0 * tsdf) / (voxel->weight + 1.0);
      voxel->rgb[0] = (unsigned char)( (((double)voxel->rgb[0]) * voxel->weight + ((double)_rgbImage[idx * 3]) )   / (voxel->weight + 1.0));
      voxel->rgb[1] = (unsigned char)( (((double)voxel->rgb[1]) * voxel->weight + ((double)_rgbImage[idx * 3+1]) ) / (voxel->weight + 1.0));
      voxel->rgb[2] = (unsigned char)( (((double)voxel->rgb[2]) * voxel->weight + ((double)_rgbImage[idx * 3+2]) ) / (voxel->weight + 1.0));
      voxel->weight += 1.0;
*/

      voxel->weight = min(voxel->weight, MAXWEIGHT);
    }
  }

  inline bool TsdSpace::coord2Voxel(double coord[3], int* x, int* y, int* z, Point* p)
  {
    // initialize
    // get voxel indices
    int xIdx = (int) (coord[0] * _invVoxelSize);
    int yIdx = (int) (coord[1] * _invVoxelSize);
    int zIdx = (int) (coord[2] * _invVoxelSize);

    // check edges / 0 is edge because of voxelfinetuning
    if ((xIdx >= (_xDim - 2)) || (xIdx < 1) || (yIdx >= (_yDim - 2)) || (yIdx < 1) || (zIdx >= (_zDim - 2)) || (zIdx < 1))
      return false;

    // get center point of current voxel
    p->x = (double(xIdx) + 0.5) * _voxelSize;
    p->y = (double(yIdx) + 0.5) * _voxelSize;
    p->z = (double(zIdx) + 0.5) * _voxelSize;

    // voxel fine tuning -> shift to lower-left-front edge
    if (coord[0] < p->x)
    {
      xIdx--;
      p->x -= _voxelSize;
    }
    if (coord[1] < p->y)
    {
      yIdx--;
      p->y -= _voxelSize;
    }
    if (coord[2] < p->z)
    {
      zIdx--;
      p->z -= _voxelSize;
    }

    // turn y-axis
    yIdx = (_yDim - 1) - yIdx;

    *x = xIdx;
    *y = yIdx;
    *z = zIdx;

    return true;
  }

  bool TsdSpace::buildSliceImage(const unsigned int depthIndex, unsigned char* image)
  {
    unsigned char R[_xDim * _yDim];
    unsigned char G[_xDim * _yDim];
    unsigned char B[_xDim * _yDim];
    unsigned int ctr = 0;
    unsigned im_ctr = 0;
    double cTsdf;

    // initialize arrays for RGB
    for (int i = 0; i < _xDim * _yDim; i++)
    {
      R[i] = 0;
      G[i] = 0;
      B[i] = 0;
    }

    // iterate over given slice, generate 2D-Picture
    for (int row = 0; row < _yDim; row++)
    {
      for (int col = 0; col < _xDim; col++)
      {
        // Get current tsdf
        cTsdf = _space[depthIndex][row][col].tsdf;

        // Blue for depth behind Voxel
        if (cTsdf > 0)
        {
          if (cTsdf > 0.999)
            G[im_ctr++] = (unsigned char) 150; //row*_xDim+col
          else
            B[im_ctr++] = (unsigned char) (cTsdf * RGB_MAX + 0.5); //row*_xDim+col
        }

        // Red for depth in front of Voxel
        else if (cTsdf < 0)
        {
          if (cTsdf < -0.9999)
            G[im_ctr++] = (unsigned char) 50;
          else
            R[im_ctr++] = (unsigned char) ((cTsdf * -1.0) * RGB_MAX + 0.5); //row*_xDim+col
        }
      }
    }

    //put components together to complete picture
    for (int i = 0; i < _xDim * _yDim * 3; i++)
    {
      image[i]   = R[ctr];
      image[++i] = G[ctr];
      image[++i] = B[ctr];
      ctr++;
    }

    return (true);
  }

}
