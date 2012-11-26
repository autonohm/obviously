#ifndef _TSD_SPACE_H
#define _TSD_SPACE_H

#include "obcore/math/Matrix.h"
#include "obvision/reconstruct/Projection.h"
#include "obvision/reconstruct/TsdSpace_utils.h"

namespace obvious
{

  class TsdSpace
  {
  public:

    /**
     * Standard constructor
     * Allocates and initializes space and matrices
     * @param height,width,depth dimension of the allocated space in meters
     * @param vxldimension edge length of the Tsd_voxels
     */
    TsdSpace(const unsigned int height = 0, const unsigned int width = 0, const unsigned int depth = 0, const double voxelDim = 0, double *perspective = NULL);

    /**
     * Destructor
     */
    virtual ~TsdSpace(void );

    MSG push(double *depthImage);

    MSG calcTsdf(const double curVxlccoords[4],const unsigned int u,const unsigned int v,const unsigned int col,const unsigned int row,const unsigned int z);

    /**
     * setMaxTruncation
     * Function to set the max truncation
     * @param new value new  max_truncation
     */
    void setMaxTruncation(const double new_value);

    /**
     * set_transformation
     * Function to set the current transformation matrix to the given values
     * @param transM_data content of the new transformation matrix
     */
    MSG setTransformation(double *transM_data);

    /**
     * gen_pcl
     * Function to generate pointcloud out of the tsd_space data
     * @param cloud space will be allocated by function and coordinates will be stored in
     */
    MSG generatePointcloud(double **cloud,double **normals, unsigned int *nbr);

    /**
     * get_model
     * Function to get virtual depth-image for ICP
     * sends rays through space
     * @param depth_image pointer to depth image. Will be allocated by function
     */
    MSG getModel(double **modelPcl,double *modelNormals,unsigned int *ctr);

    /**
     *
     */
    Matrix *getTransformation();

    /**
 	   *
	   */
    Matrix *getinvTransformation();

    /**
     * buildSliceImage
     * Checks one x,y slice of the space and writes the content to a file
     * @param depthIndex index of depth slice
     * @return success
     */
    MSG buildSliceImage(const unsigned int depthIndex, unsigned char* image);

    unsigned int getXDimension();
    unsigned int getYDimension();
    unsigned int getZDimension();
    double getVxlDimension();

  private:

    /**
     * rayCast
     * Subfunction
     * sends ray through space to calculate virtual depth-image
     * @param row,col position of the pixel in virtual kinect-view
     * @param coordinates pointer to store intersection coordinates in
     * Has to be allocated by calling function
     * @param mode sets the raycaster to different modes.
     * 		GENERAL normal mode
     * 		X_AXS parallel to X-Axis Borders : COL = _zDim, ROW = _yDim
     * 		X_AXS_N parallel to X-Axis negative direction
     * 		Y_AXS parallel to Y-Axis Borders : COL = _xDim, ROW = _zDim
     * 		Y_AXS_N parallel to Y-Axis negative direction
     * 		Z_AXS parallel to Z-Axis Borders : COL = _xDim, ROW = _yDim
     * 		Z_AXS_N parallel to Z-Axis negative direction
     */
    //MSG rayCast(const unsigned int row, const unsigned int col, double **coordinates, double *depth,RAYC_MODE mode);
    MSG rayCast(const unsigned int row, const unsigned int col, double **coordinates,double *normal,double *depth,RAYC_MODE mode);
    /**
     * calcRay
     * Subfunction of rayCast
     * Calculates direction vector and footpoint of the ray
     * @param row,col position of the pixel in virtual kinect-view
     * @param dir_vec pointer to store normalized direction vector in
     * 			Has to be allocated by calling function (3-values)
     * @param foot_point pointer to store footpoint in has to be allocated by calling function (4-values)
     *
     */
    MSG calcRay(const unsigned int row, const unsigned int col, double **dir_vec, double **foot_point,RAYC_MODE mode);

    /**
	  * Subfunction of rayCast
	  * Calculates normal to a hit plain element
	  * @param normalCoords Variable to store the coordinates in. Has to be allocated by calling function (3 coordinates)
	  */
	 MSG getNormal(const double *crosCoords,double *normalCoords);

    /**
     * interpolate_trilineary
     * Function to interpolate TSDF trilineary
     * @param coordinates pointer to coordinates of intersection
     * @param[out] tsdf interpolated TSD value
     */
    MSG interpolateTrilinear(double **coordinates, double *tsdf);

    double interpolateBilinear(double u, double v);

    MSG secSearch(const unsigned int xIdx,const unsigned int yIdx,const unsigned int zIdx);

    int _xDim;

    int _yDim;

    int _zDim;

    int _height;

    int _width;

    int _depth;

    TsdVoxel ***_space;

    double _voxelDim;

    obvious::Matrix *_T;

    obvious::Matrix *_Tinv;

    double _maxTruncation;

    double *_depthImage;

    obvious::Matrix *_zero_h;

    Projection *_projection;
  };

}

#endif
