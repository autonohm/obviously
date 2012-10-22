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
	  bool _debug_on;
    /**
     * Standard constructor
     * Allocates and initializes space and matrices
     * @param height,width,depth dimension of the allocated space in meters
     * @param vxldimension edge length of the Tsd_voxels
     */
    TsdSpace(const unsigned int height = 0, const unsigned int width = 0, const unsigned int depth = 0, const double vxldimension = 0, double *perspective = NULL);

    /**
     * Destructor
     */
    virtual ~TsdSpace(void );

    /**
     * push
     * Function to sort a depth image in the space (generate TSDF)
     * @param depth_image contains depth image as 2D Array
     * @return value MSG contains exit status
     */
    MSG push(double *depth_image);

    /**
     * set_max_truncation
     * Function to set the max truncation
     * @param new value new  max_truncation
     */
    inline void set_max_truncation(const double new_value) {_max_truncation=new_value;};

    /**
     * view_all
     * Writes colored images of all slices of the space to files
     */
    MSG view_all(void );

    /**
     * set_transformation
     * Function to set the current transformation matrix to the given values
     * @param transM_data content of the new transformation matrix
     */
    MSG set_transformation(double *transM_data);

    /**
     * gen_pcl
     * Function to generate pointcloud out of the tsd_space data
     * @param cloud space will be allocated by function and coordinates will be stored in
     */
    MSG gen_pcl(double **cloud, unsigned int *nbr);

    /**
     * get_model
     * Function to get virtual depth-image for ICP
     * sends rays through space
     * @param depth_image pointer to depth image. Will be allocated by function
     */
    MSG get_model(double **model_pcl, unsigned int *ctr);

    /**
     *
     */
    inline Matrix *get_transformation(void){return(_transformation);}

    /**
     * buildSliceImage
     * @brief Subfunction of view all
     * Checks one x,y slice of the space and writes the content to a file
     * @param depthIndex index of depth slice
     * @return success
     */
    MSG buildSliceImage(const unsigned int depthIndex, unsigned char* image);

    unsigned int getXDimension();
    unsigned int getYDimension();
    unsigned int getZDimension();

  private:

    /**
     * depth_slice
     * Subfunction of push
     * Checks one x,y slice of the space and updates the tsdf
     * Function will be called in multithreading
     * @param depth grows in z+ direction
     */
    MSG depth_slice(const unsigned int depth);

    /**
     * peak
     * Function which sends a ray through the space parallel to z axis
     * @param row,col Position of the peak within space
     * @param nbr Variable where the nbr of found points in peak is stored in
     * @param coordinates found coords are stored in. Space has to be allocated by calling function
     */
    MSG peak(unsigned int row, unsigned int col, unsigned int *nbr, double **coordinates);

    /**
     * ray_trace
     * Subfunction of get_model
     * sends ray through space to calculate virtual depth-image
     * @param row,col position of the pixel in virtual kinect-view
     * @param coordinates pointer to store intersection coordinates in
     * Has to be allocated by calling function
     */
    MSG ray_trace(const unsigned int row, const unsigned int col, double **coordinates, double *depth);

    /**
     * calc_ray
     * Subfunction of ray_trace
     * Calculates direction vector and footpoint of the ray
     * @param row,col position of the pixel in virtual kinect-view
     * @param dir_vec pointer to store normalized direction vector in
     * 			Has to be allocated by calling function (3-values)
     * @param foot_point pointer to store footpoint in has to be allocated by calling function (4-values)
     *
     */
    MSG calc_ray(const unsigned int row, const unsigned int col, double **dir_vec, double **foot_point);

    /**
     * interpolate_trilineary
     * Function to interpolate TSDF trilineary
     * @param coordinates pointer to coordinates of intersection
     * @param[out] tsdf interpolated TSD value
     */
    MSG interpolate_trilineary(double **coordinates, double *tsdf);

    unsigned int _x_nbr;

    unsigned int _y_nbr;

    unsigned int _z_nbr;

    unsigned int _height;

    unsigned int _width;

    unsigned int _depth;

    TsdVoxel ***_space;

    double _voxeldimension;

    obvious::Matrix *_transformation;

    obvious::Matrix *_inv_transformation;

    double _max_truncation;

    double *_act_depth_image;

    obvious::Matrix *_zero_h;

    Projection *_projection;
  };

}

#endif
