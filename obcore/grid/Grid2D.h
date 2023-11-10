/**
* @file   2DGrid.h
* @author Christian Pfitzner
* @date   11.12.2012
*
*
*/

#ifndef GRID2D_H_
#define GRID2D_H_

/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * @enum SUCCESFUL
 */
enum SUCCESFUL
{
  ERROR, //!< ERROR
  ALRIGHT//!< ALRIGHT
};

/**
 * @class Grid2D
 */
class Grid2D
{
public:
  /**
   * Default constructor
   * @param[in]   resolution    length of one grid cell
   * @param[in]   length        length of grid
   * @param[in]   width         width of grid
   */
  Grid2D(const double resolution = 0.05, const double length = 2.005,
          const double width = 2.005, const unsigned int channels = 1);
  /**
   * Destructor
   */
  virtual ~Grid2D(void);
  /**
   * Function to fit cloud in grid
   * @param[in]     cloud   cloud with points in double array
   * @param[in]     size    number of points in cloud
   * @return        TRUE if no error occurs @see SUCCESFUL
   */
  SUCCESFUL cloud2Grid(const double* cloud, unsigned int size);
  /**
   * Function to return number of rows
   * @return    rows
   */
  unsigned int getRows(void) const;
  /**
   * Function to return number columns
   * @return    columns
   */
  unsigned int getCols(void) const;
  /**
   * Function to return resolution of grid in meter
   * @return    resolution in meter
   */
  double getResolution(void) const;
  /**
   * Function to return all fitted points/obstacles in grid
   * @return    number of points
   */
  unsigned int getPointsInGrid(void);
  /**
   * Function to return unsigned char image for visualization
   * @param[in]     img    allocated unsigned char array
   */
  unsigned char* getImageOfGrid(void);
  /**
   * Function to get matrix
   * @return  Matrix
   */
  MatD& getMat(void);
protected:
  /**
   * Function to init single channel
   */
  void initChannel(unsigned int channel);
  /**
   * Function to init grid with INIT_DOUBLE
   * @see INIT_DOUBLE
   */
  void initGrid(void);
  /**
   * Function to estimate index for x in grid
   * @param[in]     xValue  x value of point in cloud
   * @return        x index
   */
  int getIndexX(const double& xValue) const;
  /**
   * Function to estimate index for x in grid
   * @param[in]     yValue  y value of point in cloud
   * @return        y index
   */
  int getIndexY(const double& yValue) const;

  //!< @param   INIT_DOUBLE       0.0
  static const double         INIT_DOUBLE    = 0.0;
  //!< @param   FREE_COLOR        0
  static const unsigned char FREE_COLOR     = 0;
  //!< @param   SET_COLOR        0
  static const unsigned char SET_COLOR      = 255;

  double              _resolution;          //!< resolution of grid
  double              _width;               //!< width of grid in meters
  double              _length;              //!< length of grid in meters
  unsigned int       _rows;                 //!< number of rows
  unsigned int       _cols;                 //!< number of columns
  bool                _pointsEstimated;      //!< true if function getPointsInGrid was called
  MatD*                _grid;                //!< pointer on grid
  unsigned char*     _img;                  //!< image for visualization
};

} // namespace

#endif /* GRID2D_H_ */
