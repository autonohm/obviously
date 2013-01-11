/**
* @file   XtionDevice.h
* @author Christian Pfitzner
* @date   07.01.2013
*
*
*/

#ifndef XTIONDEVICE_H_
#define XTIONDEVICE_H_

#include "Device2D.h"
#include "Device3D.h"
#include <string>

/**
 * @namespace obvious
 */
namespace obvious {

class Xtion;
/**
 * @class XtionDevice
 */
class XtionDevice : public Device2D, public Device3D
{
public:
    /**
     * Standard constructor
     * @param[in]   configFile    config file for openNI
     */
    XtionDevice(const char* configFile);
    /**
     * Default destructor
     */
    virtual ~XtionDevice(void);
    /**
     * Function to grab new data from Xtion class
     * @return  success
     */
    virtual bool grab(void);
    /**
     * Function to return coords in double*
     * @return  coords
     */
    double* getCoords(void);
    /**
     * Function to return number of columns
     * @return  columns
     */
    unsigned int getCols(void);
    /**
     * Function to return number of rows
     * @return  rows
     */
    unsigned int getRows(void);
private:
    Xtion* _xtion;
};

} // end namespace obvious

#endif /* XTIONDEVICE_H_ */
