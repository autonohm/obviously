#ifndef __DEVICE_3D__
#define __DEVICE_3D__

#include "InputDevice.h"
#include "obcore/math/MatD.h"

#include <iostream>
#include <fstream>
#include <vector>

namespace obvious {

class Point3D;

class Device3D : public InputDevice
{
public:
    /**
     * Standard constructor
     * @param   name    name of device
     */
    Device3D(const std::string& name)
      : InputDevice(name),
        _coords(NULL), _mask(NULL), _z(NULL),
        _rows(0), _cols(0),
        _record(false) { }
    /**
     * Default destructor
     */
    virtual ~Device3D(void) = 0;

    const std::vector<Point3D*>& points(void) const { return m_points; }
    /**
     * Get number of rows of images
     * @return rows
     */
    unsigned int getRows() const  { return _rows; }
    /**
     * Get number of columns of images
     * @return columns
     */
    unsigned int getCols() const  { return _cols; }
    /**
     * Accessor to pointer of coordinate data
     * @return pointer to coordinate buffer (layout x1y1z1x2...)
     */
    double* getCoords() const     { return _coords; }
    /**
     * Accessor to mask of valid points
     * @return pointer to mask
     */
    bool* getMask() const         { return _mask; }

    /**
     * Get pointer to z-Buffer
     */
    double* getZ() const          { return _z; }
    /**
     * Get Z buffer
     * @return MatD
     */
    MatD getMatZ(void) const;
    /**
     * Start serializing the data stream to file
     * @param filename name of file
     */
    void startRecording(char* filename);
    /**
     * Stop previously started recording
     */
    void stopRecording(void);
protected:
    virtual void record(void) = 0;
    void deletePoints(void);
    double*       _coords;
    unsigned int _rows;
    unsigned int _cols;
    double*       _z;
    bool*         _mask;
    std::vector<Point3D*> m_points;

    bool          _record;
    std::ofstream  _recfile;
};

} // end namespace obvious

#endif
