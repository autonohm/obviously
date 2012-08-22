#ifndef SYSTEM_H_
#define SYSTEM_H_

/**
 * @namespace obvious
 */
namespace obvious
{

/**
 * @class System
 * @brief This class encapsulates system specific calls for memory allocation
 * @author Stefan May
 */ 
template <class T>
class System
{
public:
    /**
     * Allocation of 2D arrays
     * @param unRows number of rows
     * @param unCols number of columns
     * @param aatArray data array
     */
    static void allocate (unsigned int unRows, unsigned int unCols, T** &array2D);
    /**
     * Deallocation of 2D arrays. Pointers are set to null.
     * @param aatArray data array
     */
    static void deallocate (T** &array2D);
    /**
     * Allocation of 3D arrays
     * @param unRows number of rows
     * @param unCols number of columns
     * @param unSlices number of slices
     * @param aaatArray data array
     */
    static void allocate (unsigned int unRows, unsigned int unCols, unsigned int unSlices, T*** &array3D);
    /**
     * Deallocation of 3D arrays. Pointers are set to null.
     * @param aaatArray data array
     */        
    static void deallocate (T*** &array3D);
};

#include "System.inl"

}

#endif /*SYSTEM_H_*/
