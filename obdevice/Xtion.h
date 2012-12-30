/**
* @file Xtion.h
* @autor Christian Pfitzner
* @date  23.12.2012
*/

#ifndef XTION_H_
#define XTION_H_

#include "obdevice/OpenNIDevice.h"

using namespace xn;
using namespace std;

/**
 * @namespace obvious
 */
namespace obvious
{
/**
 * @class   Xtion
 * @brief   Xtion interface
 * @author  Christian Pfitzner
 **/
class Xtion : public OpenNIDevice
{
public:
  /**
   * Standard constructor;
   */
  Xtion(const char* path);
  /**
   * Standard destructor
   */
  virtual ~Xtion();
  /**
   * Grab new image
   * @return success
   */
  bool grab();
};

} // end namespace

#endif /* XTION_H_ */
