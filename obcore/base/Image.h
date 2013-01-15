/**
* @file   Image.h
* @author Christian Pfitzner
* @date   12.01.2013
*
*
*/

#ifndef IMAGE_H_
#define IMAGE_H_
/**
 * @namespace obvious
 */
namespace obvious{
/**
 * @class Image
 * @brief Container class to work with images
 */
class Image
{
public:
  /**
   * @enum enumImgType
   * Switch betweeen grey and colored image
   */
  enum enumImgType{GREY=1, COLORED=3};
  /**
   * Standard constructor
   * @param     width   width of image in pixels
   * @param     height  heights of image in pixels
   * @param     type    type of image @see enum
   */
  Image(const unsigned int width, const unsigned int height, const enumImgType type = GREY)
    : _width(width), _height(height), _type(type)
    { _img = new unsigned char[width*height*type]; }
  /**
   * Default destructor
   */
  ~Image(void) { delete[] _img; }
  /**
   * Function to set image in containers
   * @param     img     image in unsigned char*
   */
  void setImg(unsigned char* img)   { _img = img; }
  /**
   * Function to return image out of container class
   * @return    image in unsigned char*
   */
  unsigned char* getImg(void) const     { return _img; }
  /**
   * Function to get type of image
   * @return
   */
  unsigned int getType(void) const      { return _type; }
  /**
   * Function to return width of image
   * @return    width of image in pixels
   */
  unsigned int getWidth(void) const     { return _width; }
  /**
   * Function to return height of image
   * @return    height of image in pixels
   */
  unsigned int getHeight(void) const    { return _height; }
private:
  unsigned char* _img;              ///<! image in data type unsigned char*
  unsigned int   _width;            ///<! width of image in pixels
  unsigned int   _height;           ///<! height of image in pixels
  enumImgType     _type;             ///<! type of image (grey or colored)
};

} // namespace



#endif /* IMAGE_H_ */
