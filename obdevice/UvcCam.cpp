#include "UvcCam.h"

#include "obcore/base/Logger.h"

#include <libudev.h>
#include <linux/videodev2.h>

#define DHT_SIZE 420

/**
 * Initial implementation taken from openvolksbot library
 */

namespace obvious
{
struct buffer
{
   void * start;
   size_t length;
};

#define CLEAR(x) memset (&(x), 0, sizeof (x))
#define CLIP(color) (unsigned char)(((color)>0xFF)?0xff:(((color)<0)?0:(color)))
#define R_FROMYV(y,v)  CLIP((y) + _LutRv[(v)])
#define G_FROMYUV(y,u,v) CLIP((y) + _LutGu[(u)] + _LutGv[(v)])
#define B_FROMYU(y,u) CLIP((y) + _LutBu[(u)])
#define CoefRv 1402
#define CoefGu 714 // 344
#define CoefGv 344 // 714
#define CoefBu 1772

UvcCam::UvcCam(const char *dev, unsigned int width, unsigned int height)
{
   _nDeviceHandle = -1;
   _dev           = new char[strlen(dev)];
   strcpy(_dev, dev);
   _width         = width;
   _height        = height;
   _colorMode     = CAMRGB;
   _LutRv         = (int *) malloc(256 * sizeof(int));
   _LutGu         = (int *) malloc(256 * sizeof(int));
   _LutGv         = (int *) malloc(256 * sizeof(int));
   _LutBu         = (int *) malloc(256 * sizeof(int));
   _nb_buffers    = NB_BUFFERS;
   _mem           = NULL;
   for (int i = 0; i < 256; i++)
   {
      _LutRv[i] = (i - 128) * CoefRv / 1000;
      _LutGu[i] = (128 - i) * CoefGu / 1000;
      _LutGv[i] = (128 - i) * CoefGv / 1000;
      _LutBu[i] = (i - 128) * CoefBu / 1000;
   }
}

UvcCam::~UvcCam()
{
   disconnect();
   delete _dev;
}

void UvcCam::FindDevice(const char* serial, char* &path)
{
   struct udev *udev;
   struct udev_enumerate *enumerate;
   struct udev_list_entry *devices, *dev_list_entry;

   udev      = udev_new();
   enumerate = udev_enumerate_new(udev);
   udev_enumerate_add_match_subsystem(enumerate, "video4linux");
   udev_enumerate_scan_devices(enumerate);
   devices   = udev_enumerate_get_list_entry(enumerate);

   udev_list_entry_foreach(dev_list_entry, devices)
   {
      const char *sysfs_path;
      const char *dev_path;
      struct udev_device *uvc_dev; // The device's UVC udev node.
      struct udev_device *dev;     // The actual hardware device.

      sysfs_path  = udev_list_entry_get_name(dev_list_entry);
      uvc_dev     = udev_device_new_from_syspath(udev, sysfs_path);
      dev_path    = udev_device_get_devnode(uvc_dev);

      dev = udev_device_get_parent_with_subsystem_devtype( uvc_dev, "usb", "usb_device");
      const char* strSerial  = udev_device_get_sysattr_value(dev, "serial");

      if((strSerial != NULL) && strcmp(strSerial, serial)==0)
      {
         path = new char[strlen(dev_path)+1];
         strcpy(path, dev_path);
         udev_device_unref(uvc_dev);
         break;
      }

      udev_device_unref(uvc_dev);
   }

   udev_enumerate_unref(enumerate);
   udev_unref(udev);
}

unsigned int UvcCam::getWidth()
{
   return _width;
}

unsigned int UvcCam::getHeight()
{
   return _height;
}

void UvcCam::setColorMode(EnumCameraColorMode mode)
{
   _colorMode = mode;
}

EnumCameraColorMode UvcCam::getColorMode()
{
   return _colorMode;
}

unsigned int UvcCam::getFormat()
{
   return _format;
}

unsigned int UvcCam::getChannels()
{
   if(_format == V4L2_PIX_FMT_YUYV)
   {
      if(_colorMode == CAMRGB)
         return 3;
      else if(_colorMode == CAMGRAYSCALE)
         return 1;
      else
      {
         LOGMSG(DBG_DEBUG, "Unknown color mode");
         return 0;
      }
   }
   else
      return 3;
}

EnumCameraError UvcCam::connect()
{

   if (_nDeviceHandle != -1)
   {
      LOGMSG(DBG_DEBUG, "Device handle already exists. A new connection will not be established.");
      return CAMFAILURE;
   }

   if ((_nDeviceHandle = open(_dev, O_RDWR)) == -1)
   {
      LOGMSG(DBG_DEBUG, "Could not open device V4L2.");
      return CAMERRORINIT;
   }

   struct v4l2_capability cap;
   CLEAR(cap);
   if (ioctl(_nDeviceHandle, VIDIOC_QUERYCAP, &cap) < 0)
   {
      LOGMSG(DBG_DEBUG, "unable to query device.");
      return CAMERRORINIT;
   }
   if ((cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) == 0)
   {
      LOGMSG(DBG_DEBUG, "video capture not supported.");
      return CAMERRORINIT;
   }

   return CAMSUCCESS;
}

EnumCameraError UvcCam::disconnect()
{
   if(_nDeviceHandle != -1)
   {
      stopStreaming();
      close (_nDeviceHandle);
      _nDeviceHandle = -1;
   }
   return CAMSUCCESS;
}

void UvcCam::resetControls()
{
   if (v4l2ResetControl(_nDeviceHandle, V4L2_CID_BRIGHTNESS) < 0) printf("reset Brightness error\n");
   if (v4l2ResetControl(_nDeviceHandle, V4L2_CID_SATURATION) < 0) printf("reset Saturation error\n");
   if (v4l2ResetControl(_nDeviceHandle, V4L2_CID_CONTRAST) < 0)   printf("reset Contrast error\n");
   if (v4l2ResetControl(_nDeviceHandle, V4L2_CID_SHARPNESS) < 0)  printf("reset Sharpness error\n");
   if (v4l2ResetControl(_nDeviceHandle, V4L2_CID_GAIN) < 0)       printf("reset Gain error\n");
}

EnumCameraError UvcCam::setFormat(unsigned int width, unsigned int height, unsigned int format)
{
   if (_nDeviceHandle == -1)
   {
      cout << "UvcCam::setResolution: Trying to set resolution of not initialized camera device." << endl;
      return CAMERRORINIT;
   }

   _format = format;

   struct v4l2_format fmt;
   CLEAR(fmt);
   fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   fmt.fmt.pix.width       = width;
   fmt.fmt.pix.height      = height;
   fmt.fmt.pix.pixelformat = format;
   fmt.fmt.pix.field       = V4L2_FIELD_ANY;
   int retval = ioctl(_nDeviceHandle, VIDIOC_S_FMT, &fmt);
   if (retval < 0)
   {
      LOGMSG(DBG_DEBUG, "Unable to set format.");
      return CAMERRORINIT;
   }
   if ((fmt.fmt.pix.width != width) || (fmt.fmt.pix.height != height))
   {
      LOGMSG(DBG_DEBUG, "desired format unavailable");
      LOGMSG(DBG_DEBUG, "choosing next possible format - width: " << fmt.fmt.pix.width << ", height: " << fmt.fmt.pix.height);
   }
   _width = fmt.fmt.pix.width;
   _height = fmt.fmt.pix.height;

   return CAMSUCCESS;
}

EnumCameraError UvcCam::setFramerate(unsigned int numerator, unsigned int denominator)
{
   if (_nDeviceHandle == -1)
   {
      LOGMSG(DBG_DEBUG, "Trying to set framerate of not initialized camera device.");
      return CAMERRORINIT;
   }

   struct v4l2_streamparm setfps;
   setfps.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   setfps.parm.capture.timeperframe.numerator   = numerator;
   setfps.parm.capture.timeperframe.denominator = denominator;
   if (ioctl(_nDeviceHandle, VIDIOC_S_PARM, &setfps) < 0)
   {
      LOGMSG(DBG_DEBUG, "Unable to set fps");
      return CAMERRORINIT;
   }
   return CAMSUCCESS;
}

EnumCameraError UvcCam::startStreaming()
{
   if (_nDeviceHandle == -1)
   {
      LOGMSG(DBG_DEBUG, "Trying to start streaming of not initialized camera device.");
      return CAMERRORINIT;
   }

   EnumCameraError retval = mapMemory();
   if(retval != CAMSUCCESS) return retval;

   /* Queue the buffers. */
   for (int i = 0; i < _nb_buffers; i++)
   {
      CLEAR(_buf);
      _buf.index = i;
      _buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      _buf.memory = V4L2_MEMORY_MMAP;
      if (ioctl(_nDeviceHandle, VIDIOC_QBUF, &_buf) < 0)
      {
         LOGMSG(DBG_DEBUG, "Unable to queue buffer");
         return CAMERRORINIT;
      }
   }

   //video enable
   int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

   int err = ioctl(_nDeviceHandle, VIDIOC_STREAMON, &type);
   if(err < 0)
   {
      //err = ioctl(_nDeviceHandle, VIDIOC_STREAMON, &type);
      LOGMSG(DBG_DEBUG, "Unable to enable video capture");
      exit(1);
   }

   CLEAR(_buf);
   _buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   _buf.memory = V4L2_MEMORY_MMAP;
   if (ioctl(_nDeviceHandle, VIDIOC_DQBUF, &_buf) < 0)
   {
      LOGMSG(DBG_DEBUG, "Unable to dequeue buffer");
      return CAMFAILURE;
   }

   return CAMSUCCESS;
}

EnumCameraError UvcCam::stopStreaming()
{
   if (_nDeviceHandle == -1)
   {
      LOGMSG(DBG_DEBUG, "Trying to stop streaming of not initialized camera device.");
      return CAMERRORINIT;
   }

   int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   if (ioctl(_nDeviceHandle, VIDIOC_STREAMOFF, &type) < 0)
   {
      LOGMSG(DBG_DEBUG, "Unable to stop streaming");
   }
   unmapMemory();
   return CAMSUCCESS;
}

EnumCameraError UvcCam::grab(unsigned char* image, unsigned int* bytes)
{
   if (_nDeviceHandle == -1)
   {
      LOGMSG(DBG_DEBUG, "Trying to grab image on not initialized camera device.");
      return CAMERRORINIT;
   }

   if (ioctl(_nDeviceHandle, VIDIOC_QBUF, &_buf) < 0)
   {
      LOGMSG(DBG_DEBUG, "Unable to requeue buffer");
      return CAMFAILURE;
   }

   CLEAR(_buf);
   _buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   _buf.memory = V4L2_MEMORY_MMAP;

   if (ioctl(_nDeviceHandle, VIDIOC_DQBUF, &_buf) < 0)
   {
      LOGMSG(DBG_DEBUG, "Unable to dequeue buffer");
      return CAMFAILURE;
   }

   LOGMSG(DBG_DEBUG, "Bytes used: " << _buf.bytesused);

   if(bytes) *bytes = _buf.bytesused;

   if (_buf.bytesused > 0)
   {
      if(_format == V4L2_PIX_FMT_YUYV)
      {
         if(_colorMode==CAMRGB)
            Pyuv422torgb24((unsigned char *) (_mem[_buf.index].start), image, _width, _height);
         else if(_colorMode==CAMGRAYSCALE)
            Pyuv422togray((unsigned char *) (_mem[_buf.index].start), image, _width, _height);
      }
      else
         memcpy(image, _mem[_buf.index].start, _buf.bytesused + DHT_SIZE);
      return CAMSUCCESS;
   }

   return CAMGRABBING;
}

EnumCameraError UvcCam::printAvailableFormats()
{

   EnumCameraError retval = connect();
   if(retval!=CAMSUCCESS) return retval;

   int ret;

   struct v4l2_fmtdesc desc;
   CLEAR(desc);
   desc.index = 0;
   desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   while ((ret = ioctl(_nDeviceHandle, VIDIOC_ENUM_FMT, &desc)) == 0)
   {
      desc.index++;
      printf("pixelformat = '%c%c%c%c', description = '%s'\n",
            desc.pixelformat & 0xFF,
            (desc.pixelformat >> 8) & 0xFF,
            (desc.pixelformat >> 16) & 0xFF,
            (desc.pixelformat >> 24) & 0xFF,
            desc.description);
   }

   struct v4l2_format fmt;
   unsigned int res[32] = { 90,  120,  144,  160,  176,  240,  272,  288,  320,  352,
         480,  544,  576,  600,  640,  704,  720,  768,  800,  960,
         1024, 1080, 1152, 1280, 1366, 1400, 1440, 1600, 1900, 1920,
         2048, 2560 };

   CLEAR(fmt);
   fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
   fmt.fmt.pix.field       = V4L2_FIELD_ANY;

   cout << "supported resolutions" << endl;
   // some combinations are not meaningful, but for a simple implementation the brute force check is sufficient.
   for(int i=0; i<32; i++)
   {
      for(int j=0; j<32; j++)
      {
         fmt.fmt.pix.width       = res[i];
         fmt.fmt.pix.height      = res[j];
         ret = ioctl(_nDeviceHandle, VIDIOC_S_FMT, &fmt);
         if(ret==0)
         {
            if ((fmt.fmt.pix.width == res[i]) && (fmt.fmt.pix.height == res[j]))
            {
               cout << res[i] << "x" << res[j] << endl;
            }
         }
         else
         {
            cout << "unable to query format" << endl;
         }
      }
   }

   disconnect();

   return CAMSUCCESS;
}

EnumCameraError UvcCam::mapMemory()
{
   CLEAR(_rb);
   _rb.count  = _nb_buffers;
   _rb.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
   _rb.memory = V4L2_MEMORY_MMAP;
   if (ioctl(_nDeviceHandle, VIDIOC_REQBUFS, &_rb) < 0)
   {
      LOGMSG(DBG_DEBUG, "Unable to allocate buffers .");
      return CAMERRORINIT;
   }
   _mem = (buffer *) calloc(_nb_buffers, sizeof(*_mem));

   // map the buffers
   for (int i = 0; i < _nb_buffers; i++)
   {
      CLEAR(_buf);
      _buf.index = i;
      _buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      _buf.memory = V4L2_MEMORY_MMAP;
      if (ioctl(_nDeviceHandle, VIDIOC_QUERYBUF, &_buf) < 0)
      {
         LOGMSG(DBG_DEBUG, "Unable to query buffer");
         return CAMERRORINIT;
      }
      _mem[i].length = _buf.length;
      _mem[i].start = mmap(0 , _buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, _nDeviceHandle, _buf.m.offset);
      if (_mem[i].start == MAP_FAILED)
      {
         LOGMSG(DBG_DEBUG, "Unable to map buffer");
         return CAMERRORINIT;
      }
   }
   return CAMSUCCESS;
}

void UvcCam::unmapMemory()
{
   if(_mem!=NULL)
   {
      for(unsigned int i = 0; i < _rb.count; i++)
      {
         munmap(_mem[i].start, _mem[i].length);
      }
   }
}

// aus lucview color.c util.c
unsigned int UvcCam::Pyuv422torgb24(unsigned char * input_ptr, unsigned char * output_ptr, unsigned int image_width, unsigned int image_height)
{
   unsigned int i, size;
   unsigned char Y, Y1, U, V;
   unsigned char *buff = input_ptr;
   unsigned char *output_pt = output_ptr;
   size = image_width * image_height / 2;
   for (i = size; i > 0; i--)
   {
      /* bgr instead rgb ?? */
      Y  = buff[0];
      U  = buff[1];
      Y1 = buff[2];
      V  = buff[3];
      buff += 4;
      *output_pt++ = R_FROMYV(Y,V);
      *output_pt++ = G_FROMYUV(Y,U,V); //b
      *output_pt++ = B_FROMYU(Y,U); //v

      *output_pt++ = R_FROMYV(Y1,V);
      *output_pt++ = G_FROMYUV(Y1,U,V); //b
      *output_pt++ = B_FROMYU(Y1,U); //v
   }

   return 1;
}

unsigned int UvcCam::Pyuv422togray(unsigned char* input_ptr, unsigned char* output_ptr, unsigned int image_width, unsigned int image_height)
{
   unsigned int i, size;
   unsigned char Y, Y1;
   unsigned char *buff = input_ptr;
   unsigned char *output_pt = output_ptr;
   size = image_width * image_height / 2;
   for (i = size; i > 0; i--)
   {
      Y  = buff[0];
      Y1 = buff[2];
      buff += 4;
      *output_pt++ = Y;
      *output_pt++ = Y1;
   }

   return 1;
}

/* return >= 0 ok otherwhise -1 */
int UvcCam::isv4l2Control(int nHandle, int control, struct v4l2_queryctrl *queryctrl)
{
   int err = 0;
   queryctrl->id = control;
   if ((err = ioctl(nHandle, VIDIOC_QUERYCTRL, queryctrl)) < 0)
   {
      LOGMSG(DBG_DEBUG, "ioctl querycontrol error: " << errno);
   }
   else if (queryctrl->flags & V4L2_CTRL_FLAG_DISABLED)
   {
      LOGMSG(DBG_DEBUG, "control " << (char *) queryctrl->name << " disabled");
   }
   else if (queryctrl->flags & V4L2_CTRL_TYPE_BOOLEAN)
   {
      return 1;
   }
   else if (queryctrl->type & V4L2_CTRL_TYPE_INTEGER)
   {
      return 0;
   }
   else
   {
      LOGMSG(DBG_DEBUG, "contol " << (char *) queryctrl->name << " unsupported");
   }
   return -1;
}

int UvcCam::v4l2GetControl(int nHandle, int control)
{
   struct v4l2_queryctrl queryctrl;
   struct v4l2_control control_s;
   int err;
   if (isv4l2Control(nHandle, control, &queryctrl) < 0) return -1;
   control_s.id = control;
   if ((err = ioctl(nHandle, VIDIOC_G_CTRL, &control_s)) < 0)
   {
      LOGMSG(DBG_DEBUG, "ioctl get control error");
      return -1;
   }
   return control_s.value;
}

int UvcCam::v4l2SetControl(int nHandle, int control, int value)
{
   struct v4l2_control control_s;
   struct v4l2_queryctrl queryctrl;
   int err;
   if (isv4l2Control(nHandle, control, &queryctrl) < 0) return -1;
   int min     = queryctrl.minimum;
   int max     = queryctrl.maximum;

   if ((value >= min) && (value <= max))
   {
      control_s.id = control;
      control_s.value = value;
      if ((err = ioctl(nHandle, VIDIOC_S_CTRL, &control_s)) < 0)
      {
         LOGMSG(DBG_DEBUG, "ioctl set control error");
         return -1;
      }
   }
   return 0;
}

int UvcCam::v4l2ResetControl(int nHandle, int control)
{
   struct v4l2_control control_s;
   struct v4l2_queryctrl queryctrl;
   int val_def;
   int err;
   if (isv4l2Control(nHandle, control, &queryctrl) < 0) return -1;
   val_def         = queryctrl.default_value;
   control_s.id    = control;
   control_s.value = val_def;
   if ((err = ioctl(nHandle, VIDIOC_S_CTRL, &control_s)) < 0)
   {
      LOGMSG(DBG_DEBUG, "ioctl reset control error");
      return -1;
   }

   return 0;
}

}

