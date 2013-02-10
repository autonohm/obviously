#include "Obvious2D.h"

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <jpeglib.h>
#include <linux/videodev2.h>
#define GL_GLEXT_PROTOTYPES 1
#include <GL/freeglut.h>
#include "obcore/math/mathbase.h"

#include <map>

using namespace std;

namespace obvious
{

bool _isAlive = true;

void onCloseEvent()
{
   _isAlive = false;
}

/**
 * keyboard handler
 **/
std::map<char, fptrKeyboardCallback> _mCallback;
void keyboard(unsigned char key, int x, int y)
{
   fptrKeyboardCallback fptr = _mCallback[(char)key];
   if(fptr!=NULL)(*fptr)();
}

Obvious2D::Obvious2D(unsigned int width, unsigned int height, const char* title)
{
   char* szDummy = (char*)"dummy";
   int nArgc     = 1;
   glutInit(&nArgc,&szDummy);

   glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_ACCUM);
   glutInitWindowSize(width, height);
   glutInitWindowPosition(0, 0);

   _handle = glutCreateWindow(title);
   _width  = width;
   _height = height;

   glDisable(GL_ALPHA_TEST);
   glDisable(GL_DEPTH_TEST);
   glDisable(GL_BLEND);
   glDisable(GL_DITHER);
   glDisable(GL_FOG);
   glDisable(GL_LIGHTING);
   glDisable(GL_LOGIC_OP);
   glDisable(GL_STENCIL_TEST);
   glDisable(GL_TEXTURE_1D);

   glRasterPos2f(-1.0, 1.0);
   glPixelZoom(1.0, -1.0);

   glutCloseFunc (onCloseEvent);
   glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);

   glutKeyboardFunc(keyboard);

   glutShowWindow();

   for(int i=0; i<TEXTMAX; i++)
      _text[i].text[0] = '\0';

   _textCnt = 0;
}

Obvious2D::~Obvious2D()
{

}

bool Obvious2D::isAlive()
{
   return _isAlive;
}

void Obvious2D::draw(unsigned char* image, unsigned int width, unsigned int height, unsigned int channels, unsigned int jpeg, unsigned int bytes)
{
   if(!_isAlive) return;

   if(jpeg)
   {
      struct jpeg_decompress_struct cinfo;
      struct jpeg_error_mgr jerr;
      unsigned char* img_temp = new unsigned char[width*height*channels];

      cinfo.err = jpeg_std_error (&jerr);
      jpeg_create_decompress (&cinfo);

      jpeg_mem_src (&cinfo, image, bytes);
      jpeg_read_header (&cinfo, TRUE);
      if ((cinfo.image_width != width) || (cinfo.image_height != height))
         return;

      jpeg_start_decompress (&cinfo);

      int row_stride = cinfo.output_width * cinfo.output_components;
      JSAMPARRAY buffer = (*cinfo.mem->alloc_sarray)((j_common_ptr) &cinfo, JPOOL_IMAGE, row_stride, 1);
      int idx = 0;
      while (cinfo.output_scanline < cinfo.output_height)
      {
         jpeg_read_scanlines (&cinfo, buffer, 1);
         memcpy(&img_temp[idx], buffer[0], row_stride);
         idx+=row_stride;
      }
      jpeg_finish_decompress (&cinfo);
      jpeg_destroy_decompress (&cinfo);

      drawDefault(img_temp, width, height, channels);
      delete [] img_temp;
   }
   else
      drawDefault(image, width, height, channels);

   glutSwapBuffers();
   glutMainLoopEvent();
}

void Obvious2D::registerKeyboardCallback(char key, fptrKeyboardCallback callback)
{
   _mCallback[key] = callback;
}

void Obvious2D::addText(char* text, unsigned int c, unsigned int r)
{
   if(_textCnt >= TEXTMAX)
      return;
   unsigned int size = strlen(text);
   if(size>256) size = 256;
   memcpy(_text[_textCnt].text, text, size*sizeof(*_text[_textCnt].text));
   _text[_textCnt].col = c;
   _text[_textCnt].row = r;
   _textCnt++;
}

void Obvious2D::clearText(void)
{
   _textCnt = 0;
}

void Obvious2D::addRect(unsigned int rUpLeft, unsigned int cUpLeft, unsigned int xDim, unsigned int yDim)
{
   float x   = (float)xDim/(float)_height;
   float y   = (float)yDim/(float)_width;
   float rUL = (float)(rUpLeft-(double)_height)/(float)_height;
   float cUL = (float)(cUpLeft-(double)_width)/(float)_width;
   float rDL = rUL;
   float cDL = cUL - x;
   glBegin(GL_LINES);
   glColor3f(.15,.15,.15);
   // verticals
   glVertex2f(rUL, cUL);
   glVertex2f(rDL, cDL);
   glVertex2f(rUL+y, cUL);
   glVertex2f(rDL+y, cDL);
   // horizontals
   glVertex2f(rUL, cUL);
   glVertex2f(rUL+y, cUL);
   glVertex2f(rDL, cDL);
   glVertex2f(rDL+y, cDL);

   glEnd();
}

void Obvious2D::addCircle(unsigned int rCenter, unsigned int cCenter, unsigned int radius)
{
   const int sides = 36;  // The amount of segment to create the circle
   double r = (double)radius/_height;
   glColor3f(.15,.15,.15);
   glBegin(GL_LINE_LOOP);
   for (int a=0; a<360; a+=360 / sides)
   {
      double heading = a * M_PI / 180;
      glVertex2f(cos(heading)*r + (double)(rCenter-(double)_height)/(double)_height,
            sin(heading)*r + (double)(cCenter-(double)_width) /(double)_width);
   }
   glEnd();
}


void Obvious2D::drawDefault(unsigned char* image, unsigned int width, unsigned int height, unsigned int channels)
{
   float ratioW = (float)_width/width;
   float ratioH = (float)_height/height;

   glPixelZoom(ratioW, -ratioH);

   if(channels==1)
      glDrawPixels(width, height, GL_LUMINANCE,GL_UNSIGNED_BYTE, image);
   else if(channels==3)
      glDrawPixels(width, height, GL_RGB,GL_UNSIGNED_BYTE, image);
   else
      cout << "WARNING: draw method not implemented for channels=" << channels << endl;

   for(unsigned int j=0; j<_textCnt; j++)
   {
      if(strlen(_text[j].text)>0)
      {
         glPushMatrix();
         // Flip vertically for correct text display
         glPixelZoom(ratioW, ratioH);
         glColor3f(0.0, 1.0, 0.0);
         glWindowPos2i(_text[j].col, _text[j].row);
         for (unsigned int i=0; i<strlen(_text[j].text); i++)
            glutBitmapCharacter(GLUT_BITMAP_9_BY_15, (int)_text[j].text[i]);
         glRasterPos2f(-1.0, 1.0);
         glPopMatrix();
      }
   }
}

} // namespace
