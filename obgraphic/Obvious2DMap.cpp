/**
* @file   Obvious2DMap.cpp
* @author Christian Pfitzner
* @date   12.01.2013
*
*
*/

// includes of obviously
#include "obgraphic/Obvious2DMap.h"
#include "obcore/math/mathbase.h"
// includes of glut
#define GL_GLEXT_PROTOTYPES 1
#include <GL/freeglut.h>


using namespace obvious;

void Obvious2DMap::draw(unsigned char* image, unsigned int width, unsigned int height, unsigned int channels)
{
  if(_gridActive)
    drawGrid();

  if(_circleActive)
    drawCircle();

  if(_anglesActive)
    drawAngles();

  drawCenter();
  drawDefault(image, width, height, channels);
}


void Obvious2DMap::drawGrid(void) const
{
  glColor3f(.15,.15,.15);
  glBegin(GL_LINES);
  for(int i=-10 ; i<10 ; i+=2)
  {
    // horizontal lines
    glVertex3f(-1,i*0.1,0);
    glVertex3f(1,i*0.1,0);
    // vertical lines
    glVertex3f(i*0.1,-1,0);
    glVertex3f(i*0.1,1,0);
  }
  glEnd();
}

void Obvious2DMap::drawCircle(void) const
{
  const int sides = 36;  // The amount of segment to create the circle
  float step = 1/(_lengthMap);
  unsigned int i=1;
  for(float r=step ; r<_lengthMap ; r+=step)
  {
    if (i%2 == 0)
      glColor3f(.3,.3,.3);
    else
      glColor3f(.08,.08,.08);

    glBegin(GL_LINE_LOOP);
    for (int a = 0; a < 360; a += 360 / sides)
    {
      double heading = a * M_PI / 180;
      glVertex2d(cos(heading)*r, sin(heading)*r);
    }
    glEnd();
    i++;
  }
}


void Obvious2DMap::drawAngles(const float angleStep) const
{
  glBegin(GL_LINES);
  for(int i=0 ; i<360 ; i+=angleStep)
  {
    if (i%90 == 0)
      glColor3f(.3,.3,.3);
    else
      glColor3f(.08,.08,.08);
    double heading = i * M_PI / 180;
    glVertex2d(cos(heading)*2, sin(heading)*2);
    glVertex2d(0,0);
  }
  glEnd();
}

void Obvious2DMap::drawCenter(void) const
{
  glBegin(GL_TRIANGLES);
  glColor3f(.6,0,0);
  glVertex2f(0,0.03);
  glVertex2f(-0.01, -0.03);
  glVertex2f(0.01, -0.03);
  glEnd();
  glColor3f(.15,.15,.15);

  glutSwapBuffers();
  glutMainLoopEvent();
}








