#ifndef GLFW_STUFF_H
#define GLFW_STUFF_H

#ifdef _WIN32//for glut.h
#include <windows.h>
#endif

//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/OpenGL.h>
#include <OpenGL/gl.h>
//#include <OpenGL/glu.h>
//#include <GLUT/glut.h>
#else


#ifdef _WINDOWS
#include <windows.h>
#include <GL/gl.h>
//#include <GL/glu.h>
#else
#include <GL/gl.h>
//#include <GL/glut.h>
#endif //_WINDOWS
#endif //APPLE

#include <GLFW/glfw3.h>

#if defined(BT_USE_DOUBLE_PRECISION)
#define btglLoadMatrix glLoadMatrixd
#define btglMultMatrix glMultMatrixd
#define btglColor3 glColor3d
#define btglVertex3 glVertex3d
#else
#define btglLoadMatrix glLoadMatrixf
#define btglMultMatrix glMultMatrixf
#define btglColor3 glColor3f
#define btglVertex3 glVertex3d
#endif

class DemoApplication;

int glfwmain(int argc, char **argv, int width, int height, const char* title, DemoApplication* demoApp);

#endif //GLFW_STUFF_H
