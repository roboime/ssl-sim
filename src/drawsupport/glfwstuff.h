/*
 * Small Size League Simulator (Experimental) (c) Jan Segre 2015
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.  Permission is granted to anyone to use this software
 * for any purpose, including commercial applications, and to alter it and
 * redistribute it freely, subject to the following restrictions:
 */

#ifndef GLFW_STUFF_H
#define GLFW_STUFF_H

#if defined(__APPLE__) && !defined(VMDMESA)
// think different
#include <OpenGL/OpenGL.h>
#include <OpenGL/gl.h>
#elif defined(_WINDOWS) || defined(_WIN32)
#include <windows.h>
#include <GL/gl.h>
#else
#include <GL/gl.h>
#endif

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

int glfwmain(int argc, char **argv, int width, int height, const char *title,
             DemoApplication *demoApp);

#endif // GLFW_STUFF_H
