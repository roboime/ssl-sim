/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
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
