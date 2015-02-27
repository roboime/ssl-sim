/*
 * Small Size League Simulator (Experimental) (c) Jan Segre 2015
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.  Permission is granted to anyone to use this software
 * for any purpose, including commercial applications, and to alter it and
 * redistribute it freely, subject to the following restrictions:
 */

#include "sslsim.h"
#include "glfwstuff.h"
#include "btBulletDynamicsCommon.h"

int main(int argc, char **argv) {
  SSLSim sslsim;
  sslsim.initPhysics();
  return glfwmain(argc, argv, 1024, 600, "SSL Sim GUI Demo", &sslsim);
}
