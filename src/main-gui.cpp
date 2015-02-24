#include "sslsim.h"
#ifdef USE_GLUT
#include "GlutStuff.h"
#else
#include "glfwstuff.h"
#endif
#include "btBulletDynamicsCommon.h"


int main(int argc,char** argv) {
  SSLSim sslsim;
  sslsim.initPhysics();
#ifdef USE_GLUT
  return glutmain(argc, argv, 1024, 600, "SSL Sim GUI Demo", &sslsim);
#else
  return glfwmain(argc, argv, 1024, 600, "SSL Sim GUI Demo", &sslsim);
#endif
}
