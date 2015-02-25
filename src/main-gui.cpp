#include "sslsim.h"
#include "glfwstuff.h"
#include "btBulletDynamicsCommon.h"


int main(int argc,char** argv) {
  SSLSim sslsim;
  sslsim.initPhysics();
  return glfwmain(argc, argv, 1024, 600, "SSL Sim GUI Demo", &sslsim);
}
