/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "ugly_test.h"
#include "glfwstuff.h"
#include "btBulletDynamicsCommon.h"

int main(int argc, char **argv) {
  SSLSim sslsim;
  sslsim.initPhysics();
  return glfwmain(argc, argv, 1024, 600, "SSL Sim GUI Demo", &sslsim);
}
