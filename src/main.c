/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#define _XOPEN_SOURCE 700
// http://stackoverflow.com/questions/6491019/struct-sigaction-incomplete-error
#include <signal.h>
#undef _XOPEN_SOURCE
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include "sslsim.h"

//
// How To Scale The World
// ======================
//
// Obviously any collision shapes must be scaled appropriately,
// but there are also some other things that need to be changed.
// In general, if you are scaling the world by a factor of X
// then you must do the following:
//
// - Scale collision shapes about origin by X
// - Scale all positions by X
// - Scale all linear (but not angular) velocities by X
// - Scale linear [Sleep Threshold] by X
// - Scale gravity by X
// - Scale all impulses you supply by X
// - Scale all torques by X^2
// - Scale all inertias by X if not computed by Bullet
//

bool keep_going = true;
void sigint_handler(int sig);

// int main(int argc, char *argv[]) {
int main(void) {
  printf("Hello World!\n");

  {
    // Handle SIGINT (Ctrl+C)
    struct sigaction sa;
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, NULL);
  }

  // Create a world
  struct World *world = new_world(&FIELD_2015);

  // Step the world a few times
  for (int i = 0; i < 30; i++) {
    world_step(world, 1.0 / 60, 10, 1.0 / 600);
    struct Ball *ball = world_get_ball(world);
    struct Vec3 vec = ball_get_vec(ball);
    printf("ball: (%f, %f, %f)\n", vec.x, vec.y, vec.z);
  }

  // while (keep_going) {
  //}

  // Clean it
  delete_world(world);

  printf("Done.\n");
  return 0;
}

void sigint_handler(int sig) {
  fprintf(stderr, "Aborting...\n");
  keep_going = false;
}
