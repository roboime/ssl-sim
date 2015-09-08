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
#include <time.h>

#include "sslsim.h"
#include "serialize.h"
#include "utils/net.h"

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
void sigint_handler(int sig) {
  fprintf(stderr, "\rCtrl+C pressed, closing...\n");
  keep_going = false;
}

// int main(int argc, char *argv[]) {
int main(void) {
  const char *title = "Small Size League Simulator by RoboIME";
  puts(title);

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

  // Add some robots
  for (int i = 0; i < 6; i++) {
    world_add_robot(world, i, TEAM_BLUE);
    world_add_robot(world, i, TEAM_YELLOW);
  }

  // Create and bind socket
  struct Socket *socket = new_socket(11002, "224.5.23.2");
  socket_sender_bind(socket);

#define BUFFER_SIZE 10240
  char buffer[BUFFER_SIZE];
  int send_size;
  int send_geometry = 0;

  while (keep_going) {
    world_step(world);

    // TODO: log errors
    send_size = serialize_world(world, buffer, BUFFER_SIZE);
    if (send_size > 0)
      socket_send(socket, buffer, send_size);

    if (send_geometry++ % 120 == 0) {
      send_size = serialize_field(world_get_field(world), buffer, BUFFER_SIZE);
      if (send_size > 0)
        socket_send(socket, buffer, send_size);
    }

    // TODO: realtime update
    // sleep for 16ms, prevents going too fast and intensive
    nanosleep(&(struct timespec){.tv_nsec=16000000}, NULL);
  }

  // Clean it
  delete_socket(socket);
  // FIXME: world is not freeing correctly:
  //delete_world(world);

  return 0;
}
