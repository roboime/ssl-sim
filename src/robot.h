/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef ROBOT_H
#define ROBOT_H
#ifdef __cplusplus
extern "C" {
#endif
// -----------------------------------------------------------------------------

#include "vector.h"
#include "team.h"

int get_id(struct Robot *robot);
enum Team get_team(struct Robot *robot);

struct Pos2 robot_get_pos(struct Robot *robot);
void robot_set_pos(struct Robot *robot, const struct Pos2 pos);

struct Pos2 robot_get_vel(struct Robot *robot);
void robot_set_vel(struct Robot *robot, const struct Pos2 vel);

/// return is C bool (1 for true, 0 for false)
int robot_is_touching_robot(struct Robot *robot, struct Robot *tobor);

/// fast squared speed (magnitude of velocity)
Float robot_get_speed2(struct Robot *robot);

// -----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif
