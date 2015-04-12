/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef WORLD_H
#define WORLD_H
#ifdef __cplusplus
extern "C" {
#endif
// -----------------------------------------------------------------------------

#include "team.h"
#include "vector.h"

struct FieldGeometry;

struct World *new_world(const struct FieldGeometry *field);
struct World *clone_world(const struct World *world);
void delete_world(struct World *world);

void world_step(struct World *world, Float time_step, int max_substeps,
                Float fixed_time_step);

const struct FieldGeometry *world_get_field(const struct World *world);

struct Ball *world_get_ball(struct World *world);

int world_robot_count(struct World *world);
struct Robot *world_get_robot(struct World *world, int index);
void world_add_robot(struct World *world, int id, enum Team team);

struct btDiscreteDynamicsWorld *world_bt_dynamics(struct World *world);

// -----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif
