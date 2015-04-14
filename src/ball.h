/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef BALL_H
#define BALL_H
#ifdef __cplusplus
extern "C" {
#endif
// -----------------------------------------------------------------------------

#include "vector.h"

struct Vec3 ball_get_vec(struct Ball *ball);
void ball_set_vec(struct Ball *ball, const struct Vec2 vec);

struct Pos3 ball_get_pos(struct Ball *ball);
void ball_set_pos(struct Ball *ball, const struct Pos3 pos);

struct Pos3 ball_get_vel(struct Ball *ball);
void ball_set_vel(struct Ball *ball, const struct Pos3 vel);

// TODO: ergonomic functions for kicks and dribbles

/// return is C bool (1 for true, 0 for false)
int ball_is_touching_robot(struct Ball *ball, struct Robot *robot);

struct Robot *ball_last_touching_robot(struct Ball *ball);

/// fast squared speed (magnitude of velocity)
Float ball_get_speed2(struct Ball *ball);

/// fast squared speed (magnitude of velocity)
Float ball_get_peak_speed2_from_last_kick(struct Ball *ball);

struct btRigidBody *ball_bt_rigid_body(struct Ball *ball);

// -----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif
