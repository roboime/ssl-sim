/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DRAW_H
#define DRAW_H

void draw_set_debug_mode(int mode);
void draw_physics_world(struct World *world);
void draw_rot_left(void);
void draw_rot_right(void);
void draw_rot_up(void);
void draw_rot_down(void);
void draw_zoom_in(void);
void draw_zoom_out(void);
void draw_walk_left(void);
void draw_walk_right(void);
void draw_walk_front(void);
void draw_walk_back(void);
void draw_set_screen_size(int width, int height);
void draw_set_screen_pos(double x, double y);
void draw_set_screen_active(bool active);
void draw_set_screen_drag(int button);

#endif
