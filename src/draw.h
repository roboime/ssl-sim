/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef DRAW_H
#define DRAW_H

void draw_init(struct World *world);
void draw_world(void);
void draw_debug(void);
void draw_set_debug_mode(int mode);
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
void draw_mouse_projection(int button, bool drag);
void draw_options_window(void);

extern float draw_screen_x;
extern float draw_screen_y;
extern float draw_screen_width;
extern float draw_screen_height;
extern float projected_mouse_x;
extern float projected_mouse_y;

#endif
