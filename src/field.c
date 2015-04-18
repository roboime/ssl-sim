/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "field.h"

const struct FieldGeometry FIELD_2014_SINGLE = {
  0.010, // line_width;
  6.050, // field_length;
  4.050, // field_width;
  0.250, // boundary_width;
  0.425, // referee_width;
  0.700, // goal_width;
  0.180, // goal_depth;
  0.020, // goal_wall_width;
  0.500, // center_circle_radius;
  0.800, // defense_radius;
  0.350, // defense_stretch;
  0.200, // free_kick_from_defense_dist;
  0.750, // penalty_spot_from_field_line_dist;
  0.400, // penalty_line_from_spot_dist;
  0.165  // goal_height;
};

const struct FieldGeometry FIELD_2014_DOUBLE = {
  0.010, // line_width;
  8.090, // field_length;
  6.050, // field_width;
  0.250, // boundary_width;
  0.425, // referee_width;
  1.000, // goal_width;
  0.180, // goal_depth;
  0.020, // goal_wall_width;
  0.500, // center_circle_radius;
  1.000, // defense_radius;
  0.500, // defense_stretch;
  0.200, // free_kick_from_defense_dist;
  1.000, // penalty_spot_from_field_line_dist;
  0.400, // penalty_line_from_spot_dist;
  0.165  // goal_height;
};

const struct FieldGeometry FIELD_2015 = {
  0.010, // line_width;
  9.000, // field_length;
  6.000, // field_width;
  0.250, // boundary_width;
  0.450, // referee_width;
  1.000, // goal_width;
  0.180, // goal_depth;
  0.020, // goal_wall_width;
  0.500, // center_circle_radius;
  1.000, // defense_radius;
  0.500, // defense_stretch;
  0.200, // free_kick_from_defense_dist;
  1.000, // penalty_spot_from_field_line_dist;
  0.400, // penalty_line_from_spot_dist;
  0.165  // goal_height;
};
