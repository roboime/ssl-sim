/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef FIELD_H
#define FIELD_H
#ifdef __cplusplus
extern "C" {
#endif
// -----------------------------------------------------------------------------

#include "vector.h"

struct FieldGeometry {
  Float line_width;
  Float field_length;
  Float field_width;
  Float boundary_width;
  Float referee_width;
  Float goal_width;
  Float goal_depth;
  Float goal_wall_width;
  Float center_circle_radius;
  Float defense_radius;
  Float defense_stretch;
  Float free_kick_from_defense_dist;
  Float penalty_spot_from_field_line_dist;
  Float penalty_line_from_spot_dist;
};

inline Float field_limit_x(const struct FieldGeometry *field) {
  return field->field_length / 2 + field->boundary_width + field->referee_width;
}

inline Float field_limit_y(const struct FieldGeometry *field) {
  return field->field_width / 2 + field->boundary_width + field->referee_width;
}

extern const struct FieldGeometry FIELD_2014_SINGLE;
extern const struct FieldGeometry FIELD_2014_DOUBLE;
extern const struct FieldGeometry FIELD_2015;

// -----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif
