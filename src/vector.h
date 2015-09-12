/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef VECTOR_H
#define VECTOR_H
#ifdef __cplusplus
extern "C" {
#endif
// -----------------------------------------------------------------------------

#ifdef DOUBLE_PRECISON
typedef double Float;
#else
typedef float Float;
#endif

struct Vec2 {
  Float x, y;
};

struct Vec3 {
  Float x, y, z;
};

struct Pos2 {
  Float x, y, w;
};

struct Pos3 {
  Float x, y, z, wx, wy, wz;
};
Float scalar_multiplication2(struct Vec3 v1, struct Vec3 v2);
struct Vec3 multiplication_by_scalar2(struct Vec3 v, Float f);
// -----------------------------------------------------------------------------
#ifdef __cplusplus

}
#endif
#endif
