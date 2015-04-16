/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef SERIALIZE_H
#define SERIALIZE_H
#ifdef __cplusplus
extern "C" {
#endif
// -----------------------------------------------------------------------------

// -1 on error or the size written
int serialize_world(const struct World *world, char *buffer, int buffer_size);
int serialize_field(const struct FieldGeometry *field, char *buffer,
                    int buffer_size);

// -----------------------------------------------------------------------------
#ifdef __cplusplus
}
#endif
#endif
