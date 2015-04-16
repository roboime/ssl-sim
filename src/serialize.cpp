/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "serialize.h"

#include "sslsim.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include <chrono>

extern "C" {

int serialize_world(const World *world, char *buffer, int buffer_size) {
  SSL_WrapperPacket wrapper{};

  auto detection = wrapper.mutable_detection();
  detection->set_frame_number(world_get_frame_number(world));
  detection->set_t_capture(world_get_timestamp(world));
  detection->set_camera_id(0);

  int ball_count = world_ball_count(world);
  for (int i = 0; i < ball_count; i++) {
    auto ball = world_get_ball(world, i);
    auto ball_vec = ball_get_vec(ball);
    // TODO: maybe check if ball is inside working area
    auto detection_ball = detection->add_balls();
    detection_ball->set_confidence(1.0);
    detection_ball->set_x(ball_vec.x);
    detection_ball->set_y(ball_vec.y);
    detection_ball->set_z(ball_vec.z);
    // TODO: check whether this values actually make sense
    detection_ball->set_pixel_x(ball_vec.x);
    detection_ball->set_pixel_y(ball_vec.y);
  }

  int robot_count = world_robot_count(world);
  for (int i = 0; i < robot_count; i++) {
    auto robot = world_get_robot(world, i);
    auto robot_pos = robot_get_pos(robot);
    // TODO: maybe check if robot is inside working area
    SSL_DetectionRobot *detection_robot;
    switch (get_team(robot)) {
    case TEAM_BLUE:
      detection_robot = detection->add_robots_blue();
      break;
    case TEAM_YELLOW:
      detection_robot = detection->add_robots_yellow();
      break;
    case TEAM_NONE:
      continue;
    }
    detection_robot->set_confidence(1.0);
    detection_robot->set_robot_id(get_id(robot));
    detection_robot->set_x(robot_pos.x);
    detection_robot->set_y(robot_pos.y);
    detection_robot->set_orientation(robot_pos.w);
    // TODO: check whether this values actually make sense
    detection_robot->set_pixel_x(robot_pos.x);
    detection_robot->set_pixel_y(robot_pos.y);
    // TODO: send height maybe?
  }

  // XXX: delayed as much as possible for more accuracy
  using namespace std::chrono;
  duration<double> timestamp = high_resolution_clock::now().time_since_epoch();
  detection->set_t_sent(timestamp.count());

  if (wrapper.SerializeToArray(buffer, buffer_size))
    return wrapper.ByteSize();

  return -1;
}

int serialize_field(const FieldGeometry *field, char *buffer, int buffer_size) {
  SSL_WrapperPacket wrapper{};

  auto field_geometry = wrapper.mutable_geometry()->mutable_field();
  field_geometry->set_line_width(field->line_width);
  field_geometry->set_field_length(field->field_length);
  field_geometry->set_field_width(field->field_width);
  field_geometry->set_boundary_width(field->boundary_width);
  field_geometry->set_referee_width(field->referee_width);
  field_geometry->set_goal_width(field->goal_width);
  field_geometry->set_goal_depth(field->goal_depth);
  field_geometry->set_goal_wall_width(field->goal_wall_width);
  field_geometry->set_center_circle_radius(field->center_circle_radius);
  field_geometry->set_defense_radius(field->defense_radius);
  field_geometry->set_defense_stretch(field->defense_stretch);
  field_geometry->set_free_kick_from_defense_dist(
      field->free_kick_from_defense_dist);
  field_geometry->set_penalty_spot_from_field_line_dist(
      field->penalty_spot_from_field_line_dist);
  field_geometry->set_penalty_line_from_spot_dist(
      field->penalty_line_from_spot_dist);

  if (wrapper.SerializeToArray(buffer, buffer_size))
    return wrapper.ByteSize();

  return -1;
}

} // end extern
