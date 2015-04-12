/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "draw.h"
#include "../world.h"
#include "../field.h"
#include "GL_ShapeDrawer.h"

#include <GLFW/glfw3.h>
#include <btBulletDynamicsCommon.h>
#include "imgui.h"

#include "glutils.h"
#include "colors.h"

// Utils
template <typename T> constexpr T RAD(T D) { return M_PI * D / 180.; }
template <typename T> constexpr T DEG(T R) { return 180. * R / M_PI; }
template <typename T> constexpr T SQ(T X) { return X * X; }

static int debug_mode{};
void draw_set_debug_mode(int mode) { debug_mode = mode; }

void draw_physics_world_pass(World *world, int pass) {
  static GL_ShapeDrawer shape_drawer{};
  static btVector3 sun_direction{btVector3(1, 1, -2) * 1000};

  btDiscreteDynamicsWorld *dynamics = world_bt_dynamics(world);
  btScalar m[16];
  btMatrix3x3 rot;

  const int num_objects = dynamics->getNumCollisionObjects();
  for (int i = 0; i < num_objects; i++) {
    btCollisionObject *col_obj = dynamics->getCollisionObjectArray()[i];
    btRigidBody *body = btRigidBody::upcast(col_obj);
    if (body && body->getMotionState()) {
      auto motion_state = (btDefaultMotionState *)body->getMotionState();
      motion_state->m_graphicsWorldTrans.getOpenGLMatrix(m);
      rot = motion_state->m_graphicsWorldTrans.getBasis();
    } else {
      col_obj->getWorldTransform().getOpenGLMatrix(m);
      rot = col_obj->getWorldTransform().getBasis();
    }

#if 0
    // wants deactivation
    auto wire_color = i & 1? btVector3{0.0, 0.0, 1.0} : btVector3{1.0, 1.0, 0.5};
    // active
    if (col_obj->getActivationState() == 1)
      wire_color += i & 1? btVector3{1.0, 0.0, 0.0} : btVector3{0.5, 0.0, 0.0};
    // island sleeping
    if (col_obj->getActivationState() == 2)
      wire_color += i & 1? btVector3{0.0, 1.0, 0.0} : btVector3{0.0, 0.5, 0.0};
#else
    btVector3 wire_color{0.5, 0.5, 0.5};
#endif

    btVector3 aabb_min, aabb_max;
    dynamics->getBroadphase()->getBroadphaseAabb(aabb_min, aabb_max);
    aabb_min -= btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
    aabb_max += btVector3(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);

    switch (pass) {
    case 0:
      shape_drawer.drawOpenGL(m, col_obj->getCollisionShape(), wire_color,
                              debug_mode, aabb_min, aabb_max);
      break;
    case 1:
      shape_drawer.drawShadow(m, sun_direction * rot,
                              col_obj->getCollisionShape(), aabb_min, aabb_max);
      break;
    case 2:
      shape_drawer.drawOpenGL(m, col_obj->getCollisionShape(),
                              wire_color * btScalar(0.3), 0, aabb_min,
                              aabb_max);
      break;
    }
  }
}

btScalar screen_width{0};
btScalar screen_height{0};
btScalar screen_x{0};
btScalar screen_y{0};
bool screen_active{false};
bool screen_drag{false};
int screen_button{0};

btVector3 get_ray_to(btScalar x, btScalar y);
btVector3 get_plane_from_cam(const btVector3 ray);

void draw_set_screen_size(int width, int height) {
  screen_width = width;
  screen_height = height;
}
void draw_set_screen_pos(double x, double y) {
  constexpr double MIN_DRAG_MOVE_2{3.0};
  if (screen_button && !screen_drag) {
    if (SQ(screen_x - x) + SQ(screen_y - y) > MIN_DRAG_MOVE_2) {
      screen_drag = true;
      screen_x = x;
      screen_y = y;
    }
  } else {
    screen_x = x;
    screen_y = y;
  }
}
void draw_set_screen_active(bool active) {
  screen_active = active;
  if (!active) {
    screen_drag = false;
    screen_button = 0;
  }
}
void draw_set_screen_button(int button) {
  screen_button = button;
  if (!button)
    screen_drag = false;
}

#if 0
btScalar cam_ele{30.0};
btScalar cam_azi{10.0};
btScalar cam_dist{1.0};
#else
btScalar cam_ele{90.0};
btScalar cam_azi{0.0};
btScalar cam_dist{4.0};
#endif
#define CAM_ROT
#ifdef CAM_ROT
btScalar cam_rot{0.0};
#endif
btVector3 cam_tar{0, 0, 0.0215};

btVector3 cam_pos{};
btVector3 cam_up{};
btVector3 plane_fwd{};
btVector3 plane_left{};
static const btVector3 UP{0, 0, 1};
static const btVector3 FWD{0, 1, 0};
constexpr btScalar FNEAR{0.001};
constexpr btScalar FFAR{20.0};
constexpr btScalar ROTSTEP{1};      // degrees
constexpr btScalar WALKSTEP{0.050}; // meters
constexpr btScalar ZOOMSTEP{0.050}; // meters

void draw_update_camera(void) {
  if (screen_width == 0 && screen_width == 0)
    return;

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  auto aspect = screen_width / screen_height;
  glFrustum(-aspect * FNEAR, aspect * FNEAR, -FNEAR, FNEAR, FNEAR, FFAR);

  auto rot_azi = btQuaternion{UP, RAD(cam_azi)};
  plane_fwd = btMatrix3x3{rot_azi} * FWD;
  plane_left = UP.cross(plane_fwd);
  auto rot_ele = btQuaternion{plane_left, RAD(cam_ele)};
  auto cam_fwd = btMatrix3x3{rot_ele} * plane_fwd;
#ifdef CAM_ROT
  auto rot_rot = btQuaternion{cam_fwd, RAD(cam_rot)};
  cam_up = btMatrix3x3{rot_rot} * cam_fwd.cross(plane_left);
#else
  auto cam_up = cam_fwd.cross(plane_left);
#endif
  // auto cam_pos = cam_tar - cam_dist * cam_fwd;
  cam_pos = cam_tar - cam_dist * cam_fwd;
  myglLookAt(cam_pos[0], cam_pos[1], cam_pos[2], cam_tar[0], cam_tar[1],
             cam_tar[2], cam_up[0], cam_up[1], cam_up[2]);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  ImGui::Text("cam_ele: %f", cam_ele);
  ImGui::Text("cam_azi: %f", cam_azi);
  ImGui::Text("cam_pos: %f, %f, %f", cam_pos[0], cam_pos[1], cam_pos[2]);
  ImGui::Text("cam_tar: %f, %f, %f", cam_tar[0], cam_tar[1], cam_tar[2]);
  ImGui::Text("cam_up: %f, %f, %f", cam_up[0], cam_up[1], cam_up[2]);
  ImGui::Text("cam_dist: %f", cam_dist);
  ImGui::Text("screen_width: %f", screen_width);
  ImGui::Text("screen_height: %f", screen_height);
  ImGui::Text("screen_active: %i", screen_active);
  ImGui::Text("screen_drag: %i", screen_drag);
  ImGui::Text("screen_x: %f", screen_x);
  ImGui::Text("screen_y: %f", screen_y);
}

void draw_rot_left(void) { cam_azi = fmod(cam_azi - ROTSTEP, 360); }
void draw_rot_right(void) { cam_azi = fmod(cam_azi + ROTSTEP, 360); }
void draw_rot_up(void) { cam_ele = fmin(cam_ele + ROTSTEP, 90); }
void draw_rot_down(void) { cam_ele = fmax(cam_ele - ROTSTEP, 0); }
void draw_zoom_in(void) { cam_dist = fmax(cam_dist - ZOOMSTEP, 0.100); }
void draw_zoom_out(void) { cam_dist += ZOOMSTEP; }
void draw_walk_front(void) { cam_tar += WALKSTEP * plane_fwd; }
void draw_walk_left(void) { cam_tar += WALKSTEP * plane_left; }
void draw_walk_back(void) { cam_tar -= WALKSTEP * plane_fwd; }
void draw_walk_right(void) { cam_tar -= WALKSTEP * plane_left; }

void draw_field(const FieldGeometry &f, bool depth = false) {
  if (!depth)
    glDisable(GL_DEPTH_TEST);

  glColor3ubv(FIELD_GREEN);
  glRectf(-f.field_length / 2 - f.boundary_width - f.referee_width,
          -f.field_width / 2 - f.boundary_width - f.referee_width,
          f.field_length / 2 + f.boundary_width + f.referee_width,
          f.field_width / 2 + f.boundary_width + f.referee_width);

  glColor3ubv(WHITE);

  // mid line
  glRectf(-f.line_width / 2, -f.field_width / 2, f.line_width / 2,
          f.field_width / 2);

  // back lines
  glRectf(-f.field_length / 2, -f.field_width / 2,
          -f.field_length / 2 + f.line_width, f.field_width / 2);
  glRectf(f.field_length / 2, -f.field_width / 2,
          f.field_length / 2 - f.line_width, f.field_width / 2);

  // side lines
  glRectf(-f.field_length / 2, -f.field_width / 2, f.field_length / 2,
          -f.field_width / 2 + f.line_width);
  glRectf(-f.field_length / 2, f.field_width / 2, f.field_length / 2,
          f.field_width / 2 - f.line_width);

  // center dot
  constexpr int PSIDES{20};
  glBegin(GL_TRIANGLE_FAN);
  for (int i = 0; i < PSIDES; i++) {
    float a = RAD(i * 360.0 / PSIDES);
    float r = f.line_width;
    glVertex2f(r * cos(a), r * sin(a));
  }
  glEnd();

  // center circle
  constexpr int CSIDES{60};
  glBegin(GL_TRIANGLE_STRIP);
  for (int i = 0; i <= CSIDES; i++) {
    float a = RAD(i * 360.0 / CSIDES);
    float r1 = f.center_circle_radius - f.line_width;
    float r2 = f.center_circle_radius;
    glVertex2f(r1 * cos(a), r1 * sin(a));
    glVertex2f(r2 * cos(a), r2 * sin(a));
  }
  glEnd();

  // defense areas
  constexpr int DSIDES{15};
  glBegin(GL_TRIANGLE_STRIP);
  for (int i = -DSIDES; i <= 0; i++) {
    float a = RAD(i * 90.0 / DSIDES);
    float r1 = f.defense_radius - f.line_width;
    float r2 = f.defense_radius;
    float yoff = -f.defense_stretch / 2;
    glVertex2f(-f.field_length / 2 + r1 * cos(a), yoff + r1 * sin(a));
    glVertex2f(-f.field_length / 2 + r2 * cos(a), yoff + r2 * sin(a));
  }
  for (int i = 0; i <= DSIDES; i++) {
    float a = RAD(i * 90.0 / DSIDES);
    float r1 = f.defense_radius - f.line_width;
    float r2 = f.defense_radius;
    float yoff = f.defense_stretch / 2;
    glVertex2f(-f.field_length / 2 + r1 * cos(a), yoff + r1 * sin(a));
    glVertex2f(-f.field_length / 2 + r2 * cos(a), yoff + r2 * sin(a));
  }
  glEnd();
  glBegin(GL_TRIANGLE_STRIP);
  for (int i = -DSIDES; i <= 0; i++) {
    float a = RAD(i * 90.0 / DSIDES);
    float r1 = f.defense_radius - f.line_width;
    float r2 = f.defense_radius;
    float yoff = -f.defense_stretch / 2;
    glVertex2f(f.field_length / 2 - r1 * cos(a), yoff + r1 * sin(a));
    glVertex2f(f.field_length / 2 - r2 * cos(a), yoff + r2 * sin(a));
  }
  for (int i = 0; i <= DSIDES; i++) {
    float a = RAD(i * 90.0 / DSIDES);
    float r1 = f.defense_radius - f.line_width;
    float r2 = f.defense_radius;
    float yoff = f.defense_stretch / 2;
    glVertex2f(f.field_length / 2 - r1 * cos(a), yoff + r1 * sin(a));
    glVertex2f(f.field_length / 2 - r2 * cos(a), yoff + r2 * sin(a));
  }
  glEnd();

  // penalty dots
  glBegin(GL_TRIANGLE_FAN);
  for (int i = 0; i < PSIDES; i++) {
    float a = RAD(i * 360.0 / PSIDES);
    float r = f.line_width;
    float x = -f.field_length / 2 + f.defense_radius - f.line_width / 2;
    glVertex2f(x + r * cos(a), r * sin(a));
  }
  glEnd();
  glBegin(GL_TRIANGLE_FAN);
  for (int i = 0; i < PSIDES; i++) {
    float a = RAD(i * 360.0 / PSIDES);
    float r = f.line_width;
    float x = f.field_length / 2 - f.defense_radius + f.line_width / 2;
    glVertex2f(x + r * cos(a), r * sin(a));
  }
  glEnd();

  // the goals
  glRectf(-f.field_length / 2 - f.goal_depth - f.goal_wall_width,
          -f.goal_width / 2 - f.goal_wall_width, -f.field_length / 2,
          -f.goal_width / 2);
  glRectf(-f.field_length / 2 - f.goal_depth - f.goal_wall_width,
          f.goal_width / 2 + f.goal_wall_width, -f.field_length / 2,
          f.goal_width / 2);
  glRectf(-f.field_length / 2 - f.goal_depth - f.goal_wall_width,
          -f.goal_width / 2 - f.goal_wall_width,
          -f.field_length / 2 - f.goal_depth,
          f.goal_width / 2 + f.goal_wall_width);
  glRectf(f.field_length / 2 + f.goal_depth + f.goal_wall_width,
          -f.goal_width / 2 - f.goal_wall_width, f.field_length / 2,
          -f.goal_width / 2);
  glRectf(f.field_length / 2 + f.goal_depth + f.goal_wall_width,
          f.goal_width / 2 + f.goal_wall_width, f.field_length / 2,
          f.goal_width / 2);
  glRectf(f.field_length / 2 + f.goal_depth + f.goal_wall_width,
          -f.goal_width / 2 - f.goal_wall_width,
          f.field_length / 2 + f.goal_depth,
          f.goal_width / 2 + f.goal_wall_width);

#if 0
  // x-axis
  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_LINES);
  glVertex3f(-f.field_length / 2, 0.0, 0.0);
  glVertex3f(f.field_length / 2, 0.0, 0.0);
  glEnd();
#endif

  if (!depth)
    glEnable(GL_DEPTH_TEST);
}

void draw_screen_pos(void) {
  auto ray_to = get_ray_to(screen_x, screen_y);
  auto plane_pos = get_plane_from_cam(ray_to);
  // ImGui::Text("ray_to: %f, %f, %f", ray_to[0], ray_to[1], ray_to[2]);
  // ImGui::Text("plane_pos: %f, %f, %f", plane_pos[0], plane_pos[1],
  // plane_pos[2]);

  float r;
  if (screen_button) {
    r = 0.100;
  } else {
    r = 0.120;
  }
  if (screen_drag) {
    glColor4f(1.0, 0.0, 1.0, 0.5);
  } else {
    glColor4f(1.0, 1.0, 1.0, 0.4);
  }

  glEnable(GL_BLEND);
  //#define MY_BLEND_FUNC GL_ZERO
  //#define MY_BLEND_FUNC GL_ONE
  //#define MY_BLEND_FUNC GL_SRC_COLOR
  //#define MY_BLEND_FUNC GL_ONE_MINUS_SRC_COLOR
  //#define MY_BLEND_FUNC GL_DST_COLOR
  //#define MY_BLEND_FUNC GL_ONE_MINUS_DST_COLOR
  //#define MY_BLEND_FUNC GL_SRC_ALPHA
  //#define MY_BLEND_FUNC GL_ONE_MINUS_SRC_ALPHA
  //#define MY_BLEND_FUNC GL_DST_ALPHA
  //#define MY_BLEND_FUNC GL_ONE_MINUS_DST_ALPHA
  //#define MY_BLEND_FUNC GL_CONSTANT_COLOR
  //#define MY_BLEND_FUNC GL_ONE_MINUS_CONSTANT_COLOR
  //#define MY_BLEND_FUNC GL_CONSTANT_ALPHA
  //#define MY_BLEND_FUNC GL_ONE_MINUS_CONSTANT_ALPHA
  //#define MY_BLEND_FUNC GL_SRC_ALPHA_SATURATE
  //#define MY_BLEND_FUNC GL_SRC1_ALPHA
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_DST_COLOR);
  // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  // glBlendFunc(GL_SRC_ALPHA, MY_BLEND_FUNC);
  glBegin(GL_TRIANGLE_FAN);
  for (int i = 0; i < 30; i++) {
    float a = RAD(i * 360.0 / 30);
    glVertex2f(plane_pos[0] + r * cos(a), plane_pos[1] + r * sin(a));
  }
  glEnd();
  glDisable(GL_BLEND);
}

void draw_physics_world(World *world) {
  glClear(GL_STENCIL_BUFFER_BIT);
  glEnable(GL_CULL_FACE);
  draw_physics_world_pass(world, 0);

  glDisable(GL_LIGHTING);
  glDepthMask(GL_FALSE);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_STENCIL_TEST);
  glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
  glStencilFunc(GL_ALWAYS, 1, 0xFFFFFFFFL);
  glFrontFace(GL_CCW);
  glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
  draw_physics_world_pass(world, 1);
  glFrontFace(GL_CW);
  glStencilOp(GL_KEEP, GL_KEEP, GL_DECR);
  draw_physics_world_pass(world, 1);
  glFrontFace(GL_CCW);

  glPolygonMode(GL_FRONT, GL_FILL);
  glPolygonMode(GL_BACK, GL_FILL);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glEnable(GL_LIGHTING);
  glDepthMask(GL_TRUE);
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);
  glEnable(GL_CULL_FACE);
  glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);

  glDepthFunc(GL_LEQUAL);
  glStencilFunc(GL_NOTEQUAL, 0, 0xFFFFFFFFL);
  glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
  glDisable(GL_LIGHTING);
  draw_physics_world_pass(world, 2);
  glEnable(GL_LIGHTING);
  glDepthFunc(GL_LESS);
  glDisable(GL_STENCIL_TEST);
  glDisable(GL_CULL_FACE);

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);
  // updateCamera();

  glDisable(GL_LIGHTING);
  glColor3f(0, 0, 0);

  if (screen_active)
    draw_screen_pos();
}

void draw_world(struct World *world) {
  draw_update_camera();
  draw_field(*world_get_field(world));
  draw_physics_world(world);
}

btVector3 get_ray_to(btScalar x, btScalar y) {
  // TODO: understand this function (ported from bullet demo) and make it simpler
  btScalar top{1.0};
  btScalar bottom{-1.0};
  btScalar near_plane{1.0};
  btScalar tan_fov{(top - bottom) * btScalar{0.5} / near_plane};
  btScalar fov{btScalar{2.0} * btAtan(tan_fov)};

  btVector3 ray_from{cam_pos};
  btVector3 ray_fwd{cam_tar - cam_pos};
  ray_fwd.normalize();

  btScalar far_plane{10000.0};
  ray_fwd *= far_plane;

  btVector3 rightOffset;
  btVector3 vertical = cam_up;

  btVector3 hor;
  hor = ray_fwd.cross(vertical);
  hor.normalize();
  vertical = hor.cross(ray_fwd);
  vertical.normalize();

  btScalar tanfov{tanf(0.5 * fov)};
  hor *= 2.0 * far_plane * tanfov;
  vertical *= 2.0 * far_plane * tanfov;

  btScalar aspect{screen_width / screen_height};
  hor *= aspect;

  btVector3 ray_to_center{ray_from + ray_fwd};
  btVector3 dHor{hor / screen_width};
  btVector3 dVert{vertical / screen_height};

  btVector3 ray_to{ray_to_center - hor / 2 + vertical / 2};
  ray_to += x * dHor;
  ray_to -= y * dVert;
  ray_to.normalize();
  return ray_to;
}

btVector3 get_plane_from_cam(const btVector3 ray) {
  return cam_pos - cam_pos[2] / ray[2] * ray;
}
