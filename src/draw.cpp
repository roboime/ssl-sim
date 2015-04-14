/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "draw.h"
#include "world.h"
#include "field.h"

#include <GLFW/glfw3.h>
#include <btBulletDynamicsCommon.h>
#include "imgui.h"

#include "glutils.h"
#include "colors.h"

// Utils
template <typename T> constexpr T RAD(T D) { return M_PI * D / 180.; }
template <typename T> constexpr T DEG(T R) { return 180. * R / M_PI; }

struct DebugDrawer : public btIDebugDraw {
  DebugDrawer() {}
  virtual ~DebugDrawer() {}

  virtual void drawLine(const btVector3 &from, const btVector3 &to,
                        const btVector3 &fromColor, const btVector3 &toColor) {
    glBegin(GL_LINES);
    glColor3f(fromColor.getX(), fromColor.getY(), fromColor.getZ());
    glVertex3d(from.getX(), from.getY(), from.getZ());
    glColor3f(toColor.getX(), toColor.getY(), toColor.getZ());
    glVertex3d(to.getX(), to.getY(), to.getZ());
    glEnd();
  }

  virtual void drawLine(const btVector3 &from, const btVector3 &to,
                        const btVector3 &color) {
    drawLine(from, to, color, color);
  }

  virtual void drawContactPoint(const btVector3 &pointOnB,
                                const btVector3 &normalOnB, btScalar distance,
                                int /*lifeTime*/, const btVector3 &color) {
    btVector3 to = pointOnB + normalOnB * distance;
    const btVector3 &from = pointOnB;
    glColor4f(color.getX(), color.getY(), color.getZ(), 1.f);
    glBegin(GL_LINES);
    glVertex3d(from.getX(), from.getY(), from.getZ());
    glVertex3d(to.getX(), to.getY(), to.getZ());
    glEnd();
  }

  virtual void draw3dText(const btVector3 &location,
                          const char * /*textString*/) {
    glRasterPos3f(location.x(), location.y(), location.z());
  }

  virtual void reportErrorWarning(const char *warningString) {
    fputs(warningString, stderr);
  }

  virtual void drawSphere(const btVector3 &p, btScalar radius,
                          const btVector3 &color) {
    glColor4f(color.getX(), color.getY(), color.getZ(), btScalar(1.0f));
    glPushMatrix();
    glTranslatef(p.getX(), p.getY(), p.getZ());

    int lats = 5;
    int longs = 5;

    int i, j;
    for (i = 0; i <= lats; i++) {
      btScalar lat0 = SIMD_PI * (-btScalar(0.5) + (btScalar)(i - 1) / lats);
      btScalar z0 = radius * sin(lat0);
      btScalar zr0 = radius * cos(lat0);

      btScalar lat1 = SIMD_PI * (-btScalar(0.5) + (btScalar)i / lats);
      btScalar z1 = radius * sin(lat1);
      btScalar zr1 = radius * cos(lat1);

      glBegin(GL_QUAD_STRIP);
      for (j = 0; j <= longs; j++) {
        btScalar lng = 2 * SIMD_PI * (btScalar)(j - 1) / longs;
        btScalar x = cos(lng);
        btScalar y = sin(lng);

        glNormal3f(x * zr0, y * zr0, z0);
        glVertex3f(x * zr0, y * zr0, z0);
        glNormal3f(x * zr1, y * zr1, z1);
        glVertex3f(x * zr1, y * zr1, z1);
      }
      glEnd();
    }

    glPopMatrix();
  }

#if 0
  virtual void drawTriangle(const btVector3 &a, const btVector3 &b,
                            const btVector3 &c, const btVector3 &color,
                            btScalar alpha) {
    const btVector3 n = btCross(b - a, c - a).normalized();
    glBegin(GL_TRIANGLES);
    glColor4f(color.getX(), color.getY(), color.getZ(), alpha);
    glNormal3d(n.getX(), n.getY(), n.getZ());
    glVertex3d(a.getX(), a.getY(), a.getZ());
    glVertex3d(b.getX(), b.getY(), b.getZ());
    glVertex3d(c.getX(), c.getY(), c.getZ());
    glEnd();
  }
#endif

  int debug_mode{1};
  virtual void setDebugMode(int mode) { debug_mode = mode; }
  virtual int getDebugMode() const { return debug_mode; }
  inline void setSingleDebugMode(DebugDrawModes mode, bool on) {
    debug_mode = on ? debug_mode | mode : debug_mode & ~mode;
  }
  inline bool getSingleDebugMode(DebugDrawModes mode) const {
    return (debug_mode & mode) == mode;
  }
};
static DebugDrawer debug_drawer;

static World *world;
static const FieldGeometry *field;
static btDiscreteDynamicsWorld *dynamics;
void draw_init(World *w) {
  world = w;
  field = world_get_field(w);
  dynamics = world_bt_dynamics(world);
  dynamics->setDebugDrawer(&debug_drawer);
}

float draw_screen_x{0};
float draw_screen_y{0};
float draw_screen_width{0};
float draw_screen_height{0};
float projected_mouse_x{0};
float projected_mouse_y{0};
const float &screen_width{draw_screen_width};
const float &screen_height{draw_screen_height};
const float &screen_x{draw_screen_x};
const float &screen_y{draw_screen_y};
;

btVector3 get_ray_to(btScalar x, btScalar y);
btVector3 get_plane_from_cam(const btVector3 ray);

#if 1
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

void draw_field(bool depth = false) {
  auto &f = *field;
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

void draw_mouse_projection(int button, bool drag) {
  auto ray_to = get_ray_to(screen_x, screen_y);
  auto plane_pos = get_plane_from_cam(ray_to);
  projected_mouse_x = plane_pos[0];
  projected_mouse_y = plane_pos[1];
  // ImGui::Text("ray_to: %f, %f, %f", ray_to[0], ray_to[1], ray_to[2]);
  // ImGui::Text("plane_pos: %f, %f, %f", plane_pos[0], plane_pos[1],
  // plane_pos[2]);

  float r;
  if (button) {
    r = 0.100;
  } else {
    r = 0.120;
  }
  if (drag) {
    glColor4f(1.0, 0.0, 1.0, 0.5);
  } else {
    glColor4f(1.0, 1.0, 1.0, 0.4);
  }

  glDisable(GL_TEXTURE_2D);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_DST_COLOR);
  // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBegin(GL_TRIANGLE_FAN);
  for (int i = 0; i < 30; i++) {
    float a = RAD(i * 360.0 / 30);
    glVertex2f(plane_pos[0] + r * cos(a), plane_pos[1] + r * sin(a));
  }
  glEnd();
  glDisable(GL_BLEND);
}

#if 0
void draw_physics_world(void) {
  glClear(GL_STENCIL_BUFFER_BIT);
  glEnable(GL_CULL_FACE);
  draw_physics_world_pass(0);

  glDisable(GL_LIGHTING);
  glDepthMask(GL_FALSE);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_STENCIL_TEST);
  glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
  glStencilFunc(GL_ALWAYS, 1, 0xFFFFFFFFL);
  glFrontFace(GL_CCW);
  glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
  draw_physics_world_pass(1);
  glFrontFace(GL_CW);
  glStencilOp(GL_KEEP, GL_KEEP, GL_DECR);
  draw_physics_world_pass(1);
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
  draw_physics_world_pass(2);
  glEnable(GL_LIGHTING);
  glDepthFunc(GL_LESS);
  glDisable(GL_STENCIL_TEST);
  glDisable(GL_CULL_FACE);

  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
  glMatrixMode(GL_MODELVIEW);

  glDisable(GL_LIGHTING);
  glColor3f(0, 0, 0);
}
#endif

void draw_world(void) {
  draw_update_camera();
  draw_field();
  // draw_physics_world();
  // draw_physics_world_pass(2);
}

void draw_debug(void) { dynamics->debugDrawWorld(); }

void draw_set_debug_mode(int mode) { debug_drawer.setDebugMode(mode); }

void draw_options_window(void) {
  ImGui::Begin("Draw options");
#define ADD_DBG_OPT(OPT, DESC)                                                 \
  bool _##OPT = debug_drawer.getSingleDebugMode(DebugDrawer::OPT);             \
  ImGui::Checkbox(DESC, &_##OPT);                                              \
  debug_drawer.setSingleDebugMode(DebugDrawer::OPT, _##OPT);
  ADD_DBG_OPT(DBG_DrawWireframe, "draw wireframe");
  ADD_DBG_OPT(DBG_DrawAabb, "draw AABB");
  ADD_DBG_OPT(DBG_DrawFeaturesText, "draw features text");
  ADD_DBG_OPT(DBG_DrawContactPoints, "draw contact pooints");
  ADD_DBG_OPT(DBG_NoDeactivation, "no deactivation");
  ADD_DBG_OPT(DBG_NoHelpText, "no help text");
  ADD_DBG_OPT(DBG_DrawText, "draw text");
  ADD_DBG_OPT(DBG_ProfileTimings, "profile timings");
  ADD_DBG_OPT(DBG_EnableSatComparison, "enable sat compresion");
  ADD_DBG_OPT(DBG_DisableBulletLCP, "disable bullet LCP");
  ADD_DBG_OPT(DBG_EnableCCD, "enable CCD");
  ADD_DBG_OPT(DBG_DrawConstraints, "draw constraints");
  ADD_DBG_OPT(DBG_DrawConstraintLimits, "draw constraint limits");
  ADD_DBG_OPT(DBG_FastWireframe, "fast wireframe");
  ADD_DBG_OPT(DBG_DrawNormals, "draw normals");
#undef ADD_DBG_OPT
  ImGui::End();
}

btVector3 get_ray_to(btScalar x, btScalar y) {
  // TODO: understand this function (ported from bullet demo) and make it
  // simpler
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
