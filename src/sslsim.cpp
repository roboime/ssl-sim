/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "sslsim.h"

#include <random>
#include <btBulletDynamicsCommon.h>
#include "utils/stack_vector.h"

// physics units are meters and kilograms
constexpr btScalar BALL_DIAM = 0.043;    // 43mm
constexpr btScalar BALL_MASS = 0.046;    // 46g
constexpr btScalar ROBOT_DIAM = 0.180;   // 180mm
constexpr btScalar ROBOT_HEIGHT = 0.150; // 150mm
constexpr btScalar ROBOT_MASS = 2.000;   // 2lg
constexpr btScalar DROP_HEIGHT = 0.500;  // 43mm

typedef btRigidBody::btRigidBodyConstructionInfo btRigidBodyCI;

static std::random_device rdevice{};
static std::default_random_engine gen{rdevice()};
static Pos2 random_robot_pos2(const FieldGeometry *f) {
  std::uniform_real_distribution<Float> rx(
      -f->field_length / 2 + ROBOT_DIAM / 2,
      f->field_length / 2 + ROBOT_DIAM / 2);
  std::uniform_real_distribution<Float> ry(-f->field_width / 2 + ROBOT_DIAM / 2,
                                           f->field_width / 2 + ROBOT_DIAM / 2);
  std::uniform_real_distribution<Float> rw(0, 360);
  // return std::forward(rx(gen), ry(gen), rw(gen));
  return {rx(gen), ry(gen), rw(gen)};
}

extern "C" {

struct Ball {
  Robot *last_touching_robot{nullptr};

  // physics body
  btSphereShape shape{BALL_DIAM / 2};
  btDefaultMotionState motion_state{
      btTransform({0, 0, 0, 1}, {0, 0, DROP_HEIGHT})};
  const struct CI : public btRigidBodyCI {
    CI(btDefaultMotionState *s, btCollisionShape *h)
        : btRigidBodyCI(BALL_MASS, s, h, {0, 0, 0}) {
      m_restitution = 0.5;
    }
  } body_ci{&motion_state, &shape};
  btRigidBody body{body_ci};

  // Ball() { shape.calculateLocalInertia(mass, inertia); }
};

struct Robot {
  int id;
  Team team;

  // physics body
  btCylinderShapeZ shape{{ROBOT_DIAM / 2, ROBOT_DIAM / 2, ROBOT_HEIGHT / 2}};
  // btSphereShape shape{BALL_DIAM / 2};
  btDefaultMotionState motion_state{
      btTransform({0, 0, 0, 1}, {0, 0, ROBOT_HEIGHT + 0.001 + 0.5})};
  const struct CI : public btRigidBodyCI {
    CI(btDefaultMotionState *s, btCollisionShape *h)
        : btRigidBodyCI(ROBOT_MASS, s, h, {0, 0, 0}) {
      m_restitution = 0.9;
    }
  } body_ci{&motion_state, &shape};
  btRigidBody body{body_ci};

  Robot(int id, Team team) : id(id), team(team) {
    // shape.calculateLocalInertia(mass, inertia);
  }

  // TODO: wheels and their joints

  // DO NOT USE THIS CONSTRUCTOR:
  Robot(void) : Robot(-1, TEAM_BLUE) {}
};

struct Ground {
  // physics body
  btStaticPlaneShape shape{{0, 0, 1}, 1};
  btDefaultMotionState motion_state{btTransform({0, 0, 0, 1}, {0, 0, -1})};
  // const btRigidBodyCI body_ci{0, &motion_state, &shape, {0, 0, 0}};
  const struct CI : public btRigidBodyCI {
    CI(btDefaultMotionState *s, btCollisionShape *h)
        : btRigidBodyCI(0, s, h, {0, 0, 0}) {
      m_restitution = 0.9;
    }
  } body_ci{&motion_state, &shape};
  btRigidBody body{body_ci};
};

struct World {
  const FieldGeometry *const field;
  // XXX: these don't work, so I made my own vector with stack allocation
  // btAlignedObjectArray<Robot> robots{};
  // btAlignedObjectArray<Ball> balls{};
  std::stack_vector<Ball, 20> balls{};
  std::stack_vector<Robot, 100> robots{};

  // TODO: findout what can be static, maybe?

  // the simulation timestamp, and a clock, can be used for comparison
  btScalar timestamp{0};
  btClock real_clock{};

  // broadphase
  btDbvtBroadphase broadphase{};

  // config and dispatcher
  btDefaultCollisionConfiguration collision_configuration{};
  btCollisionDispatcher dispatcher{&collision_configuration};

  // the solver
  btSequentialImpulseConstraintSolver solver{};

  // the world
  btDiscreteDynamicsWorld dynamics{&dispatcher, &broadphase, &solver,
                                   &collision_configuration};

  // the ground
  Ground ground{};

  World(const FieldGeometry *const field) : field{field} {
    dynamics.setGravity({0, 0, -9.80665});
    dynamics.addRigidBody(&ground.body);
    balls.emplace_back();
    dynamics.addRigidBody(&balls[0].body);
  }

  World(const World *w)
      : field{w->field}, balls{w->balls}, robots{w->robots}, broadphase{},
        // broadphase{w->broadphase},
        collision_configuration{w->collision_configuration},
        // dispatcher{w->dispatcher},
        dispatcher{&collision_configuration},
        // dynamics{w->dynamics}
        dynamics{&dispatcher, &broadphase, &solver, &collision_configuration} {
    // dispatcher.setCollisionConfiguration(&collision_configuration);
    // robots.copyFromArray(w->robots);
    // TODO: is this enough? certainly not
  }

  ~World(void) {
    for (auto &robot : robots)
      dynamics.removeRigidBody(&robot.body);
    for (auto &ball : balls)
      dynamics.removeRigidBody(&ball.body);
    dynamics.removeRigidBody(&ground.body);
  }
};

// world.h impl
World *new_world(const FieldGeometry *field) { return new World{field}; }
World *clone_world(const World *world) { return new World{world}; }
void delete_world(World *world) { delete world; }

void world_step(struct World *world, Float time_step, int max_substeps,
                Float fixed_time_step) {
  world->dynamics.stepSimulation(time_step, max_substeps, fixed_time_step);
  world->timestamp += time_step;
}

const FieldGeometry *world_get_field(const World *world) {
  return world->field;
}
Ball *world_get_ball(World *world) { return &(world->balls[0]); }
int world_robot_count(World *world) { return world->robots.size(); }
Robot *world_get_robot(World *world, int index) {
  return &world->robots[index];
}

void world_add_robot(World *world, int id, Team team) {
  world->robots.emplace_back(id, team);
  world->dynamics.addRigidBody(&world->robots.back().body);
  robot_set_pos(&world->robots.back(), random_robot_pos2(world->field));
  // TODO: robot position
}

btDiscreteDynamicsWorld *world_bt_dynamics(struct World *world) {
  return &world->dynamics;
}

// ball.h impl
struct Vec3 ball_get_vec(struct Ball *ball) {
  btTransform trans;
  ball->body.getMotionState()->getWorldTransform(trans);
  auto orig = trans.getOrigin();
  return {orig.getX(), orig.getY(), orig.getZ()};
}

void ball_set_vec(struct Ball *ball, const struct Vec2 vec) {
  auto transf = btTransform({0, 0, 0, 1}, {vec.x, vec.y, DROP_HEIGHT});
  ball->body.setWorldTransform(transf);
  ball->body.activate(true);
}

struct Pos3 ball_get_pos(struct Ball *ball);
void ball_set_pos(struct Ball *ball, const struct Pos3 pos);

struct Pos3 ball_get_vel(struct Ball *ball);
void ball_set_vel(struct Ball *ball, const struct Pos3 vel);

int ball_is_touching_robot(struct Ball *ball, struct Robot *robot);

struct Robot *ball_last_touching_robot(struct Ball *ball);

/// fast squared speed (magnitude of velocity)
Float ball_get_speed2(struct Ball *ball);

/// fast squared speed (magnitude of velocity)
Float ball_get_peak_speed2_from_last_kick(struct Ball *ball);

btRigidBody *ball_bt_rigid_body(struct Ball *ball) { return &ball->body; }

// robot.h impl
int get_id(struct Robot *robot);
enum Team get_team(struct Robot *robot);

struct Pos2 robot_get_pos(struct Robot *robot);
void robot_set_pos(struct Robot *robot, const struct Pos2 pos) {
  auto transf = btTransform({0, 0, 1, pos.w}, {pos.x, pos.y, DROP_HEIGHT});
  robot->body.setWorldTransform(transf);
  robot->body.activate(true);
}

struct Pos2 robot_get_vel(struct Robot *robot);
void robot_set_vel(struct Robot *robot, const struct Pos2 vel);

/// return is C bool (1 for true, 0 for false)
int robot_is_touching_robot(struct Robot *robot, struct Robot *tobor);

/// fast squared speed (magnitude of velocity)
Float robot_get_speed2(struct Robot *robot);
}
