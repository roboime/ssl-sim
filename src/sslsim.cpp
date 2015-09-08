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
#include "utils/stack_vector.hh"
#include "utils/math.hh"

// physics units are meters and kilograms
constexpr btScalar BALL_DIAM = 0.043;    // 43mm
constexpr btScalar BALL_MASS = 0.046;    // 46g
constexpr btScalar ROBOT_DIAM = 0.180;   // 180mm
constexpr btScalar ROBOT_HEIGHT = 0.145; // 145mm
constexpr btScalar ROBOT_MASS = 2.000;   // 2lg
constexpr btScalar DROP_HEIGHT = 0.500;  // 43mm
constexpr btScalar WHEEL_DIAM = 0.050;
constexpr btScalar WHEEL_WIDTH = 0.010;
constexpr btScalar WHEEL_HEIGHT = 0.005; // the robot is 5mm up the ground
constexpr int WHEELS_COUNT = 4;
constexpr btScalar WHEELS_ANGLE[] = {RAD(60.0), RAD(135.0), RAD(-135.0),
                                     RAD(-60.0)};

typedef btRigidBody::btRigidBodyConstructionInfo btRigidBodyCI;

static std::random_device rdevice{};
static std::default_random_engine gen{rdevice()};
static Pos2 random_robot_pos2(const FieldGeometry *f) {
  auto w = f->field_length / 2 - ROBOT_DIAM / 2;
  auto h = f->field_width / 2 - ROBOT_DIAM / 2;
  std::uniform_real_distribution<Float> rx(-w, w);
  std::uniform_real_distribution<Float> ry(-h, h);
  std::uniform_real_distribution<Float> rw(0, RAD(360));
  // return std::forward(rx(gen), ry(gen), rw(gen));
  return {rx(gen), ry(gen), rw(gen)};
}

static const struct BallCI : public btRigidBodyCI {
  btSphereShape shape{BALL_DIAM / 2};
  BallCI() : btRigidBodyCI(BALL_MASS, nullptr, &shape, {0, 0, 0}) {
    m_restitution = 0.5;
  }
} ball_ci{};

extern "C" struct Ball {

  Robot *last_touching_robot{nullptr};

  btRigidBody body{ball_ci};

  Ball(void) {
    body.setCcdMotionThreshold(0.005);   // 5mm
    body.setCcdSweptSphereRadius(0.001); // 1mm
  }
};

static const struct RobotCI : public btRigidBodyCI {
#if 0
#if 0
  btCylinderShapeZ chassis_shape{{ROBOT_DIAM / 2, ROBOT_DIAM / 2, ROBOT_HEIGHT / 2}};
  btCompoundShape shape;
  RobotCI() : btRigidBodyCI(ROBOT_MASS, nullptr, &shape, {0, 0, 0}) {
    m_restitution = 0.2;

    btScalar h = ROBOT_HEIGHT / 2 - WHEEL_DIAM / 2 + WHEEL_HEIGHT;
    shape.addChildShape(btTransform({0, 0, 0, 1}, {0, 0, h}), &chassis_shape);
  }
#else
  btCylinderShapeZ shape{{ROBOT_DIAM / 2, ROBOT_DIAM / 2, ROBOT_HEIGHT / 2}};
  RobotCI() : btRigidBodyCI(ROBOT_MASS, nullptr, &shape, {0, 0, 0}) {
    m_restitution = 0.2;
  }
#endif
#else
  btCylinderShapeZ chassis_shape{
      {ROBOT_DIAM / 2, ROBOT_DIAM / 2, ROBOT_HEIGHT / 2}};
  btCylinderShapeX wheel_shape{
      {WHEEL_WIDTH / 2, WHEEL_DIAM / 2, WHEEL_DIAM / 2}};
  btCompoundShape shape{};
  RobotCI() : btRigidBodyCI(ROBOT_MASS, nullptr, &shape, {0, 0, 0}) {
    m_restitution = 0.2;

    btScalar h = ROBOT_HEIGHT / 2 - WHEEL_DIAM / 2 + WHEEL_HEIGHT;
    shape.addChildShape(btTransform({0, 0, 0, 1}, {0, 0, h}), &chassis_shape);

    const btVector3 dis{ROBOT_DIAM / 2 + WHEEL_WIDTH / 2, 0, 0};
    for (int i = 0; i < 4; i++) {
      auto rot = btQuaternion{{0, 0, -1}, WHEELS_ANGLE[i]};
      auto pos = btMatrix3x3{rot} * dis;
      shape.addChildShape(btTransform(rot, pos), &wheel_shape);
    }
  }
#endif
} robot_ci{};

extern "C" struct Robot {
  int id;
  Team team;

  btRigidBody chassis{robot_ci};
  btRaycastVehicle::btVehicleTuning tuning{};
  btDefaultVehicleRaycaster raycaster;
  btRaycastVehicle vehicle{tuning, &chassis, &raycaster};

  Robot(int id, Team team, btDynamicsWorld *dynamics)
      : id{id}, team{team}, raycaster{dynamics} {
    chassis.setActivationState(DISABLE_DEACTIVATION);

    const btVector3 dis{ROBOT_DIAM / 2 - WHEEL_WIDTH / 2, 0,
                        -ROBOT_HEIGHT / 2 + WHEEL_DIAM / 2 - WHEEL_HEIGHT};
    for (int i = 0; i < WHEELS_COUNT; i++) {
      auto rotm = btMatrix3x3{btQuaternion{{0, 0, -1}, WHEELS_ANGLE[i]}};
      auto conp = rotm * dis;
      auto wdir = rotm * btVector3{0, 1, 0};
      auto axle = rotm * btVector3{1, 0, 0};
      vehicle.addWheel(conp, wdir, axle, 0.00, WHEEL_DIAM / 2, tuning, false);
      auto &wheel = vehicle.getWheelInfo(i);
      wheel.m_suspensionStiffness = 5000.0;
      wheel.m_wheelsDampingRelaxation = 20.0;
      wheel.m_wheelsDampingCompression = 4.4;
      wheel.m_frictionSlip = 1000.0;
      wheel.m_rollInfluence = 0.1;
    }
  }

  // XXX: DO NOT USE THIS CONSTRUCTOR:
  Robot(void) : id{-1}, team{TEAM_NONE}, raycaster{nullptr} {}
};

struct PlaneCI : public btRigidBodyCI {
  btStaticPlaneShape shape;
  PlaneCI(btVector3 plane_normal, btScalar plane_constant)
      : btRigidBodyCI(0, nullptr, &shape, {0, 0, 0}),
        shape{plane_normal, plane_constant} {
    m_restitution = 1.0;
  }
};

struct GoalCI : public btRigidBodyCI {
  btBoxShape wall_side, wall_rear;
  btCompoundShape shape;
  GoalCI(btScalar goal_width, btScalar goal_depth, btScalar goal_height,
         btScalar wall_width)
      : btRigidBodyCI(0, nullptr, &shape, {0, 0, 0}),
        wall_side{
            {goal_depth / 2 + wall_width / 2, wall_width / 2, goal_height / 2}},
        wall_rear{{wall_width / 2, goal_width / 2, goal_height / 2}}, shape{} {
    shape.addChildShape(
        btTransform({0, 0, 0, 1},
                    {-goal_depth / 2 - wall_width / 2,
                     -goal_width / 2 - wall_width / 2, goal_height / 2}),
        &wall_side);
    shape.addChildShape(
        btTransform({0, 0, 0, 1},
                    {-goal_depth / 2 - wall_width / 2,
                     +goal_width / 2 + wall_width / 2, goal_height / 2}),
        &wall_side);
    shape.addChildShape(btTransform({0, 0, 0, 1}, {-goal_depth - wall_width / 2,
                                                   0, goal_height / 2}),
                        &wall_rear);
  }
};

struct Field {
  const PlaneCI ground_ci;
  const PlaneCI ceil_ci;
  const PlaneCI left_wall_ci;
  const PlaneCI right_wall_ci;
  const PlaneCI top_wall_ci;
  const PlaneCI bottom_wall_ci;
  const GoalCI goal_ci;
  btRigidBody ground_body;
  btRigidBody ceil_body;
  btRigidBody left_wall_body;
  btRigidBody right_wall_body;
  btRigidBody top_wall_body;
  btRigidBody bottom_wall_body;
  btRigidBody left_goal_body;
  btRigidBody right_goal_body;
  Field(const FieldGeometry *f)
      : ground_ci{{0, 0, 1}, 0.0}, ceil_ci{{0, 0, -1}, -10.0},
        left_wall_ci{{1, 0, 0}, -field_limit_x(f)},
        right_wall_ci{{-1, 0, 0}, -field_limit_x(f)},
        top_wall_ci{{0, 1, 0}, -field_limit_y(f)},
        bottom_wall_ci{{0, -1, 0}, -field_limit_y(f)},
        goal_ci{f->goal_width, f->goal_depth, f->goal_height,
                f->goal_wall_width},
        ground_body{ground_ci}, ceil_body{ceil_ci},
        left_wall_body{left_wall_ci}, right_wall_body{right_wall_ci},
        top_wall_body{top_wall_ci}, bottom_wall_body{bottom_wall_ci},
        left_goal_body{goal_ci}, right_goal_body{goal_ci} {
    auto trans1 = btTransform({0, 0, 0, 1}, {-f->field_length / 2, 0, 0});
    left_goal_body.setWorldTransform(trans1);
    auto trans2 = btTransform({0, 0, 1, 0}, {f->field_length / 2, 0, 0});
    right_goal_body.setWorldTransform(trans2);
  }
};

extern "C" struct World {
  const FieldGeometry *const field_geometry;

  // the field (bounding box and goals)
  Field field;

  // XXX: these don't work, so I made my own vector with stack allocation
  // btAlignedObjectArray<Robot> robots{};
  // btAlignedObjectArray<Ball> balls{};
  utils::stack_vector<Ball, 20> balls{};
  utils::stack_vector<Robot, 100> robots{};

  // TODO: findout what can be static, maybe?

  // the simulation timestamp, and a clock, can be used for comparison
  unsigned int frame_number{0};
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

  World(const FieldGeometry *const field_geom)
      : field_geometry{field_geom}, field{field_geom} {
    dynamics.setGravity({0, 0, -9.80665});
    dynamics.addRigidBody(&field.ground_body);
    dynamics.addRigidBody(&field.ceil_body);
    dynamics.addRigidBody(&field.left_wall_body);
    dynamics.addRigidBody(&field.right_wall_body);
    dynamics.addRigidBody(&field.top_wall_body);
    dynamics.addRigidBody(&field.bottom_wall_body);
    dynamics.addRigidBody(&field.left_goal_body);
    dynamics.addRigidBody(&field.right_goal_body);
    balls.emplace_back();
    dynamics.addRigidBody(&balls.back().body);
    ball_set_vec(&balls.back(), {});
  }

  World(const World *w)
      : field_geometry{w->field_geometry},
        //
        field{w->field},
        //
        balls{w->balls},
        //
        robots{w->robots},
        //
        broadphase{},
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
    for (auto &robot : robots) {
      // dynamics.removeRigidBody(robot.vehicle.getRigidBody());
      dynamics.removeRigidBody(&robot.chassis);
    }

    for (auto &ball : balls) {
      dynamics.removeRigidBody(&ball.body);
    }

    dynamics.removeRigidBody(&field.ground_body);
    dynamics.removeRigidBody(&field.ceil_body);
    dynamics.removeRigidBody(&field.left_wall_body);
    dynamics.removeRigidBody(&field.right_wall_body);
    dynamics.removeRigidBody(&field.top_wall_body);
    dynamics.removeRigidBody(&field.bottom_wall_body);
    dynamics.removeRigidBody(&field.left_goal_body);
    dynamics.removeRigidBody(&field.right_goal_body);
  }
};

// API implementation

extern "C" {

// world.h impl
World *new_world(const FieldGeometry *field_geom) {
  return new World{field_geom};
}
World *clone_world(const World *world) { return new World{world}; }
void delete_world(World *world) { delete world; }

void world_step(struct World *world) {
  // TODO: realtime step
  world_step_delta(world, 1.0 / 60, 2, 1.0 / 60 / 2);
}

void world_step_delta(struct World *world, Float time_step, int max_substeps,
                      Float fixed_time_step) {
  if (world->robots.size() > 0) {
    constexpr btScalar force = 10.0;
    world->robots[0].vehicle.applyEngineForce(force, 0);
    world->robots[0].vehicle.applyEngineForce(force, 1);
    world->robots[0].vehicle.applyEngineForce(force, 2);
    world->robots[0].vehicle.applyEngineForce(force, 3);
    // world->robots[0].vehicle.applyEngineForce(-force, 2);
    // world->robots[0].vehicle.applyEngineForce(-force, 3);
  }
  world->dynamics.stepSimulation(time_step, max_substeps, fixed_time_step);
  world->timestamp += time_step;
  world->frame_number++;
}

const FieldGeometry *world_get_field(const World *world) {
  return world->field_geometry;
}

int world_ball_count(const World *world) { return world->balls.size(); }

const Ball *world_get_ball(const World *world, int index) {
  return &(world->balls[index]);
}

const Ball *world_get_game_ball(const World *world) {
  return &(world->balls[0]);
}

Ball *world_get_mut_ball(World *world, int index) {
  return &(world->balls[index]);
}

Ball *world_get_mut_game_ball(World *world) { return &(world->balls[0]); }

int world_robot_count(const World *world) { return world->robots.size(); }

const Robot *world_get_robot(const World *world, int index) {
  return &world->robots[index];
}

Robot *world_get_mut_robot(World *world, int index) {
  return &world->robots[index];
}

void world_add_robot(World *world, int id, Team team) {
  world->robots.emplace_back(id, team, &world->dynamics);
  world->dynamics.addRigidBody(&world->robots.back().chassis);
  // world->dynamics.addVehicle(&world->robots.back().vehicle);
  robot_set_pos(&world->robots.back(),
                random_robot_pos2(world->field_geometry));
  // TODO: robot position
}

unsigned int world_get_frame_number(const World *world) {
  return world->frame_number;
}

double world_get_timestamp(const World *world) { return world->timestamp; }

btDiscreteDynamicsWorld *world_bt_dynamics(struct World *world) {
  return &world->dynamics;
}

// ball.h impl
Vec3 ball_get_vec(const Ball *ball) {
  auto trans = ball->body.getWorldTransform();
  auto orig = trans.getOrigin();
  return {orig.getX(), orig.getY(), orig.getZ()};
}

void ball_set_vec(Ball *ball, const Vec2 vec) {
  auto transf = btTransform({0, 0, 0, 1}, {vec.x, vec.y, DROP_HEIGHT});
  ball->body.setWorldTransform(transf);
  ball->body.activate(true);
}

Pos3 ball_get_pos(const Ball *ball);

void ball_set_pos(Ball *ball, const Pos3 pos);

Pos3 ball_get_vel(const Ball *ball);

void ball_set_vel(Ball *ball, const Pos3 vel);

int ball_is_touching_robot(const Ball *ball, const Robot *robot);

Robot *ball_last_touching_robot(Ball *ball);

/// fast squared speed (magnitude of velocity)
Float ball_get_speed2(const Ball *ball);

/// fast squared speed (magnitude of velocity)
Float ball_get_peak_speed2_from_last_kick(const Ball *ball);

btRigidBody *ball_bt_rigid_body(Ball *ball) { return &ball->body; }

// robot.h impl
int get_id(const Robot *robot) { return robot->id; }

enum Team get_team(const Robot *robot) { return robot->team; }

Pos2 robot_get_pos(const Robot *robot) {
  auto trans = robot->vehicle.getRigidBody()->getWorldTransform();
  auto orig = trans.getOrigin();
  auto rot = trans.getRotation();
  auto sign = rot.getAxis().getZ() > 0 ? 1 : -1;
  return {orig.getX(), orig.getY(), sign * rot.getAngle()};
}

void robot_set_pos(Robot *robot, const Pos2 pos) {
  auto transf = btTransform({{0, 0, 1}, pos.w}, {pos.x, pos.y, DROP_HEIGHT});
  robot->vehicle.getRigidBody()->setWorldTransform(transf);
  // robot->body.setWorldTransform(transf);
  // robot->body.activate(true);
}

Pos2 robot_get_vel(const Robot *robot);

void robot_set_vel(Robot *robot, const Pos2 vel);

/// return is C bool (1 for true, 0 for false)
int robot_is_touching_robot(const Robot *robot, const Robot *tobor);

/// fast squared speed (magnitude of velocity)
Float robot_get_speed2(const Robot *robot);

} // end extern
