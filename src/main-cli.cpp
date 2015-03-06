/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <iostream>
#include <signal.h>
#include <stdlib.h>

#include <btBulletDynamicsCommon.h>
//#include <bullet/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h>

//
// How To Scale The World
// ======================
//
// Obviously any collision shapes must be scaled appropriately,
// but there are also some other things that need to be changed.
// In general, if you are scaling the world by a factor of X
// then you must do the following:
//
// - Scale collision shapes about origin by X
// - Scale all positions by X
// - Scale all linear (but not angular) velocities by X
// - Scale linear [Sleep Threshold] by X
// - Scale gravity by X
// - Scale all impulses you supply by X
// - Scale all torques by X^2
// - Scale all inertias by X if not computed by Bullet
//

bool keep_going = true;
void sigint_handler(int s);

int main(int argc, char *argv[]) {
  std::cout << "Hello World!" << std::endl;

  // Handle SIGINT (Ctrl+C)
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = sigint_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // Build the broadphase
  btBroadphaseInterface *broadphase = new btDbvtBroadphase();

  // Set up the collision configuration and dispatcher
  btDefaultCollisionConfiguration *collisionConfiguration =
      new btDefaultCollisionConfiguration();
  btCollisionDispatcher *dispatcher =
      new btCollisionDispatcher(collisionConfiguration);
  // btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

  // The actual physics solver
  btSequentialImpulseConstraintSolver *solver =
      new btSequentialImpulseConstraintSolver;

  // The world
  btDiscreteDynamicsWorld *dynamicsWorld = new btDiscreteDynamicsWorld(
      dispatcher, broadphase, solver, collisionConfiguration);
  dynamicsWorld->setGravity(btVector3(0, 0, -980.f));

  // Create the ground
  btCollisionShape *groundShape = new btStaticPlaneShape(btVector3(0, 0, 1), 1);
  btDefaultMotionState *groundMotionState = new btDefaultMotionState(
      btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, -1)));
  btRigidBody::btRigidBodyConstructionInfo groundRigidBodyCI(
      0, groundMotionState, groundShape, btVector3(0, 0, 0));
  btRigidBody *groundRigidBody = new btRigidBody(groundRigidBodyCI);
  dynamicsWorld->addRigidBody(groundRigidBody);

  // Create a falling sphere
  btCollisionShape *fallShape = new btSphereShape(2.15); // 43mm diameter
  btDefaultMotionState *fallMotionState = new btDefaultMotionState(
      btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 5000)));
  btScalar fallMass = 0.046; // 46g
  btVector3 fallInertia(0, 0, 0);
  fallShape->calculateLocalInertia(fallMass, fallInertia);
  btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(
      fallMass, fallMotionState, fallShape, fallInertia);
  btRigidBody *fallRigidBody = new btRigidBody(fallRigidBodyCI);
  dynamicsWorld->addRigidBody(fallRigidBody);

  // Create a cylinder
  btCollisionShape *cylShape = new btCylinderShapeZ(
      btVector3(9, 9, 7.5)); // 180mm diameter, 150mm height
  btDefaultMotionState *cylMotionState = new btDefaultMotionState(
      btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 0, 8)));
  btScalar cylMass = 2; // 2kg
  btVector3 cylInertia(0, 0, 0);
  cylShape->calculateLocalInertia(cylMass, cylInertia);
  btRigidBody::btRigidBodyConstructionInfo cylRigidBodyCI(
      cylMass, cylMotionState, cylShape, cylInertia);
  btRigidBody *cylRigidBody = new btRigidBody(cylRigidBodyCI);
  dynamicsWorld->addRigidBody(cylRigidBody);

  // Step the simulation a few times
  for (int i = 0; i < 240; i++) {
    dynamicsWorld->stepSimulation(1 / 60.f, 10, 1 / 600.f);
    btTransform trans;

    fallRigidBody->getMotionState()->getWorldTransform(trans);
    std::cout << "sphere: (" << trans.getOrigin().getX() << ", "
              << trans.getOrigin().getY() << ", " << trans.getOrigin().getZ()
              << ")" << std::endl;

    // cylRigidBody->getMotionState()->getWorldTransform(trans);
    // std::cout
    //  << "cylinder: ("
    //  << trans.getOrigin().getX() << ", "
    //  << trans.getOrigin().getY() << ", "
    //  << trans.getOrigin().getZ() << ")"
    //  << std::endl;
  }

  // while (keep_going) {
  //}

  // Clean this mess up
  dynamicsWorld->removeRigidBody(cylRigidBody);
  delete cylRigidBody;
  delete cylMotionState;
  delete cylShape;

  dynamicsWorld->removeRigidBody(fallRigidBody);
  delete fallRigidBody;
  delete fallMotionState;
  delete fallShape;

  dynamicsWorld->removeRigidBody(groundRigidBody);
  delete groundRigidBody;
  delete groundMotionState;
  delete groundShape;

  delete dynamicsWorld;
  delete solver;
  delete dispatcher;
  delete collisionConfiguration;
  delete broadphase;

  std::cout << "Done." << std::endl;

  return 0;
}

void sigint_handler(int s) {
  std::cout << std::endl << "Aborting..." << std::endl;
  keep_going = false;
}
