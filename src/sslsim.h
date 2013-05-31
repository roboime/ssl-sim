/*
 * Small Size League Simulator (Experimental) (c) RoboIME
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose, 
 * including commercial applications, and to alter it and redistribute it freely, 
 * subject to the following restrictions:
 *
 * This is a modified version from the DynamicControlDemo shipped with bullet 2.81
 */

#ifndef SSLSIM_H
#define SSLSIM_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class SSLSim : public GlutDemoApplication
{
 public:
  void initPhysics();

  void exitPhysics();

  virtual ~SSLSim()
  {
    exitPhysics();
  }

  void spawnTestRig(const btVector3& startOffset, bool bFixed);

  virtual void clientMoveAndDisplay();

  virtual void displayCallback();

  virtual void keyboardCallback(unsigned char key, int x, int y);

  static DemoApplication* Create()
  {
    SSLSim* demo = new SSLSim();
    demo->myinit();
    demo->initPhysics();
    return demo;
  }

  void setMotorTargets(btScalar deltaTime);

 private:
  float m_Time;
  float m_fCyclePeriod; // in milliseconds
  float m_fMuscleStrength;

  btAlignedObjectArray<class TestRig*> m_rigs;

  //keep the collision shapes, for deletion/cleanup
  btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

  btBroadphaseInterface* m_broadphase;

  btCollisionDispatcher* m_dispatcher;

  btConstraintSolver* m_solver;

  btDefaultCollisionConfiguration* m_collisionConfiguration;
};


#endif
