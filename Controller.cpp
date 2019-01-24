/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.hpp"
using namespace dart;
using namespace std;

//==========================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot)
  :mRobot(_robot)
{

  // Check that a null SkeletonPtr was not passed 
  // (i.e. make sure MyWindow class found robot in the world)
  assert(_robot != nullptr);

  int dof = mRobot->getNumDofs();
  mForces.setZero(4);
  mSteps = 0;

  // Get pointers to each wheel Joint
  // dart::dynamics::BodyNode* FLWheel = mRobot->getBodyNode("FLWheel");
  // dart::dynamics::BodyNode* FRWheel = mRobot->getBodyNode("FRWheel");
  // dart::dynamics::BodyNode* RLWheel = mRobot->getBodyNode("RLWheel");
  // dart::dynamics::BodyNode* RRWheel = mRobot->getBodyNode("RRWheel");

  double k = 500;
  double c = 5;

  // Set Damping, Spring Stiffness, PositionLimitsEnforced
  // Wheels, Turning
  mRobot->getJoint("FLUpright_FLTurn")->setPositionLimitEnforced(false);
  mRobot->getJoint("FLTurn_FLWheel")->setPositionLimitEnforced(false);
  mRobot->getJoint("FRUpright_FRTurn")->setPositionLimitEnforced(false);
  mRobot->getJoint("FRTurn_FRWheel")->setPositionLimitEnforced(false);
  mRobot->getJoint("RRUpright_RRWheel")->setPositionLimitEnforced(false);
  mRobot->getJoint("RLUpright_RLWheel")->setPositionLimitEnforced(false);
  //Front Uprights
  mRobot->getJoint("Chassis_FLUpright")->setPositionLimitEnforced(true);
  mRobot->getJoint("Chassis_FLUpright")->setDampingCoefficient(0,c);
  mRobot->getJoint("Chassis_FLUpright")->setSpringStiffness(0,k);
  mRobot->getJoint("Chassis_FRUpright")->setPositionLimitEnforced(true);
  mRobot->getJoint("Chassis_FRUpright")->setDampingCoefficient(0,c);
  mRobot->getJoint("Chassis_FRUpright")->setSpringStiffness(0,k);
  // Rear Uprights
  mRobot->getJoint("Chassis_RLUpright")->setPositionLimitEnforced(true);
  mRobot->getJoint("Chassis_RLUpright")->setDampingCoefficient(0,c);
  mRobot->getJoint("Chassis_RLUpright")->setSpringStiffness(0,k);
  mRobot->getJoint("Chassis_RRUpright")->setPositionLimitEnforced(true);
  mRobot->getJoint("Chassis_RRUpright")->setDampingCoefficient(0,c);
  mRobot->getJoint("Chassis_RRUpright")->setSpringStiffness(0,k);

  cout << "DOF: " << dof << endl;


  // Default wheel torque and steering angle values
  mTorque = 0;
  mSteeringAngle = 0;
}

//=========================================================================
Controller::~Controller() {}

//=========================================================================
void Controller::update()
{
  // increase the step counter
  mSteps++;

  // mForces(0) = torq;
  // mForces(1) = torq;
  // mForces(2) = torq;
  // mForces(3) = torq;

  // const vector<size_t> index{2, 4, 6, 8};

  // mRobot->setForces(index, mForces);
  // mRobot->getDof(9)->setForce(mTorque);
  // mRobot->getDof(12)->setForce(mTorque);


  // Limit steering angle to +/- 45 degrees
  if(mSteeringAngle > 0.6){
    mSteeringAngle = 0.6;
  }
  else if (mSteeringAngle < -0.6){
    mSteeringAngle = -0.6;
  }

  // Set Torque on rear wheel joints of car
  mRobot->getDof(13)->setForce(mTorque);
  mRobot->getDof(15)->setForce(mTorque);
  mRobot->getDof(8)->setForce(mTorque/2);
  mRobot->getDof(11)->setForce(mTorque/2);

  // Set Steering Angle in the vehicle turn joints
  mRobot->getDof(7)->setPosition(mSteeringAngle);
  mRobot->getDof(10)->setPosition(mSteeringAngle);

  cout << "Torque: " << mTorque << "   Steering Angle: " << mSteeringAngle << endl;
}

//=========================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const {
  return mRobot;
}


//=========================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {
}