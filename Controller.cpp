#include "Controller.hpp"
using namespace dart;
using namespace std;

#define STEER_LIM  0.6

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

  // Default wheel torque and steering angle values
  mTorque = 0;
  mSteeringAngle = 0;
}

//=========================================================================
Controller::~Controller() {}

//=========================================================================
void Controller::update()
{
  // Increase the step counter
  mSteps++;

  // Set force vector
  mForces(0) = mTorque;
  mForces(1) = mTorque;
  mForces(2) = mTorque/2;
  mForces(3) = mTorque/2;

  // Define subset of generalized coordinates for force to be applied
  const vector<size_t> index{8, 11, 13, 15};

  // Apply forces
  mRobot->setForces(index, mForces);

  // Set steering angle in the vehicle turn joints
  mRobot->getDof(7)->setPosition(mSteeringAngle);
  mRobot->getDof(10)->setPosition(mSteeringAngle);

  // Limit steering angle
  if(mSteeringAngle > STEER_LIM){
    mSteeringAngle = STEER_LIM;
  }
  else if (mSteeringAngle < -STEER_LIM){
    mSteeringAngle = -STEER_LIM;
  }

  // Simulate castor force at front wheels
  mSteeringAngle -= mRobot->getDof(7)->getPosition()*0.0005;
}

//=========================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const {
  return mRobot;
}


//=========================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {
}