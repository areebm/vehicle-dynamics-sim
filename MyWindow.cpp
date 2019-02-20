#include "MyWindow.hpp"

using namespace std;
using namespace dart;

ofstream q_out_file("../data/q_out.txt");
ofstream t_file("../data/time.txt");

void MyWindow::recordData() {
  q_out_file << mWorld->getSkeleton("vehicle")->getJoint("Chassis_FRUpright")->getPosition(0) <<
    "   " << mWorld->getSkeleton("vehicle")->getJoint("Chassis_FLUpright")->getPosition(0) <<
    "   " << mWorld->getSkeleton("vehicle")->getJoint("Chassis_RRUpright")->getPosition(0) <<
    "   " << mWorld->getSkeleton("vehicle")->getJoint("Chassis_RLUpright")->getPosition(0) << endl;
  t_file << mSteps << endl;
}

//====================================================================
void MyWindow::drawWorld() const {

  // mRI is the member Render Interface, initialized by default (?)
  if(mRI){

    // Draw spherical ellipsoid at COM of Chassis in red
    mRI->setPenColor(Eigen::Vector3d(1, 0.1, 0.1));
    mRI->pushMatrix();
    mRI->translate(mWorld->getSkeleton("vehicle")->getBodyNode("Chassis")->getCOM());
    mRI->drawEllipsoid(Eigen::Vector3d(mRadius*1.5, mRadius*1.5, mRadius*1.5));
    mRI->popMatrix();

    // Draw ellipsoids at COM of Wheels in blue
    // mRI->setPenColor(Eigen::Vector3d(0.1, 0.1, 1));    
    // mRI->pushMatrix();
    // mRI->translate(mWorld->getSkeleton("vehicle")->getBodyNode("FLWheel")->getCOM());
    // mRI->drawEllipsoid(Eigen::Vector3d(mRadius*0.5, mRadius*1.5, mRadius*0.5));
    // mRI->popMatrix();

    // mRI->pushMatrix();
    // mRI->translate(mWorld->getSkeleton("vehicle")->getBodyNode("FRWheel")->getCOM());
    // mRI->drawEllipsoid(Eigen::Vector3d(mRadius*0.5, mRadius*1.5, mRadius*0.5));
    // mRI->popMatrix();
    
    // mRI->pushMatrix();
    // mRI->translate(mWorld->getSkeleton("vehicle")->getBodyNode("RLWheel")->getCOM());
    // mRI->drawEllipsoid(Eigen::Vector3d(mRadius*0.5, mRadius*1.5, mRadius*0.5));
    // mRI->popMatrix();
    
    // mRI->pushMatrix();
    // mRI->translate(mWorld->getSkeleton("vehicle")->getBodyNode("RRWheel")->getCOM());
    // mRI->drawEllipsoid(Eigen::Vector3d(mRadius*0.5, mRadius*1.5, mRadius*0.5));
    // mRI->popMatrix();
  }

  // Draw world
  SimWindow::drawWorld();
}

//====================================================================
void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
  double incremental = 0.01;

  switch (_key) {

    case 'w':
      // Forward
      mController->mTorque = 30;
      break;

    case 's':
      // Backward
      mController->mTorque = -30;
      break;

    case 'b':
      // "Brake"
      mController->mTorque = 0;
      break;

    case 'a':
      // Turn Left
      mController->mSteeringAngle += 0.0135;
      break;

    case 'd':
      // Turn Right
      mController->mSteeringAngle -= 0.0135;
      break;

    default:
      // Default keyboard control
      mController->mTorque = 0;

      // Send Steering Angle to zero. 
      // TODO: Replace with PID controller in the future with thetaRef = 0
      if(mController->mSteeringAngle > 0){
        mController->mSteeringAngle -= 0.1;
      }
      else if(mController->mSteeringAngle < 0){
        mController->mSteeringAngle += 0.1;
      }
      SimWindow::keyboard(_key, _x, _y);
      break;
  }

  // Keyboard control for Controller
  mController->keyboard(_key, _x, _y);
}

//====================================================================
Eigen::Vector3d MyWindow::getBodyCOM(dart::dynamics::SkeletonPtr robot) {
  double fullMass = robot->getMass();
  double wheelMass = robot->getBodyNode("LWheel")->getMass();
  return (fullMass*robot->getCOM() - wheelMass*robot->getBodyNode("LWheel")->getCOM() - wheelMass*robot->getBodyNode("RWheel")->getCOM())/(fullMass - 2*wheelMass);
}

//====================================================================
void MyWindow::timeStepping() {
  mSteps++;

  // recordData();
  mController->update();

  //update stereo camera here
  mStereoCam->update();

  SimWindow::timeStepping();
}