#include "MyWindow.hpp"

using namespace std;
using namespace dart;

dart::dynamics::BodyNode* chassisNode;
dart::dynamics::BodyNode* FRWheelNode;

Eigen::Matrix3d chassisRot;
Eigen::Vector3d chassisTrans;

Eigen::Isometry3d testIso;

ofstream q_out_file("../data/q_out.txt");
ofstream t_file("../data/time.txt");
ofstream rot_file("../data/rot.txt");

MyWindow::MyWindow(const dart::simulation::WorldPtr& world) {
      
      // Attach the world passed in the input argument to the window
      // World is a class and SimWindow is a class. SimWindow(dart::gui::SimWindow) has method
      // setWorld, taking type World (dart::simulation::World).
      setWorld(world);
      // Get vehicle skeleton from world to pass to Controller

      mVehicle = world->getSkeleton("vehicle");

      mRadius = 0.2;    

      int dof = mVehicle->getNumDofs();

      for(int i = 0; i < dof-5; i++){
        cout << mVehicle->getJoint(i)->getName() << endl;
      }

      chassisNode = mVehicle->getBodyNode(0); // Chassis
      FRWheelNode = mVehicle->getBodyNode(3); // FRWheel

      mController = new Controller(mVehicle);
}

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
  // recordData();

  // Output translation of chassis
  // chassisTrans = chassisNode->getTransform().translation();
  /*chassisTrans = FRWheelNode->getTransform(chassisNode).translation();
  cout << endl;
  cout << "[ " << chassisTrans(0) << " " << chassisTrans(1) << " " << chassisTrans(2) << " ]" << endl;
  
  testIso = FRWheelNode->getTransform();
  cout << "[ " << testIso(0,0) << " " << testIso(0,1) << " " << testIso(0,2) << " ]" << endl
       << "[ " << testIso(1,0) << " " << testIso(1,1) << " " << testIso(1,2) << " ]" << endl
       << "[ " << testIso(2,0) << " " << testIso(2,1) << " " << testIso(2,2) << " ]" << endl << endl;
  
  chassisRot = FRWheelNode->getTransform(chassisNode).rotation();
  cout << "[ " << chassisRot(0,0) << " " << chassisRot(0,1) << " " << chassisRot(0,2) << " ]" << endl
       << "[ " << chassisRot(1,0) << " " << chassisRot(1,1) << " " << chassisRot(1,2) << " ]" << endl
       << "[ " << chassisRot(2,0) << " " << chassisRot(2,1) << " " << chassisRot(2,2) << " ]" << endl;*/

  mSteps++;

  mController->update();
  
  SimWindow::timeStepping();
}