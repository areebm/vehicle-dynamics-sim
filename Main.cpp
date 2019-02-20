#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>
#include "Controller.hpp"
#include "StereoCam.hpp"

using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

ofstream q_out_file("../data/q_out.txt");
ofstream t_file("../data/time.txt");

class MyWindow : public dart::gui::SimWindow {

  public:
    MyWindow(const WorldPtr& world) {
      
      // Attach the world passed in the input argument to the window
      setWorld(world);
      // Get vehicle skeleton from world to pass to Controller

      mVehicle = world->getSkeleton("vehicle");

      mRadius = 0.2;      

      int dof = mVehicle->getNumDofs();

      for(int i = 0; i < dof-5; i++){
        cout << mVehicle->getJoint(i)->getName() << endl;
      }

      mController = new Controller(mVehicle);

      // TODO: Add camera location and rotation parameters in constructor
      mStereoCam = new StereoCam();
    }

    // Draw things in the world (e.g. spheres at COMs)
    void drawWorld() const override;

    // Get keyboard input
    void keyboard(unsigned char _key, int _x, int _y) override;

    Eigen::Vector3d getBodyCOM(dart::dynamics::SkeletonPtr robot);

    // Record data to files if called in timestep
    void recordData();

    // Define timestep inclusive of Controller and StereoCam updates
    void timeStepping() override;

    ~MyWindow() {}

  protected:

    // Full robot
    SkeletonPtr mVehicle;

    // Controller
    Controller* mController;

    StereoCam* mStereoCam;

    double mRadius;

    int mSteps;
};

// void dart::gui::RenderInterface::setViewport(0,0,200,100);

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
  //Currently makes the robot spin in the air
  mController->update();

  //update stereo camera here
  //mStereoCam->update();

  SimWindow::timeStepping();
}

//====================================================================
SkeletonPtr createFloor() {
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double floor_width = 50;
  double floor_height = 0.05;
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  auto shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height*20.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

SkeletonPtr createLedge() {
   SkeletonPtr ledge = Skeleton::create("ledge");

  // Give the ledge a body
  BodyNodePtr body =
      ledge->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

  // Give the body a shape
  double ledge_width = 5;
  double ledge_length = 4;
  double ledge_height = 0.05;
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(ledge_length, ledge_width, ledge_height)));
  auto shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Red());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(2.05, 0.0, -ledge_height*17.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return ledge; 
}



//====================================================================
dart::dynamics::SkeletonPtr createVehicle() {

  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr vehicle;
 
  Eigen::Matrix<double, 4, 1> initPoseParams; // heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect, qLArm0, ... qLArm6, qRArm0, ..., qRArm6
  double initVehicleSpeed;
  Eigen::Vector3d COM;

  // Load the Skeleton from a file
  vehicle = loader.parseSkeleton("/home/areeb/dart/vehicle-dynamics-sim/simpleVehicle.urdf");
  vehicle->setName("vehicle");

  // Set the positions and get the resulting COM angle
  // vehicle->setPositions(q);
  // COM = vehicle->getCOM();

  //Set Damping Coefficients of every wheel
  vehicle->getJoint(0)->setDampingCoefficient(0, 0.5);
  vehicle->getJoint(1)->setDampingCoefficient(0, 0.5);
  vehicle->getJoint(2)->setDampingCoefficient(0, 0.5);
  vehicle->getJoint(3)->setDampingCoefficient(0, 0.5);

  return vehicle;
}

//====================================================================
int main(int argc, char* argv[]) {

  // Create world
  WorldPtr world = std::make_shared<World>();

  // Load Floor
  SkeletonPtr floor = createFloor();
  world->addSkeleton(floor);
  cout << "Floor Loaded" << endl;

  // Load Ledge
  SkeletonPtr ledge = createLedge();
  world->addSkeleton(ledge);
  cout << "Ledge Loaded" << endl;

  // Load robot
  SkeletonPtr robot = createVehicle();
  world->addSkeleton(robot);
  cout << "Vehicle Loaded" << endl;

  // Create window
  MyWindow window(world);

  // Run the world
  glutInit(&argc, argv);
  window.initWindow(1280,720, "Simple Vehicle Dynamics");
  glutMainLoop();

  //OpenGL to save image to file (See implementation in OpenGlRenderInterface)
  // "render to an off-screen buffer"
  // "set up OpenGL to render to a texture, read the resulting texture into..."
  // "... main memory and save it to a file"

  return 0;
}
