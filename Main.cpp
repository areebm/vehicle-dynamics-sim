#include "MyWindow.hpp"

using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

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
