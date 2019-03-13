#ifndef CONTROLLER_HPP_
#define CONTROLLER_HPP_

#include <Eigen/Eigen>
#include <string>
#include <dart/dart.hpp>
#include <string>
#include <iostream>

using namespace dart;

/// \brief Operational space controller for 6-dof manipulator
class Controller {
public:
  /// \brief Constructor
  Controller(dart::dynamics::SkeletonPtr _robot);

  /// \brief Destructor
  virtual ~Controller();

  /// \brief
  void update();

  /// \brief Get robot
  dart::dynamics::SkeletonPtr getRobot() const;

  /// \brief Keyboard control
  virtual void keyboard(unsigned char _key, int _x, int _y);

//private:
  /// \brief Robot
  dart::dynamics::SkeletonPtr mRobot;

  /// \brief Control forces
  // Eigen::VectorXd<double, 4> mForces;
  Eigen::VectorXd mForces;

  size_t mSteps;

  double mTorque;

  double mSteeringAngle;

  Eigen::Matrix<double, 4, 4> mBaseTf;
};

#endif  // EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
