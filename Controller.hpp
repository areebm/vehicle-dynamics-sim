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
