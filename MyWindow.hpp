#ifndef MYWINDOW_HPP_
#define MYWINDOW_HPP_

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>

#include "Controller.hpp"
#include "StereoCam.hpp"

class MyWindow : public dart::gui::SimWindow {

  public:
    MyWindow(const dart::simulation::WorldPtr& world);

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
    dart::dynamics::SkeletonPtr mVehicle;

    // Controller
    Controller* mController;

    StereoCam* mStereoCam;

    double mRadius;

    int mSteps;
};

#endif