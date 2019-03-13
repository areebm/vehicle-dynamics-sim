/*
*	Areeb Mehmood
*	February 2019
*/

#ifndef STEREOCAM_HPP_
#define STEREOCAM_HPP_

#include <string>
#include <iostream>

#include <Eigen/Eigen>

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>

using namespace std;
using namespace dart;
// using namespace dart::common;
// using namespace dart::math;

class StereoCam : public dart::gui::SimWindow { //Might need to extend SimWindow <- Win3D <- GlutWindow
	
	public:
		// Constructor
		// TODO: Take two points as camera location targets
		StereoCam(const dart::simulation::WorldPtr& worldPtr);

		// Desctructor
		virtual ~StereoCam();

		// Update function for updating positions/orientations
		void update();

		/// \brief Keyboard control
  		void keyboard(unsigned char _key, int _x, int _y);

  		// These are pure virtual functions inherited from GlutWindow
  		// Pure virtual methods must be implemented
  		void resize(int _w, int _h);

  		void render();

	protected:
		int mFrame;
};

#endif //STEREOCAM_HPP_