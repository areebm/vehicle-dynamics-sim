/*
*	Areeb Mehmood
*	February 2019
*/

#include "StereoCam.hpp"

// #include "dart/gui/LoadOpengl.hpp"
// #include "dart/gui/LoadGlut.hpp"

using namespace dart;
using namespace std;

StereoCam::StereoCam(const dart::simulation::WorldPtr& worldPtr){

	mFrame = 0;

	
}

StereoCam::~StereoCam(){}

void StereoCam::update(){

	// Make sure pointer is not null
	// assert(worldPtr);
	mFrame++;
	// Update positions and orientations here

	// cout << "mFrame:" << mFrame << endl;
	//Save screenshot every N frames (Suggest N = 100)
	
}

// Resize window?
void StereoCam::resize(int _w, int _h){
	// nonimplemented
}

// Render the view from the camera
void StereoCam::render(){
	// nonimplemented
}

// Get keyboard input
void StereoCam::keyboard(unsigned char _key, int _x, int _y){
	switch (_key) {

    case 'r':
      glRotated(30, 1, 0, 0);
	break;

	default:
      // Default keyboard control
      SimWindow::keyboard(_key, _x, _y);
	break;
	}
	// glutPostRedisplay();
}

