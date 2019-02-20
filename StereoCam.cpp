/*
*	Areeb Mehmood
*	February 2019
*/

#include "StereoCam.hpp"
using namespace dart;
using namespace std;

StereoCam::StereoCam(){

	mFrame = 0;

	// Set initial positions and orientations here
	
}

StereoCam::~StereoCam(){}

void StereoCam::update(){
	mFrame++;
	// Update positions and orientations here

	cout << "mFrame:" << mFrame << endl;
	//Save screenshot every N frames
	
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
	// nonimplemented
}

