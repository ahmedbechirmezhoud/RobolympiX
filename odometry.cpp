/*
 * odometry.cpp
 */

#include "odometry.h"
#include <math.h>
#include "settings.h"



float degree2miliradian(float degree ){
	return 90 * 1000 * M_PI / 180;
}

/**
 returns the angle of the given coordinates relative to the current position
 @param vector<float> coordinates contains X and Y of a 2d vector
 @return sign(X) * arctan(X/Y) + ( 0 si Y > 0 sinon 1570.796)
 */
float relativeAngle(state currentState, vector<float> coordinates){
	return sign(currentState.position[0] - coordinates[0]) * atan(currentState.position[0] - coordinates[0] / currentState.position[1] - coordinates[1] )
			+ (sign(currentState.position[1] - coordinates[1]) ? 0 : degree2miliradian(90) );
}


/**
 * Sum robot vector with translation vector to get the translation absolute coordinates
 */
vector<float> relative2absolute(state currentState, vector<float> translation){
	return vector<float>{currentState.position[0] + translation[0], currentState.position[1] + translation[1]};
}

vector<float> absolute2relative(state currentState, vector<float> coordinates){
	return vector<float>{currentState.position[0] - coordinates[0], currentState.position[1] - coordinates[1]};
}

/**
 returns the angle of the given coordinates
 @param vector<float> coordinates contains X and Y of a 2d vector
 @return sign(X) * arctan(X/Y) + ( 0 si Y > 0 sinon 1570.796)
 */
float absoluteAngle(vector<float> coordinates){
	return sign(coordinates[0]) * atan(coordinates[0] / coordinates[1])
			+ (sign(coordinates[1]) ? 0 : degree2miliradian(90) )	;
}

float radian2milliradian(float radian){
	return 1000 * radian;
}

float milliradian2radian(float mrad){
	return 0.001 * mrad;
}


//initialize the position vector
vector<float> position = {X_INIT,Y_INIT};

//previous DATA
vector<float> prevDistance = {0,0};

/**
 returns the new State after considering the new and the previous sensed values
 @params distanceL the distance from the left encoder
 @params distanceL the distance from the rignt encoder
 @return state the current Sensed state
 */
state odometry(float distanceL, float distanceR){

	//the difference between the two value is the arc created by the rotation of the robot
	// arc = angle * radius -> angle = arc / radius
	angle = (distanceL - distanceR) / (ENTRAXE/2);

	//to simplify the robot is considered as a point in the 2D space
	//then, we consider the middle point of entraxe as the Robot coordinates
	//we want to calculate the Variation of the point's position
	float modulus = ( (distanceL + distanceR) - (prevDistance[0] + prevDistance[1]) )  / 2 ;

	//update previous distances value
	prevDistance[0] = distanceL;
	prevDistance[1] = distanceR;

	//project the result on the X and Y axis
	//and add the variation for each axis
	position[0] += modulus * sin(milliradian2radian(angle)); // X
	position[1] += modulus * cos(milliradian2radian(angle));  // Y

	//return the new values
	state sensedState = {
		position,
		angle
	};

	return sensedState;
}

