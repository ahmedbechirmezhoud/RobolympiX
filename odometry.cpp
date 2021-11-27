/*
 * odometry.cpp
 */

#include "odometry.h"
#include <math.h>
#include "settings.h"



float degree2radian(float degree ){
	return degree * M_PI / 180;
}

float radian2degree(float rad ){
	return rad * (180 / M_PI) ;
}

float degree2miliradian(float deg){
	return radian2milliradian(degree2radian(deg));
}

/**
 returns the angle of the given coordinates relative to the current position
 @param currentState the current state of the robot
 @param coordinates the coordinates of the desired postion
 @return angle that will make the robot face the desired point
 */
float relativeAngle(state currentState, vector<float> coordinates){

	// if the two coordinates are on the same axe return 90
	if ((coordinates[1] - currentState.position[1]) == 0)
		return sign(coordinates[0] - currentState.position[0]) * degree2radian(90);
	
	// calculate the angle of the shortest path to the desired point from the current position of the robot
	float angle =  sign(coordinates[0] - currentState.position[0]) * atan(-(coordinates[0] - currentState.position[0]) / (coordinates[1] - currentState.position[1]));

	// if the resulted angle is the opposite direction
	if (sign(coordinates[1] - currentState.position[1]) == -1)
		angle = sign(coordinates[0] - currentState.position[0]) * M_PI - angle;
	
	return angle;

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
			+ (sign(coordinates[1]) ? 0 : radian2milliradian(degree2radian(90)) )	;
}


float radian2milliradian(float radian){
	return 1000 * radian;
}


float milliradian2radian(float mrad){
	return 0.001 * mrad;
}

/**
 Map any angle to the interval [-180°, 180°]
 @param mrad and angle in milliradian
 @return angle converted to the interval in milliradian
 */
float mapAngle(float mrad){
	if(mrad > degree2miliradian(180))
		return mrad - degree2miliradian(360);
	if(mrad <= degree2miliradian(-180))
		return mrad + degree2miliradian(360);
	return mrad;
}



//initialize the position vector
vector<float> position = {X_INIT,Y_INIT};

//previous DATA
vector<float> prevDistance = {0,0};

/**
 returns the new State after considering the new and the previous sensed values
 @param distanceL the distance from the left encoder
 @param distanceL the distance from the rignt encoder
 @return state the current Sensed state
 */
state odometry(float distanceL, float distanceR){

	//the difference between the two value is the arc created by the rotation of the robot
	// arc = angle * radius -> angle = arc / radius
	// then map the angle to the correct interval [-180°, 180°]
	float angle = mapAngle((distanceR - distanceL) / ENTRAXE);


	//to simplify the robot is considered as a point in the 2D space
	//then, we consider the middle point of entraxe as the Robot coordinates
	//we want to calculate the Variation of the point's position
	float modulus = ( (distanceL + distanceR) - (prevDistance[0] + prevDistance[1]) )  / 2 ;

	//update previous distances value
	prevDistance[0] = distanceL;
	prevDistance[1] = distanceR;

	//project the result on the X and Y axis
	//and add the variation for each axis
	position[0] -= modulus * sin(angle); // X
	position[1] += modulus * cos(angle);  // Y

	//return the new values
	state sensedState = {
		position,
		radian2milliradian(angle)
	};

	return sensedState;
}

