/*
 * Path.cpp
 */

#include "Path.h"
#include "settings.h"
#include "PID.h"
#include <math.h>
#include "odometry.h"
#include "Robot.h"


Path::Path(Robot robot){
	vector<float> a{X_INIT, Y_INIT};
	this->currentState.position = a;
	this->currentState.angle = ALPHA_INIT;
	this->robot = robot;
	initPID(currentState);
	time_var = HAL_GetTick();
	dt = 0;
}

Path::~Path() {
	// TODO Auto-generated destructor stub
}

void Path::updateTime(){
	dt = HAL_GetTick() - time_var;
	time_var = HAL_GetTick();
}

int Path::updateState(){

	state sensedState = odometry(robot.getDistanceL(), robot.getDistanceR());
	updateTime();
	set_processVariable(sensedState, dt);

	if(linearReached() && angularReached()){

		job reachedTarget = predefinedPath.front();
		predefinedPath.pop();

		reachedTarget.task();
		
		switch(predefinedPath.front().type){

			case alpha:
				set_SetPoint(reachedTarget.absoluteCoordinates, predefinedPath.front().angle);
				break;

			case xyAbsolute:
				set_SetPoint(predefinedPath.front().absoluteCoordinates, relativeAngle(currentState, absolute2relative(currentState, reachedTarget.absoluteCoordinates)) );
				break;

			case xyRelative:
				predefinedPath.front().absoluteCoordinates = relative2absolute(currentState, predefinedPath.front().translation);
				set_SetPoint(predefinedPath.front().translation, relativeAngle(currentState, reachedTarget.translation) );
				break;

			case stop:
				set_SetPoint(reachedTarget.absoluteCoordinates, reachedTarget.angle);
				break;

			default:
				return 1;
		};

	}

	vector<int> command = controller();
	robot.setMotorL(command[0]);
	robot.setMotorR(command[1]);

	return 0;
}

void Path::setPredefinedPath(queue<job> path){
	this->predefinedPath = path;
}

queue<job> Path::getPredefinePath(){
	return predefinedPath;
};
state Path::getCurrentState(){
	return currentState;
};

