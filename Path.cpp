/*
 * Path.cpp
 */

#include "Path.h"
#include "settings.h"
#include "PID.h"
#include <math.h>
#include "odometry.h"
#include "Robot.h"
#include <queue>


Path::Path(Robot robot){
	vector<float> a{X_INIT, Y_INIT};
	this->currentState.position = a;
	this->currentState.angle = ALPHA_INIT;
	this->robot = robot;
	initPID(currentState);


	this->robot = robot;

	job init;
	init.absoluteCoordinates = this->currentState.position;
	init.angle = this->currentState.angle;
	init.type = xyAbsolute;
	this->predefinedPath.push(init);
	
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

	if( linearReached() && angularReached() && !predefinedPath.empty()){
		job reachedPoint = predefinedPath.front();
		predefinedPath.pop();

		if (!predefinedPath.empty()){

			initError();

			if(predefinedPath.front().type == xyAbsolute)
				set_SetPoint(predefinedPath.front().absoluteCoordinates, predefinedPath.front().angle);

			if(predefinedPath.front().type == alpha)
				set_SetPoint(reachedPoint.absoluteCoordinates , predefinedPath.front().angle);

			if(predefinedPath.front().type == xyRelative){
				predefinedPath.front().absoluteCoordinates = relative2absolute(currentState, predefinedPath.front().translation);
				set_SetPoint(predefinedPath.front().absoluteCoordinates, NAN);
			}
		}
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

