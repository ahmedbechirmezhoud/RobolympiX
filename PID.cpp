/*
 * PID.cpp
 */


#include "PID.h"
#include "settings.h"
#include <math.h>
#include "odometry.h"

static state setPoint;
static state processVariable;
static error err;

static float Kp_linear;
static float Kd_linear;

static float Kp_angular;
static float Kd_angular;


/**
    Initialize PID variables
    @param initState the robot initial position and angle
*/
void initPID(state initState){
	Kp_linear = KP_LINEAR;
	Kd_linear = KD_LINEAR;

	Kp_angular = KP_ANGULAR;
	Kd_angular = KD_ANGULAR;

	processVariable = initState;

	vector<float> position{0,0};
	setPoint = {
		position,
		0
	};

	err = error{0,0,0,0,0,0};

}

/**
    Scale final command to the predefined limits
    @param linearCommand the linear command before scaling
    @param angularCommand the angular command before scaling
    @return command vector contain linearCommand and angularCommand after scaling
*/
vector<int> scale(float linearCommand, float angularCommand){
	vector<int> command;
	if((linearCommand + angularCommand) > MAX_VELOCITY){
		command[0] = MAX_VELOCITY;
		command[1] = MAX_VELOCITY - angularCommand;
	}
	command[0] = roundf(linearCommand + angularCommand);
	command[1] = roundf(linearCommand - angularCommand);

	return command;
}

/**
    Get linear and angular command from the PID Algorithm
    @return command vector contain linearCommand and angularCommand
*/
vector<int> controller(){

	float angularCommand = PID_angular(err);
	float linearCommand = PID_linear(err);

	return scale(linearCommand, angularCommand);

};



/**
    linear PID Algorithm
    @return command the linear command
*/
float PID_linear(error err){

	float command = 0;

	command += (float) Kp_linear * err.currErrLinear;
	if(KI_LINEAR) command += (float) KI_LINEAR * err.sumErrorLinear;
	command += (float) Kd_linear * (err.currErrLinear - err.prevErrLinear);

	return command;

}

/**
    angular PID Algorithm
    @return command the angular command
*/
float PID_angular(error err){

	float command = 0;

	command += Kp_angular * err.currErrAngular;
	if(KI_ANGULAR) command += KI_ANGULAR * err.sumErrorAngular;
	command += Kd_angular * (err.currErrAngular - err.prevErrAngular);

	return command;
}

void set_kp_linear(float new_kp){
	Kp_linear = new_kp;
}

void set_kd_linear(float new_kd){
	Kd_linear = new_kd;
}


void set_kp_angular(float new_kp){
	Kp_angular = new_kp;
}

void set_kd_angular(float new_kd){
	Kd_angular = new_kd;
}


void set_SetPoint(vector<float> position, float angle){
	state desiredState{
		position,
		angle
	};
	setPoint = desiredState;
}

/**
    update process variable and error
    @params sensedState the current sensed state from robot's sensors
*/
void set_processVariable(state sensedState){

	err.currErrLinear = sqrt((setPoint.position[0] - sensedState.position[0]) * (setPoint.position[0] - sensedState.position[0])
			+ (setPoint.position[1] - sensedState.position[1]) * (setPoint.position[1] - sensedState.position[1]) );

	err.prevErrLinear = sqrt((setPoint.position[0] - processVariable.position[0]) * (setPoint.position[0] - processVariable.position[0])
				+ (setPoint.position[1] - processVariable.position[1]) * (setPoint.position[1] - processVariable.position[1]) );

	err.currErrAngular = absoluteAngle(setPoint.position) - sensedState.angle ;
	err.prevErrAngular = absoluteAngle(setPoint.position) - processVariable.angle;

	processVariable = sensedState;

	err.sumErrorLinear += err.currErrLinear;
	err.sumErrorAngular += err.prevErrLinear;
}

/**
    tests if the target state is reached
    @return boolean
*/
bool targerReached(){
	return (err.currErrLinear <= PRECISION_LINEAR) && (err.currErrAngular <= PRECISION_ANGULAR) ;
}


