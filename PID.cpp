/*
 * PID.cpp
 */


#include "PID.h"
#include "settings.h"
#include <math.h>
#include "odometry.h"


static float Kp_linear;
static float Kd_linear;

static float Kp_angular;
static float Kd_angular;

state setPoint;
state processVariable;
error err;

vector<float> oldCommand = {0, 0};


/**
    Initialize PID variables
    @param initState the robot initial position and angle
*/
void initPID(state initState){
	Kp_linear = KP_LINEAR;
	Kd_linear = KD_LINEAR;

	Kp_angular = KP_ANGULAR;
	Kd_angular = KD_ANGULAR;
	set_processVariable(initState, 0);
	vector<float> position{0,0};
	setPoint = {
		position,
		0
	};
	oldCommand = {0, 0};

	err = error{0,0,0,0,0,0,0,1};

}

/**
 * Map any value from an interval to other
 */
int map(int val, int fromLow, int fromHigh, int toLow, int toHigh){
	return (val - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}


/**
    Scale final command to the predefined limits
    @param linearCommand the linear command before scaling
    @param angularCommand the angular command before scaling
    @return command vector contain linearCommand and angularCommand after scaling
*/
vector<int> scale(float linearCommand, float angularCommand){
	
	vector<int> command; 

	// calculate the motors commands from the PID controller
	command[0] = roundf(linearCommand - angularCommand);
	command[1] = roundf(linearCommand + angularCommand);

	// limit the values of the command to the maximun given speed value
	if ((absolute(linearCommand) + absolute(angularCommand)) > MAX_VELOCITY){
		float r = 1. * MAX_VELOCITY / (absolute(linearCommand) + absolute(angularCommand));
		command[0] *= r;
		command[1] *= r;
	}

	if (command[0] != 0)
		command[0] = sign(command[0]) * map(absolute(command[0]), 0, MAX_VELOCITY, MIN_VELOCITY, MAX_VELOCITY);

	if (command[1] != 0)
		command[1] = sign(command[1]) * map(absolute(command[1]), 0, MAX_VELOCITY, MIN_VELOCITY, MAX_VELOCITY);


	// limit the accelaration and decceleration of the robot
	command[0] = ACC_RATE * command[0] + (1 - ACC_RATE) * oldCommand[0];
	command[1] = ACC_RATE * command[1] + (1 - ACC_RATE) * oldCommand[1];

	// conserve the command 
	oldCommand[0] = command[0];
	oldCommand[1] = command[1];


	/*int q = roundf(MIN_VELOCITY - (linearCommand - angularCommand));
	if(q > 0) {
			command[0] += q;
			command[1] += q;
		}*/

	//float b = MAX_VELOCITY / (linearCommand + angularCommand);


	return command;
}


/**
    Get linear and angular command from the PID Algorithm
    @return command vector contain linearCommand and angularCommand
*/
vector<int> controller(){

	// Calculate the commands if the error is out of the thresholds
	float angularCommand = (angularReached()) ? 0 : PID_angular(err);
	float linearCommand = (linearReached()) ? 0 : PID_linear(err);

	if ((linearCommand == 0) && isnan(setPoint.angle))
		return {0, 0};

	return scale(linearCommand, angularCommand);
};



/**
    linear PID Algorithm
    @return command the linear command
*/
float PID_linear(error err){

	float command = Kp_linear * err.currErrLinear;

	command += KI_LINEAR * err.integErrorLinear;
	command += Kd_linear * (err.currErrLinear - err.prevErrLinear) * err.dt;

	return command;

}

/**
    angular PID Algorithm
    @return command the angular command
*/
float PID_angular(error err){

	float command = Kp_angular * err.currErrAngular;

	command += KI_ANGULAR * err.integErrorAngular;
	command += Kd_angular * (err.currErrAngular - err.prevErrAngular) * err.dt;

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

void set_SetPoint(vector<float> position, float angle=NAN){
	state desiredState{
		position,
		angle
	};
	setPoint = desiredState;
}

/**
    update process variable and error
    @param sensedState the current sensed state from robot's sensors
	@param dt the time difference between the current and previous update
*/
void set_processVariable(state sensedState, uint32_t dt){

	err.prevErrLinear = err.linearErrSign * sqrt((setPoint.position[0] - processVariable.position[0]) * (setPoint.position[0] - processVariable.position[0])
					+ (setPoint.position[1] - processVariable.position[1]) * (setPoint.position[1] - processVariable.position[1]));


	float desiredAngle = radian2milliradian(relativeAngle(sensedState, setPoint.position));

	float angle_diff = mapAngle(desiredAngle - sensedState.angle);

	err._real_angle_diff = radian2degree(milliradian2radian(angle_diff));


	if ((absolute(angle_diff) > degree2miliradian(140)) && (err.prevErrLinear < 30)){

		err.linearErrSign = -1;

		//desiredAngle = mapAngle(desiredAngle + degree2miliradian(180));
		angle_diff = mapAngle(angle_diff + degree2miliradian(180));
	}
	else
		err.linearErrSign = 1;
	

	err.currErrAngular = angle_diff;

	err.currErrLinear = err.linearErrSign * sqrt((setPoint.position[0] - sensedState.position[0]) * (setPoint.position[0] - sensedState.position[0])
			+ (setPoint.position[1] - sensedState.position[1]) * (setPoint.position[1] - sensedState.position[1]));



	if(linearReached())
		(isnan(setPoint.angle)) ? (err.currErrAngular = 0) : (err.currErrAngular = mapAngle(setPoint.angle - sensedState.angle));
		

	err.prevErrAngular = 0;//absoluteAngle(setPoint.position) - processVariable.angle;

	err.dt = dt;

	processVariable = sensedState;

	err.integErrorLinear += err.currErrLinear*dt;
	err.integErrorAngular += err.currErrAngular*dt;



	/*
	if (err.currErrLinear > ENTRAXE){
		err.currErrAngular = desiredAngle;
	}
	else{
		err.currErrAngular = setPoint.angle - sensedState.angle;
	}
	*/


}

/**
    tests if the target position is reached
    @return boolean
*/
bool linearReached(){
	return (absolute(err.currErrLinear) <= PRECISION_LINEAR);
}


/**
    tests if the target angle is reached
    @return boolean
*/
bool angularReached(){
	return (absolute(err.currErrAngular) <= PRECISION_ANGULAR) ;
}


