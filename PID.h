/*
 * PID.h
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

#include <vector>
using std::vector;

struct state{
	vector<float> position; // absolute position
	float angle;  // Y relative % Y absolute
};

struct error{
	float currErrLinear;
	float prevErrLinear;
	float currErrAngular;
	float prevErrAngular;
	float sumErrorLinear, sumErrorAngular;
};



#define sign(x) ((x) < 0 ? - (1) : (1))

vector<int> controller();

void initPID(state initState);

float PID_linear(error err);
float PID_angular(error err);

void set_kp_linear(float new_kp);
void set_kd_linear(float new_kd);
void set_kp_angular(float new_kp);
void set_kd_angular(float new_kd);

void updateError();

void set_SetPoint(vector<float> position, float angle);
void set_processVariable(state sensedState);

bool targerReached();


#endif /* SRC_PID_H_ */
