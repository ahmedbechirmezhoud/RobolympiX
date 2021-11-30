/*
 * Path.h
 */

#ifndef SRC_PATH_H_
#define SRC_PATH_H_

#include <queue>
#include <vector>
using std::vector;
using std::queue;
#include "PID.h"
#include "Robot.h"

enum jobType {
	alpha,  // orientate the robot to an angle alpha
	xyRelative, // go to the position relative to the robot reference
 	xyAbsolute, // go to the position relative to the map and robot's initial postion
	stop // stop the robot in a given state
};

typedef void(* func)();

typedef struct job{
	vector<float> absoluteCoordinates, translation;
	float angle;
	int delay;
	func task;
	jobType type;
} job;



class Path {
private:
	queue<job> predefinedPath; // predefined queue of Jobs
	state currentState; //current position and orientation
	Robot robot;
	uint32_t time_var;
	uint32_t dt;

public:
	Path(Robot robot);
	virtual ~Path();
	int updateState();
	void updateTime();
	queue<job> getPredefinePath();
	void setPredefinedPath(queue<job> path);
	state getCurrentState();
};

#endif /* SRC_PATH_H_ */
