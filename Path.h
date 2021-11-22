/*
 * Path.h
 */

#ifndef SRC_PATH_H_
#define SRC_PATH_H_

#include <queue>
#include <vector>
using std::vector;
using std::queue;
#include "Path.h"
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


queue<job> predefinedPath;

class Path {
private:
	queue<job> predefinedPath; // predefined set of Jobs
	state currentState; //current position and orientation
	Robot robot;
public:
	Path(Robot robot);
	virtual ~Path();
	int updateState();
	queue<job> getPredefinePath();
	void setPredefinedPath(queue<job> path);
	state getCurrentState();
};

#endif /* SRC_PATH_H_ */
