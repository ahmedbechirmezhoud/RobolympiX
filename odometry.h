/*
 * odometry.h
 */

#ifndef SRC_ODOMETRY_H_
#define SRC_ODOMETRY_H_
#include <vector>
#include "Path.h"

float relativeAngle(state currentState, vector<float> coordinates);

vector<float> relative2absolute(state currentState, vector<float> translation);

vector<float> absolute2relative(state currentState, vector<float> coordinates);

float absoluteAngle(vector<float> coordinates);

float degree2miliradian(float degree);

float radian2milliradian(float radian);

float milliradian2radian(float mrad);

state odometry(float distanceL, float distanceR);

#endif /* SRC_ODOMETRY_H_ */
