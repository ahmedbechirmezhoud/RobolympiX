/*
 * reglages.h
 */

#ifndef SRC_SETTINGS_H_
#define SRC_SETTINGS_H_

//PID params initialization

#define KI_LINEAR  0
#define KP_LINEAR  0.8
#define KD_LINEAR  .1

#define KI_ANGULAR 0
#define KP_ANGULAR 0.2
#define KD_ANGULAR .1

// Motor max Velocity
#define MAX_VELOCITY 1024
#define MIN_VELOCITY 380
 
#define WHEEL_PERIMETER 250  //ENCODER'S WHEEL

#define ENTRAXE 360

// Robot initialization
#define X_INIT 0
#define Y_INIT 0
#define ALPHA_INIT 0

#define PRECISION_LINEAR 7
#define PRECISION_ANGULAR 30

#endif /* SRC_SETTINGS_H_ */
