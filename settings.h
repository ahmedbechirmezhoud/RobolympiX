/*
 * reglages.h
 */

#ifndef SRC_SETTINGS_H_
#define SRC_SETTINGS_H_

//PID params initialization

#define KI_LINEAR  0
#define KP_LINEAR  1
#define KD_LINEAR  1

#define KI_ANGULAR 0
#define KP_ANGULAR 1
#define KD_ANGULAR 1

// Motor max Velocity
#define MAX_VELOCITY 255
#define MIN_VELOCITY 0
 
#define WHEEL_PERIMETER  250  //ENCODER'S WHEEL 

#define ENTRAXE 350

// Robot initialization
#define X_INIT 0
#define Y_INIT 0
#define ALPHA_INIT 0

#define PRECISION_LINEAR 0.1
#define PRECISION_ANGULAR 0.1

#endif /* SRC_SETTINGS_H_ */
