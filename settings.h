/*
 * reglages.h
 */

#ifndef SRC_SETTINGS_H_
#define SRC_SETTINGS_H_

//PID params initialization

#define KI_LINEAR  0
#define KP_LINEAR  1.75
#define KD_LINEAR  0

#define KI_ANGULAR 0
#define KP_ANGULAR 0.8
#define KD_ANGULAR 0

// Motor max Velocity
#define MAX_VELOCITY 1024
#define MIN_VELOCITY 300
#define ACC_RATE 0.005
 
#define WHEEL_PERIMETER 250  //ENCODER'S WHEEL

#define ENTRAXE 362

// Robot initialization
#define X_INIT 0
#define Y_INIT 0
#define ALPHA_INIT 0

#define PRECISION_LINEAR 30
#define PRECISION_ANGULAR 120

#endif /* SRC_SETTINGS_H_ */
