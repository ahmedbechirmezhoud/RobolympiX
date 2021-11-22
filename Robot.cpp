/*
 * Robot.cpp
 */

#include "Robot.h"

Robot::Robot(TIM_HandleTypeDef* motorsTimer, TIM_HandleTypeDef* encoderTimerL, TIM_HandleTypeDef* encoderTimerR) {
	// TODO Auto-generated constructor stub
	motorRF = 0;
	motorLF = 0;
	motorRB = 0;
	motorLB = 0;
	encoderR = 0;
	encoderL = 0;
	angleL = 0;
	angleR = 0;
	counterL = 0;
	counterR = 0;
	distanceR = 0;
	distanceL = 0;
	this->motorsTimer = motorsTimer;
	this->encoderTimerL = encoderTimerL;
	this->encoderTimerR = encoderTimerR;
}

Robot::~Robot() {
	// TODO Auto-generated destructor stub
}



void Robot::change_pulse(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t pulse)
{


  TIM_OC_InitTypeDef sConfigOC = {0};

  HAL_TIM_PWM_Stop(htim, Channel);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, Channel) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(htim);
  HAL_TIM_PWM_Start(htim, Channel);

}

void Robot::setMotorLF(int motorLF){
	change_pulse(motorsTimer, TIM_CHANNEL_1, motorLF);
}

void Robot::setMotorLB(int motorLB){
	change_pulse(motorsTimer, TIM_CHANNEL_2, motorLB);
}

void Robot::setMotorRF(int motorRF){
	change_pulse(motorsTimer, TIM_CHANNEL_3, motorRF);
}

void Robot::setMotorRB(int motorRB){
	change_pulse(motorsTimer, TIM_CHANNEL_4, motorRB);
}

void Robot::setMotorL(int motorL){
	if (motorL>=0){
		setMotorLF(motorL);
		setMotorLB(0);
	}
	else{
		setMotorLF(0);
		setMotorLB(-motorL);
	}
}

void Robot::setMotorR(int motorR){
	if (motorR>=0){
		setMotorRF(motorR);
		setMotorRB(0);
	}
	else{
		setMotorRF(0);
		setMotorRB(-motorR);
	}
}



int Robot::getEncoderL(){
	return encoderL;
}

int Robot::getEncoderR(){
	return encoderR;
}

void Robot::setEncoderR(int encoderR){
	this->encoderR = encoderR;
}

void Robot::setEncoderL(int encoderL){
	this->encoderL = encoderL;
}

float Robot::getAngleL(){
	return angleL;
}

float Robot::getAngleR(){
	return angleR;
}

void Robot::setAngleL(float angleL){
	this->angleL = angleL;
}

void Robot::setAngleR(float angleR){
	this->angleR = angleR;
}

void Robot::setCounterR(uint32_t counterR){
	this->counterR = counterR;
}

void Robot::setCounterL(uint32_t counterL){
	this->counterL = counterL;
}

uint32_t Robot::getCounterR(){
	return counterR;
}

uint32_t Robot::getCounterL(){
	return counterL;
}

void Robot::setDistanceR(float distanceR){
	this->distanceR = distanceR;
}

void Robot::setDistanceL(float distanceL){
	this->distanceL = distanceL;
}

float Robot::getDistanceL(){
	return distanceL;
}

float Robot::getDistanceR(){
	return distanceR;
}

TIM_HandleTypeDef* Robot::getEncoderLTimer(){
	return encoderTimerL;
}

TIM_HandleTypeDef* Robot::getEncoderRTimer(){
	return encoderTimerR;
}





