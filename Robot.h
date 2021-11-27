/*
 * Robot.h
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "main.h"

class Robot{
private:
	int motorRF;
	int motorLF;
	int motorRB;
	int motorLB;
	int encoderR;
	int encoderL;
	float angleR;
	float angleL;
	uint32_t counterL;
	uint32_t counterR;
	float distanceR;
	float distanceL;
	TIM_HandleTypeDef* motorsTimer;
	TIM_HandleTypeDef* encoderTimerL;
	TIM_HandleTypeDef* encoderTimerR;

	void init();
	void setMotorRF(int motorRF);
	void setMotorLF(int motorLF);
	void setMotorRB(int motorRB);
	void setMotorLB(int motorLB);
	void change_pulse(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t pulse);

public:
	Robot();
	Robot(TIM_HandleTypeDef* motorsTimer, TIM_HandleTypeDef* encoderTimerL, TIM_HandleTypeDef* encoderTimerR);
	virtual ~Robot();
	void setMotorR(int motorR);
	void setMotorL(int motorL);
	void setEncoderR(int encoderR);
	void setEncoderL(int encoderL);
	void setAngleL(float angleL);
	void setAngleR(float angleR);
	void setCounterL(uint32_t counterL);
	void setCounterR(uint32_t counterR);
	void setDistanceR(float distanceR);
	void setDistanceL(float distanceL);
	int getEncoderR();
	int getEncoderL();
	float getAngleR();
	float getAngleL();
	uint32_t getCounterL();
	uint32_t getCounterR();
	float getDistanceR();
	float getDistanceL();
	TIM_HandleTypeDef* getEncoderLTimer();
	TIM_HandleTypeDef* getEncoderRTimer();

};

#endif /* SRC_ROBOT_H_ */
