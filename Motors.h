/*
	Motors.h - Proyecto CUAR - 2014

	Clase para controlar los motores
*/

#ifndef Motors_h
#define Motors_h

#include <Arduino.h>
#include "Servo.h"

// PIN
#define MOTOR_1_PIN	9
#define MOTOR_3_PIN	11

#define MOTOR_COUNT	2

#define MOTOR_1	0
#define MOTOR_3	1

#define DELTA_MOTOR_SPEED		10
#define STOP_MOTOR_SPEED		0
#define START_PWM				800
#define MIN_MOTOR_SPEED			1100
#define MAX_MOTOR_SPEED			2000

#define DELAY_BEFORE_START		1000
#define DELAY_INCREASE			50

class Motors
{
	public:
		void init();
		void setMotorsOn();
		void setMotorsOff();
		int getMotorSpeed(int motor);
		void setMotorSpeed(int motor, int motorSpeed);
		void increaseMotorSpeed(int motor);
		void decreaseMotorSpeed(int motor);

	private:
		Servo motors[MOTOR_COUNT];
		int motorsPins[MOTOR_COUNT];

		void setAllSpeed(int motorSpeed);
		void sendPWD(int motor, int motorSpeed);
		void barridoIncreaseMotorSpeed(int motor);
};

#endif

