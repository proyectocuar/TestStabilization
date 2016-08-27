/*
	Motors.cpp - Proyecto CUAR - 2014

    Clase para controlar los motores
*/

#include "Motors.h"

// Realiza la conexion con los motores
void Motors::init()
{
	motorsPins[MOTOR_1] = MOTOR_1_PIN;
	motorsPins[MOTOR_3] = MOTOR_3_PIN;

	// Attach
	for (int motor = 0; motor < MOTOR_COUNT; motor++)
	{
		motors[motor].attach(motorsPins[motor]);
	}

	setMotorsOff();
}

// Detiene los motores
void Motors::setMotorsOff()
{
	for (int motor = 0; motor < MOTOR_COUNT; motor++)
	{
		motors[motor].writeMicroseconds(STOP_MOTOR_SPEED);
	}
}

// Inicia los mootres
void Motors::setMotorsOn()
{
	setAllSpeed(START_PWM);

	delay(DELAY_BEFORE_START);
	
	int speedMotors = 0;

	while (speedMotors < MIN_MOTOR_SPEED)
	{
		for (int j = 0; j < MOTOR_COUNT; j++)
		{
			barridoIncreaseMotorSpeed(j);
		}
		
		speedMotors = getMotorSpeed(MOTOR_COUNT - 1);

		delay(DELAY_INCREASE);
	}
}

void Motors::setAllSpeed(int motorSpeed)
{
	for (int motor = 0; motor < MOTOR_COUNT; motor++)
	{
		sendPWD(motor, motorSpeed);
	}
}

void Motors::setMotorSpeed(int motor, int motorSpeed)
{
	if (motorSpeed < MIN_MOTOR_SPEED)
	{
		motorSpeed = MIN_MOTOR_SPEED;
	}
	if (motorSpeed > MAX_MOTOR_SPEED)
	{
		motorSpeed = MAX_MOTOR_SPEED;
	}

	sendPWD(motor, motorSpeed);
}

void Motors::sendPWD(int motor, int motorSpeed)
{
	motors[motor].writeMicroseconds(motorSpeed);
}

int Motors::getMotorSpeed(int motor)
{
	return motors[motor].readMicroseconds();
}

void Motors::barridoIncreaseMotorSpeed(int motor)
{
	sendPWD(motor, getMotorSpeed(motor) + DELTA_MOTOR_SPEED);
}

void Motors::increaseMotorSpeed(int motor)
{
	setMotorSpeed(motor, getMotorSpeed(motor) + DELTA_MOTOR_SPEED);
}

void Motors::decreaseMotorSpeed(int motor)
{
	setMotorSpeed(motor, getMotorSpeed(motor) - DELTA_MOTOR_SPEED);
}
