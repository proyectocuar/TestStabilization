/*
	TestStabilization.ino - Proyecto CUAR - 2014

    Pruebas de algoritmo de estabilizacion con un prototipo reducido de solo dos motores.
*/

#include <IMU.h>
#include "Motors.h"
#include <PID.h>

#define MONITOR_SERIAL 115200

//--------------------------------
//// DEBUG TO ON/OFF PRINT
//--------------------------------
#define PRINT_MICROS
#define PRINT_PID_OUTPUT
#define PRINT_ANGLES
#define PRINT_MOTORS
#define PRINT_PID
//#define PRINT_INFLUENCE
#define PRINT_VELOCITY
#define PRINT_SETPOINT

unsigned long loopTimeIni = 0;
unsigned long loopTimeEnd = 0;

//--------------------------------
//// PID CONFIGURATION
//--------------------------------
#define INPUT_PITCH_P 0
#define INPUT_PITCH_I 1
#define INPUT_PITCH_D 2
#define INPUT_DISABLED 3
#define INPUT_PITCH_INFLUENCE 4

double PITCH_P_VAL = 0.13;
double PITCH_I_VAL = 0.09;
double PITCH_D_VAL = 0.06;

int currentInputPIDValue = INPUT_DISABLED;

//--------------------------------
//// IMU CONTROL VARIABLES
//--------------------------------
IMU imu;

double pitch = 0.0f;

double pitchLast = 0.0f;

//--------------------------------
//// MOTOR CONTROL VARIABLE
//--------------------------------
Motors motors;

int velocity = 1100;

int speedMotor1 = 0;
int speedMotor3 = 0;

int velocityMotorsPitch = 0;

bool motorsOn = false;
bool motorsOnOld = false;

//--------------------------------
////  PID VARIABLES
//--------------------------------
double PID_PITCH_INFLUENCE = 20;

double setPointPitch = 0.0f;

double pitchOutput = 0.0f;

//				INPUT		OUTPUT			SETPOINT			KP				KI				KD			CONTROLLER DIRECTION
PID pitchReg(	&pitch,	&pitchOutput,	&setPointPitch,		PITCH_P_VAL,	PITCH_I_VAL,	PITCH_D_VAL,	REVERSE);

//--------------------------------
////  SETUP FUNCTION
//--------------------------------
 void setup()
{
	Serial.begin(MONITOR_SERIAL);

	imu.init();

	motors.init();

	initPID();
}

//--------------------------------
////  LOOP FUNCTION
//--------------------------------
void loop()
{
  	loopTimeIni = micros();

	// DMP is ready when IMU init finally ok
	if (!imu.initOk())
	{
		Serial.println("IMU DMP not ready");
		return;
	}

	// Wait for IMU ready
	while(!imu.isReady());

	if (imu.calculateYPR())
	{
		pitch = imu.getPitch();

		if (motorsOn == true && motorsOnOld == false)
		{
			motorsOnOld = true;
			motors.setMotorsOn();
		}

		if (motorsOn == false && motorsOnOld == true)
		{
			motorsOnOld = false;
			motors.setMotorsOff();
			restartVariables();
		}

		if (motorsOn == true && motorsOnOld == true)
		{
			computePID();

			calculateVelocities();
			updateMotors();
		}
	}

	readSerial();
	debugValues();
}

void computePID()
{
	if(abs(pitch - pitchLast) > 30)
	{
		pitch = pitchLast;
	}

	pitchLast = pitch;

	pitchReg.Compute();
}

/*
 *  calculateVelocities function
 *  calculates the velocities of every motor
 *  using the PID output
 */
void calculateVelocities()
{
	velocityMotorsPitch = velocity;
	speedMotor1 = ((100 - pitchOutput) / 100) * velocityMotorsPitch;
	speedMotor3 = (abs((-100 - pitchOutput) / 100)) * velocityMotorsPitch;
}

void updateMotors()
{
	motors.setMotorSpeed(MOTOR_1, speedMotor1);
	motors.setMotorSpeed(MOTOR_3, speedMotor3);
}

void initPID()
{
	pitchReg.SetMode(AUTOMATIC);
	pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
}

void updatePIDConstants()
{
	pitchReg.SetTunings(PITCH_P_VAL, PITCH_I_VAL, PITCH_D_VAL);
}

void updatePIDInfluence()
{
	pitchReg.SetOutputLimits(-PID_PITCH_INFLUENCE, PID_PITCH_INFLUENCE);
}

//--------------------------------
//// READ SERIAL INPUT
//--------------------------------
void readSerial()
{
	int instruc = 'A';
	if(Serial.available())
	{
		if(currentInputPIDValue == INPUT_DISABLED)
		{
			instruc = Serial.read();
			switch(instruc)
			{
				case 'w':
					velocity += 10;
				break;
				case 'x':
					velocity -= 10;
				break;
				case 'e':
					setPointPitch+= 1;
				break;
				case 'c':
					setPointPitch -= 1;
				break;
				case 's':
					motorsOn = true;
				break;
				case 'q':
					motorsOn = false;
				break;
				case 'p':
					currentInputPIDValue = INPUT_PITCH_P;
				break;
				case 'i':
					currentInputPIDValue = INPUT_PITCH_I;
				break;
				case 'd':
					currentInputPIDValue = INPUT_PITCH_D;
				break;
			}
		}
		else
		{
			float pidValue = Serial.parseFloat();
			if(currentInputPIDValue == INPUT_PITCH_INFLUENCE)
			{
				PID_PITCH_INFLUENCE = pidValue;
				updatePIDInfluence();
			}
			else
			{
				if(currentInputPIDValue == INPUT_PITCH_P)
				{
					if (pidValue < 1)
					{
						PITCH_P_VAL = pidValue;
						Serial.print("Pitch P");
						Serial.println(pidValue);
					}
				} 
				else if(currentInputPIDValue == INPUT_PITCH_I)
				{
					PITCH_I_VAL = pidValue;
				}
				else if(currentInputPIDValue == INPUT_PITCH_D)
				{
					PITCH_D_VAL = pidValue;
				}
				updatePIDConstants();
			}
			currentInputPIDValue = INPUT_DISABLED;
		}
	}
}

//--------------------------------
//// MONITOR SERIAL DEBUG
//--------------------------------
void debugValues()
{
	#ifdef PRINT_MICROS
		Serial.print("Micros ");
		Serial.print(micros());
		Serial.print(" ");
	#endif

	#ifdef PRINT_PID_OUTPUT
		Serial.print("P_Out ");
		Serial.print(pitchOutput);
		Serial.print("\t");
	#endif

	#ifdef PRINT_ANGLES
		Serial.print("Pitch ");
		Serial.print(pitch);
		Serial.print("\t");
	#endif

	#ifdef PRINT_MOTORS
		Serial.print("M1 ");
		Serial.print(motors.getMotorSpeed(MOTOR_1));
		Serial.print(" M3 ");
		Serial.print(motors.getMotorSpeed(MOTOR_3));
		Serial.print(" ");
	#endif

	#ifdef PRINT_PID
		Serial.print("Kp ");
		Serial.print(pitchReg.GetKp(), 3);
		Serial.print(" Ki ");
		Serial.print(pitchReg.GetKi(), 3);
		Serial.print(" Kd ");
		Serial.print(pitchReg.GetKd(), 3);
		Serial.print(" ");
	#endif

	#ifdef PRINT_INFLUENCE
		Serial.print("Influe ");
		Serial.print(PID_PITCH_INFLUENCE);
		Serial.print(" ");
	#endif

	#ifdef PRINT_VELOCITY
		Serial.print("Veloc ");
		Serial.print(velocity);
		Serial.print(" ");
	#endif

	#ifdef PRINT_SETPOINT
		Serial.print("SetPonit ");
		Serial.print(setPointPitch);
		Serial.print(" ");
	#endif

	loopTimeEnd = micros();

	Serial.print(" Loop ");
	Serial.print((loopTimeEnd - loopTimeIni) / 1000);
	Serial.println("");
}

void restartVariables()
{
	pitch = 0.0f;

	pitchLast = 0.0f;

	setPointPitch = 0.0f;

	pitchReg.Restart();
}
