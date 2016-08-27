/*
	IMU.cpp - Proyecto CUAR - 2014

    Clase para controlar el sensor IMU
*/

#include "IMU.h"

volatile bool mpuInterrupt;

/*
 *  init function
 *  initialize DMP and waiting for first interrupt (data)
 */
void IMU::init()
{
	ypr[0] = 0.0f;
	ypr[1] = 0.0f;
	ypr[2] = 0.0f;

	mpuInterrupt = false;

	dmpReady = false;

//	// join I2C bus (I2Cdev library doesn't do this automatically)
//	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
//		TWBR = 12; // 400kHz I2C clock (200kHz if CPU is 8MHz) ESTABA EN 24.
//	#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//		Fastwire::setup(400, true);	//set TWBR to 12.
//	#endif
	
	// initialize device
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();

	mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

	// verify connection
	Serial.println(F("Testing device connections..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	if(devStatus == 0)
	{
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}

void IMU::dmpDataReady()
{
	mpuInterrupt = true;
}

bool IMU::initOk()
{
	return dmpReady;
}

bool IMU::isReady()
{
	return !(!mpuInterrupt && fifoCount < packetSize);
}

/*
 *  getYPR function
 *  gets data from MPU and
 *  computes pitch, roll, yaw on the MPU's DMP
 */
bool IMU::calculateYPR()
{
	bool ok = false;

	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();
	fifoCount = mpu.getFIFOCount();

	if((mpuIntStatus & 0x10) || fifoCount >= 1024)
	{
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));

		ok = false;
	}
	else if(mpuIntStatus & 0x02)
	{
		while (fifoCount < packetSize)
		{
			fifoCount = mpu.getFIFOCount();
		}

		mpu.getFIFOBytes(fifoBuffer, packetSize);

		fifoCount -= packetSize;

		float yprAux[3] = {0.0f, 0.0f, 0.0f};

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(yprAux, &q, &gravity);

		ypr[0] = (double) yprAux[0];
		ypr[1] = (double) yprAux[1];
		ypr[2] = (double) yprAux[2];

		// Converson de angulos a grados
		ypr[0] = (ypr[0] * 180 / M_PI);
		ypr[1] = (ypr[1] * 180 / M_PI);
		ypr[2] = (ypr[2] * 180 / M_PI);

		// Redondeo a solo 2 decimales.
		//ypr[0] = ((int)(ypr[0] * 10 )) / 10.0;
		//ypr[1] = ((int)(ypr[1] * 10 )) / 10.0;
		//ypr[2] = ((int)(ypr[2] * 10 )) / 10.0;

		ok = true;
	}
	else
	{
		//Serial.println(F("IMU not data!"));
		ok = false;
	}

	return ok;
}

double IMU::getYaw()
{
	return ypr[0];
}

double IMU::getPitch()
{
	return ypr[1];
}

double IMU::getRoll()
{
	return ypr[2];
}
