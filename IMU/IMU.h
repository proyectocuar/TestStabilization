/*
	IMU.cpp - Proyecto CUAR - 2014

    Clase para controlar el sensor IMU
*/

#ifndef IMU_h
#define IMU_h

#include "MPU6050_6Axis_MotionApps20.h"
#include "helper_3dmath.h"


class IMU
{
	public:
		void init();
		bool initOk();
		bool isReady();
		bool calculateYPR();
		double getYaw();
		double getPitch();
		double getRoll();
		
		static void dmpDataReady();

	private:  
		MPU6050 mpu;							// mpu interface object

		bool dmpReady;							// set true if DMP init was successful
		uint8_t mpuIntStatus;					// mpu statusbyte
		uint8_t devStatus;						// device status    
		uint16_t packetSize;					// estimated packet size  
		uint16_t fifoCount;						// fifo buffer size   
		uint8_t fifoBuffer[64];					// fifo buffer 

		Quaternion q;							// quaternion for mpu output
		VectorFloat gravity;					// gravity vector for ypr
		double ypr[3];							// yaw pitch roll values
};

#endif
