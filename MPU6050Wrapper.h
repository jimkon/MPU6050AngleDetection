

#ifndef _MPU6050WRAPPER_H_
#define _MPU6050WRAPPER_H_

#include "MPU6050.h"
#include "./libs/helper_3dmath.h"


const float ACCEL_DIVIDERS[] =	{16384.0, 8192.0,	4096.0,	2048.0};
const float GYRO_DIVIDERS[] = {131,	65.5,	32.8,	16.4};
const float COMPLIMENTARY_ACCEL = 0.02;
const float COMPLIMENTARY_GYRO = 0.98;

class MPU6050Wrapper {
	
	private:
		MPU6050 mpu;
		float accel_div;
		float gyro_div;
		VectorFloat angle;
		VectorInt16 accel_sensor, gyro_sensor;
	
	public:
		
		//
	
        MPU6050Wrapper();
		
		void init();
		//MPU6050 getMPUSensor();
		
		void setRangeSettings(int accel, int gyro);
		
		
		bool fullTest();
		
		//bool quickTest();
		
		void refresh(float dt);
		
		float getAngleX();
		
		float getAngleY();
		
		float getAngleZ();
		
		int getSampleRate();
		
		
	private:
		//settings configuration
		void setDefaultSettings();
		
		void setProperOffsets();
		
		void setFIFOSettings();
		//
		
		void parseSensorValues();
		
		int getFIFOEnabledSensors();
		
		
	public:
	//test function
		double test();
};
#endif