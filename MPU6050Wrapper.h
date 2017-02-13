

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
		const static bool X_ENABLED = true;
		const static bool Y_ENABLED = true;
		const static bool Z_ENABLED = true;
		
		bool x_enabled = true, y_enabled = true, z_enabled = true;
		//
		
		MPU6050Wrapper();
	
        MPU6050Wrapper(bool x, bool y, bool z);
		
		void init();
		//MPU6050 getMPUSensor();
		
		void setRangeSettings(int accel, int gyro);
		
		bool fullTest();
		
		bool quickTest();
		
		void refresh(float dt);
		
		float getAngleX();
		
		float getAngleY();
		
		float getAngleZ();
		
		int getMaxDT();
		
	private:
		//settings configuration
		void setDefaultSettings();
		
		void setProperOffsets();
		
		void setFIFOSettings();
		//
		void parseSensorValues();
		
		int getFIFOEnabledSensors();
		
		int getSampleRate();
		
		int getFIFOSampleSize();
		
	public:
	//test function
		double test();
};
#endif