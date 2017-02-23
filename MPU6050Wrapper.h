

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
		bool x_enabled = true, y_enabled = true, z_enabled = true;
	
	public:
		//------------------configuration------------------//
		
		//constructor with no parameters  
		MPU6050Wrapper();
	
		//constructor with parameters specifying enables rotation axes
        MPU6050Wrapper(bool x, bool y, bool z);
		
		//configuration of sensitivities
		void setRangeSettings(int accel, int gyro);
		
		//setter and getter for sampling rate. 
		//max rate = 8kHz, min rate = 3.9 Hz
		/*
			setSampleRate function will change the SMPL_DIV value to  
			reach the closest value to rate parameter. If rate is greater 
			than 1000, then DLPF mode will be set to zero. Else, DLPF mode
			will not change.
		*/
		void setSampleRate(int rate);
		float getSampleRate();
		//------------------setup------------------//
		//initialize the mpu6050 
		void init();
		
		/*
			make a full check for:
				.connections
				.FIFO enables (getFIFOEnabledSensors>5 || getFIFOEnabledSensors<2)
				.proper offsets
				.self tests
		*/
		bool fullTest();
		
		//get the maximum refresh time. This is the time that FIFO
		//needs to fill. It is highly recommended time difference
		//between refresh() calls not to exceed that time, in order
		//not to lose samples
		int getMaxDT();
		
		//------------------loop------------------//
		//performs a quick test to ensure connection is ok
		bool quickTest();
		
		//return true if there are data ready to parsed
		bool readyData();
		
		//take new measurements from FIFO and calculate the current angle
		//dt must be the time difference in ms from the previous call
		void refresh(unsigned long dt);
				
		//get the current estimation of these values. 
		//disables axes will return 0.0 or wrong values
		float getAngleX();
		float getAngleY();
		float getAngleZ();
		
		void setProperOffsets(int millis);
		
		void resetOffsets();
		
	private:
		//settings configuration
		void setDefaultSettings();
		
		
		
		void setFIFOSettings();
		//
		
		void parseSensorValues();
		
		int getFIFOEnabledSensors();
		
		int getFIFOSampleSize();
		
	public:
	//test function
		double test();
};
#endif