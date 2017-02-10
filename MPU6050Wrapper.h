

#ifndef _MPU6050WRAPPER_H_
#define _MPU6050WRAPPER_H_

#include "MPU6050.h"


const double ACCEL_DIVIDERS[] =	{8192.0,	4096.0,	2048.0,	1024.0};
const double GYRO_DIVIDERS[] = {131,	65.5,	32.8,	16.4};

class MPU6050Wrapper {
	
	private:
		MPU6050 mpu;
		
		//sersor values
		int ax, ay, az;
		int gx, gy, gz;
	
	
	public:
        MPU6050Wrapper();
		
		bool fullTest();
		
		void refresh();
		
		double getAngleX();
		
		double getAngleY();
		
		double getAngleZ();
		
	private:
		void configureSettings();
	
};
#endif