

#include "MPU6050Wrapper.h"

//setup
MPU6050Wrapper::MPU6050Wrapper() {
	configureSettings();
}

void MPU6050Wrapper::configureSettings() {
	mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setSleepEnabled(false);
}

bool MPU6050Wrapper::fullTest() {
	return mpu.testConnection();
}

//loop
void MPU6050Wrapper::refresh() {
	mpu.getMotion6(&sensor_accel.x, &sensor_accel.y, &sensor_accel.z, &sensor_gyro.x, &sensor_gyro.y, &sensor_gyro.z);
}

double MPU6050Wrapper::getAngleX() {
	return gyro.getX();
}
		
double MPU6050Wrapper::getAngleY() {
	return gyro.getY();
}
		
double MPU6050Wrapper::getAngleZ() {
	return gyro.getY();
}