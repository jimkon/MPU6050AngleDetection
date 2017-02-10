

#include "MPU6050Wrapper.h"

//setup
MPU6050Wrapper::MPU6050Wrapper() {
	setDefaultSettings();
}

MPU6050 MPU6050Wrapper::getMPUSensor(){
	return mpu;
}

void MPU6050Wrapper::setDefaultSettings() {
	mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    mpu.setSleepEnabled(false);
	accel_div = ACCEL_DIVIDERS[0];
	gyro_div = GYRO_DIVIDERS[0];
}

bool MPU6050Wrapper::fullTest() {
	return mpu.testConnection();
}

//loop
void MPU6050Wrapper::refresh() {
	VectorFloat temp[] = getSensorValues();
	
	VectorFloat accel_sensor = temp[0];
	VectorFloat gyro_sensor = temp[1];
}

double MPU6050Wrapper::getAngleX() {
	return gx;
}
		
double MPU6050Wrapper::getAngleY() {
	return gy;
}
		
double MPU6050Wrapper::getAngleZ() {
	return gz;
}

//temporary implementation
VectorFloat* MPU6050Wrapper::getSensorValues(){
	//sersor values
	int ax, ay, az;
	int gx, gy, gz;
	
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	
	
	
	VectorFloat accel((float)ax/accel_div, (float)ay/accel_div, (float)az/accel_div);
	VectorFloat gyro((float)gx/gyro_div, (float)gy/gyro_div, (float)gz/gyro_div);
	
	VectorFloat res[] = {accel, gyro};
	return res;
}