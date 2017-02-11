
#include "MPU6050Wrapper.h"

//setup
MPU6050Wrapper::MPU6050Wrapper() {
	
}

//MPU6050 MPU6050Wrapper::getMPUSensor(){
//	return mpu;
//}

void MPU6050Wrapper::init(){
	setDefaultSettings();
	setProperOffsets();
}

void MPU6050Wrapper::setRangeSettings(int accel, int gyro){
	accel %= 4;
	gyro  %= 4;
	
	mpu.setFullScaleAccelRange(accel);
	mpu.setFullScaleGyroRange(gyro);
	
	accel_div = ACCEL_DIVIDERS[accel];
	gyro_div = GYRO_DIVIDERS[gyro];
	
}

void MPU6050Wrapper::setDefaultSettings() {
	mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setRangeSettings(0, 0);
    mpu.setSleepEnabled(false);
}

void MPU6050Wrapper::setProperOffsets(){
	int16_t avg_ax = mpu.getAccelerationX();
	int16_t avg_ay = mpu.getAccelerationY();
	int16_t avg_az = mpu.getAccelerationZ();
	
	int16_t avg_gx = mpu.getRotationX();
	int16_t avg_gy = mpu.getRotationY();
	int16_t avg_gz = mpu.getRotationZ();
	/*
	int count = 0;
	while( one second not passed (count = number of samples per second)){
		average measurements for each sensor and its axis
	}
	*/
	
	
	mpu.setXAccelOffset(avg_ax);
    mpu.setYAccelOffset(avg_ay);
    mpu.setZAccelOffset(avg_az);
    mpu.setXGyroOffset (avg_gx);
    mpu.setYGyroOffset (avg_gy);
    mpu.setZGyroOffset (avg_gz);
}

bool MPU6050Wrapper::fullTest() {
	return mpu.testConnection();
}

//loop
void MPU6050Wrapper::refresh(float dt) {
	parseSensorValues();
	
	VectorFloat accel(accel_sensor, accel_div);
	VectorFloat gyro(gyro_sensor, gyro_div);
	
	//implementing complimentary filter
	/*accel.mult(COMPLIMENTARY_ACCEL);
	gyro.mult(COMPLIMENTARY_GYRO);
	gyro.add(accel);
	
	gyro.mult(dt);
	
	angle.add(gyro);*/
	
	angle = gyro;
}

float MPU6050Wrapper::getAngleX() {
	return angle.x;
}
		
float MPU6050Wrapper::getAngleY() {
	return angle.y;
}
		
float MPU6050Wrapper::getAngleZ() {
	return angle.z;
}

//temporary implementation. doesn't use FIFO
void MPU6050Wrapper::parseSensorValues(){
	accel_sensor.x = mpu.getAccelerationX();
	accel_sensor.y = mpu.getAccelerationY();
	accel_sensor.z = mpu.getAccelerationZ();
	
	gyro_sensor.x = mpu.getRotationX();
	gyro_sensor.y = mpu.getRotationY();
	gyro_sensor.z = mpu.getRotationZ();
}