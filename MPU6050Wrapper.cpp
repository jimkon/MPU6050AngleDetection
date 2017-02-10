

#include "MPU6050Wrapper.h"

//setup
MPU6050Wrapper::MPU6050Wrapper() {
	setDefaultSettings();
}

//MPU6050 MPU6050Wrapper::getMPUSensor(){
//	return mpu;
//}

void MPU6050Wrapper::setRangeSettings(int accel, int gyro){
	accel %= 4;
	gyro  %= 4;
	
	mpu.setFullScaleAccelRange(accel);
	mpu.setFullScaleGyroRange(gyro);
	
	accel_div = ACCEL_DIVIDERS[accel];
	gyro_div = GYRO_DIVIDERS[gyro];
	
}

void MPU6050Wrapper::setDefaultSettings() {
	mpu.setClockSource(mpu.MPU6050_CLOCK_PLL_XGYRO);
    setRangeSettings(mpu.MPU6050_ACCEL_FS_250, mpu.MPU6050_GYRO_FS_250);
    mpu.setSleepEnabled(false);
}

bool MPU6050Wrapper::fullTest() {
	return mpu.testConnection();
}

//loop
void MPU6050Wrapper::refresh(float dt) {
	VectorFloat temp[] = getSensorValues();
	
	VectorFloat accel_sensor = temp[0];
	VectorFloat gyro_sensor = temp[1];
	
	//implementing complimentary filter
	accel_sensor.mult(COMPLIMENTARY_ACCEL);
	gyro_sensor.mult(COMPLIMENTARY_GYRO);
	gyro_sensor.add(accel_sensor);
	
	gyro_sensor.mult(dt);
	
	angle.add(gyro_sensor);
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
VectorFloat* MPU6050Wrapper::getSensorValues(){
	//sersor values
	int ax, ay, az;
	int gx, gy, gz;
	
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	
	VectorFloat accel((float)ax, (float)ay, (float)az);
	accel.div(accel_div);
	
	VectorFloat gyro((float)gx, (float)gy, (float)gz);
	gyro.div(gyro_div);
	
	VectorFloat res[] = {accel, gyro};
	return res;
}