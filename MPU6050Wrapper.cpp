
#include "MPU6050Wrapper.h"


//setup
MPU6050Wrapper::MPU6050Wrapper(){
	
}

MPU6050Wrapper::MPU6050Wrapper(bool x, bool y, bool z) {
	x_enabled = x;
	y_enabled = y;
	z_enabled = z;
}

void MPU6050Wrapper::init(){
	setDefaultSettings();
}

bool MPU6050Wrapper::fullTest() {
	return mpu.testConnection();
}

bool MPU6050Wrapper::quickTest(){
	/*
	here we can make a quick check for:
		.connections
		.fifo enables (getFIFOEnabledSensors>5 || getFIFOEnabledSensors<2)
	*/
	return true;
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

int MPU6050Wrapper::getSampleRate(){
	float gyro_sample_rate = 1000.0;
	if(mpu.getDLPFMode() == 0 || mpu.getDLPFMode() == 7){
		gyro_sample_rate *= 8;
	}
	float res = gyro_sample_rate/(1.0+(float)mpu.getRate());
	return (int)res;
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

//private 
void MPU6050Wrapper::setDefaultSettings() {
	mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setRangeSettings(0, 0);
	mpu.setSleepEnabled(false);
	
	mpu.setDLPFMode(1);
	mpu.setRate(99);
	setFIFOSettings();
	
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

void MPU6050Wrapper::setFIFOSettings(){
	mpu.setFIFOEnabled(true);
	
	mpu.setXGyroFIFOEnabled(x_enabled);
	mpu.setYGyroFIFOEnabled(y_enabled);
	mpu.setZGyroFIFOEnabled(z_enabled);
	mpu.setAccelFIFOEnabled(true);
	mpu.setTempFIFOEnabled(false);
	mpu.setSlave0FIFOEnabled(false);
	mpu.setSlave1FIFOEnabled(false);
	mpu.setSlave2FIFOEnabled(false);
	mpu.setSlave3FIFOEnabled(false);
	
	mpu.resetFIFO();
}

int MPU6050Wrapper::getFIFOEnabledSensors(){
	bool ens[9] = {	mpu.getTempFIFOEnabled(),
					mpu.getXGyroFIFOEnabled(),
					mpu.getYGyroFIFOEnabled(),
					mpu.getZGyroFIFOEnabled(),
					mpu.getAccelFIFOEnabled(),
					mpu.getSlave3FIFOEnabled(),
					mpu.getSlave2FIFOEnabled(),
					mpu.getSlave1FIFOEnabled(),
					mpu.getSlave0FIFOEnabled()};
					
	int sum = 0;
	for(int i=0; i<9; i++){
		if(ens[i]){
			sum ++;
		}
	}
	return mpu.getFIFOEnabled()?sum:0;
}

int MPU6050Wrapper::getFIFOSampleSize(){
	int sum = 0;
	if(	mpu.getXGyroFIFOEnabled() ){
		sum ++;
	}
	if( mpu.getYGyroFIFOEnabled() ){
		sum ++;
	}
	if( mpu.getZGyroFIFOEnabled() ){
		sum ++;
	}
	sum += 3;		//for accel XYZ
	sum *= 2; 	//2 bytes of data for each sample
	
	return sum;
}

int MPU6050Wrapper::getMaxDT(){
	float period = 1000.0/getSampleRate();
	float max_sample_number = 1024.0/getFIFOSampleSize();
	return (int)(period*max_sample_number);
}

//test
double MPU6050Wrapper::test(){
	return getMaxDT();
}