
#include "MPU6050Wrapper.h"

/*
	TODO:
	test setSampleRate
	test parseSensorValues
	test readyData
	implement setProperOffsets
	debug
	
*/ 

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
	return true;
}

//loop
bool MPU6050Wrapper::readyData(){
	return mpu.getFIFOCount()>0;
}

void MPU6050Wrapper::refresh(unsigned long dt) {
	parseSensorValues();
	
	VectorFloat accel(accel_sensor, accel_div);
	VectorFloat gyro(gyro_sensor, gyro_div);
	
	//implementing complimentary filter
	/*
	accel.mult(COMPLIMENTARY_ACCEL);
	gyro.mult(COMPLIMENTARY_GYRO);
	gyro.add(accel);
	
	gyro.mult(dt);
	
	angle.add(gyro);
	*/
	
	angle = gyro;
}

float MPU6050Wrapper::getAngleX() {
	if(x_enabled)
		return angle.x;
	return 0;
}
		
float MPU6050Wrapper::getAngleY() {
	if(y_enabled)
		return angle.y;
	return 0;
}
		
float MPU6050Wrapper::getAngleZ() {
	if(z_enabled)
		return angle.z;
	return 0;
}

void MPU6050Wrapper::setSampleRate(int rate){
	rate = min(8000, rate);
	if(rate>1000){
		mpu.setDLPFMode(0);
	}
	float gyro_sample_rate = 1000.0;
	if(mpu.getDLPFMode() == 0 || mpu.getDLPFMode() == 7){
		gyro_sample_rate *= 8;
	}
	int smpl_div = round((gyro_sample_rate/(float)rate)-1);
	mpu.setRate(smpl_div);
}
float MPU6050Wrapper::getSampleRate(){
	float gyro_sample_rate = 1000.0;
	if(mpu.getDLPFMode() == 0 || mpu.getDLPFMode() == 7){
		gyro_sample_rate *= 8;
	}
	float res = gyro_sample_rate/(1.0+(float)mpu.getRate());
	return res;
}

//temporary implementation. doesn't use FIFO
/*void MPU6050Wrapper::parseSensorValues(){
	accel_sensor.x = mpu.getAccelerationX();
	accel_sensor.y = mpu.getAccelerationY();
	accel_sensor.z = mpu.getAccelerationZ();
	
	gyro_sensor.x = mpu.getRotationX();
	gyro_sensor.y = mpu.getRotationY();
	gyro_sensor.z = mpu.getRotationZ();
}*/
int tv = 0;
void MPU6050Wrapper::parseSensorValues(){
	tv = 0;
	int size = mpu.getFIFOCount();
	//checking if FIFO is empty
	if(size == 0){
		return;
	}
	
	float samples = (float)size/(float)getFIFOSampleSize();// make this int
	//checking if something is wrong with the sample size
	if((int)samples-samples>0){
		//WARNING !!!
		//samples can't be used.
		return;
	}
	//read FIFO data in packets of 255 or less
	uint8_t data[size];
	int index = 0;
	do{
		uint8_t packet = (uint8_t)min(size, 255);
		mpu.getFIFOBytes(&data[index], packet);
		index += packet;
	}while(index < size);
	
	//calculate position of each measurement in the sample
	int ax_offset = 0;
	int ay_offset = 2;
	int az_offset = 4;
	int gx_offset = x_enabled?6:4;
	int gy_offset = gx_offset+(y_enabled?2:0);
	int gz_offset = gy_offset+(z_enabled?2:0);
	
	//averaging values of each measurement
	long ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
	for(int i=0; i<size; i+=getFIFOSampleSize()){
		tv++;
		ax += (((long)data[i+ax_offset]) << 8) | data[i+ax_offset+1];
		ay += (((long)data[i+ay_offset]) << 8) | data[i+ay_offset+1];
		az += (((long)data[i+az_offset]) << 8) | data[i+az_offset+1];
		if(x_enabled)
			gx += (((long)data[i+gx_offset]) << 8) | data[i+gx_offset+1];
		if(y_enabled)
			gy += (((long)data[i+gy_offset]) << 8) | data[i+gy_offset+1];
		if(z_enabled)
			gz += (((long)data[i+gz_offset]) << 8) | data[i+gz_offset+1];
	}
	
	accel_sensor.x = (uint16_t)ax / samples;
	accel_sensor.y = (uint16_t)ay / samples;
	accel_sensor.z = (uint16_t)az / samples;
	
	gyro_sensor.x = (uint16_t)gx / samples;
	gyro_sensor.y = (uint16_t)gy / samples;
	gyro_sensor.z = (uint16_t)gz / samples;
	
}

//private 
void MPU6050Wrapper::setDefaultSettings() {
	mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setRangeSettings(0, 0);
	mpu.setSleepEnabled(false);
	
	//mpu.setDLPFMode(1);
	//mpu.setRate(99);
	setSampleRate(60);
	setFIFOSettings();
	
	//setProperOffsets();
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
	sum *= 2; 		//2 bytes of data for each sample
	
	return sum;
}

int MPU6050Wrapper::getMaxDT(){
	float period = 1000.0/getSampleRate();
	float max_sample_number = 1024.0/getFIFOSampleSize();
	return (int)(period*max_sample_number);
}

//test
double MPU6050Wrapper::test(){
	return tv;
}