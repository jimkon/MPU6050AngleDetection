
#include "MPU6050Wrapper.h"

/*
	TODO:
	implement setProperOffsets
	debug
	
*/ 
int16_t tv = 0;
//setup
MPU6050Wrapper::MPU6050Wrapper(){
	
}

MPU6050Wrapper::MPU6050Wrapper(bool x, bool y, bool z) {
	x_enabled = x;
	y_enabled = y;
	z_enabled = z;
}

void MPU6050Wrapper::init(int sample_rate, int millis_calibration){
	mpu.setClockSource(MPU6050_CLOCK_PLL_XGYRO);
    setRangeSettings(0, 0);
	mpu.setSleepEnabled(false);
	
	//mpu.setDLPFMode(1);
	setSampleRate(sample_rate);
	setFIFOSettings();
	
	setProperOffsets(millis_calibration);
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
	angle.reset();
	
	//VectorFloat accel(accel_sensor, accel_div);
	//VectorFloat gyro(gyro_sensor, gyro_div);
	
	//implementing complimentary filter
	/*
	accel.mult(COMPLIMENTARY_ACCEL);
	gyro.mult(COMPLIMENTARY_GYRO);
	gyro.add(accel);
	
	gyro.mult(dt);
	
	angle.add(gyro);
	*/
	
	angle.sum(gyro_sensor, 10000, gyro_div);
}

long MPU6050Wrapper::getAngleX() {
	if(x_enabled)
		return angle.x;
	return 0;
}
		
long MPU6050Wrapper::getAngleY() {
	if(y_enabled)
		return angle.y;
	return 0;
}
		
long MPU6050Wrapper::getAngleZ() {
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
	smpl_div = min(smpl_div, 255);
	smpl_div = max(smpl_div, 0);
	mpu.setRate(smpl_div);
	tv = 1000/getSampleRate();
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

void MPU6050Wrapper::parseSensorValues(){
	int size = mpu.getFIFOCount();
	//checking if FIFO is empty
	if(size == 0){
		return;
	}
	
	int  samples = size/getFIFOSampleSize();// make this int
	//checking if something is wrong with the sample size
	if(size%getFIFOSampleSize()>0){
		//WARNING !!!
		//samples can't be used.
		tv = -1;
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
	
	
	gyro_sensor.x = (int16_t)gx / samples;
	gyro_sensor.y = (int16_t)gy / samples;
	gyro_sensor.z = (int16_t)gz / samples;
}

//private 

void MPU6050Wrapper::setRangeSettings(int accel, int gyro){
	accel %= 4;
	gyro  %= 4;
	
	mpu.setFullScaleAccelRange(accel);
	mpu.setFullScaleGyroRange(gyro);
	
	accel_div = ACCEL_DIVIDERS[accel];
	gyro_div = GYRO_DIVIDERS[gyro];
	
}

//averaging measurements for millis and set this value as offset
void MPU6050Wrapper::setProperOffsets(int millis){
	if(millis<=0){
		return;
	}
	//reset fifo?????
	//save the previous sample rate
	int previous_sample_rate = getSampleRate();
	//find the optimal sampling rate via search
	int optimal_sampling_rate = 1000; //unknown
	//setSampleRate(optimal_sampling_rate); // set this rate
	
	//find the number of samples in millis
	int samples = optimal_sampling_rate * ((float)millis/1000.0); //getSampleRate instead of optimal_sampling_rate
	int full_samples = samples/getFIFOSampleSize();
	samples = full_samples * getFIFOSampleSize();
	
	//find the number of packets
	int max_fifo_samples = (1024 / getFIFOSampleSize()) *getFIFOSampleSize();
	tv = max_fifo_samples;
	//untested
	//wait for FIFO to get these samples
	int count  = samples;
	int div = 0;
	long ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
	while(count>0){ // take all the samples
		while(mpu.getFIFOCount()<min(max_fifo_samples, count)){
			;//spin waiting for FIFO to obtain the data
		}
		parseSensorValues();
		ax += accel_sensor.x;
		ay += accel_sensor.y;
		az += accel_sensor.z;
		gx += gyro_sensor.x;
		gy += gyro_sensor.y;
		gz += gyro_sensor.z;
		
		div++;
		count -= min(max_fifo_samples, count);
	}
	
	//set the averages as offsets
	mpu.setXAccelOffset((int16_t)ax/div);
    mpu.setYAccelOffset((int16_t)ay/div);
    mpu.setZAccelOffset((int16_t)az/div);
    mpu.setXGyroOffset ((int16_t)gx/div);
    mpu.setYGyroOffset ((int16_t)gy/div);
    mpu.setZGyroOffset ((int16_t)gz/div);
	
	//set the sample rate as before
	setSampleRate(previous_sample_rate);
}
//use custom offsets calculated by IMU_Zero sketch
/*
XAccel		[-5039,-5038] --> [-9,5]		
YAccel		[-807,-806] --> [-7,9]
ZAccel		[1181,1181] --> [16384,16388]	
XGyro		[150,151] --> [0,2]	
YGyro		[64,65] --> [-2,1]
ZGyro		[16,17] --> [0,4]
*/
//					
/*void MPU6050Wrapper::setProperOffsets(){
	mpu.setXAccelOffset(-5039);
    mpu.setYAccelOffset(-807);
    mpu.setZAccelOffset(1181);
    mpu.setXGyroOffset (150);
    mpu.setYGyroOffset (64);
    mpu.setZGyroOffset (16);
}*/
void MPU6050Wrapper::resetOffsets(){
	mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset (0);
    mpu.setYGyroOffset (0);
    mpu.setZGyroOffset (0);
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