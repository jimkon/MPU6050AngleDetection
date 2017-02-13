# MPU6050AngleDetection
This is a library that helps user have an precise measurement of the angle in XYZ axis using an MPU6050 sensor. 
This library is a wrapper of MPU6050 library implementation by Jeff Rowberg (https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)

User can choose which and how many axis will be enabled. This will infuence the rate that fifo is filled, thus the rate we have to clear it. 