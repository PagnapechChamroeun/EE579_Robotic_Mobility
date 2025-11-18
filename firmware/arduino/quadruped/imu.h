#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


extern Adafruit_BNO055 bno;

// IMU baseline ("robot is level")  -- initialization for calibration 
extern float imu_roll0;
extern float imu_pitch0;
extern float imu_yaw0;

// live IMU reading 
extern float imu_roll; 
extern float imu_pitch; 
extern float imu_yaw; 

// flags
extern bool imu_ok; 

void readIMU(float& roll_deg, float& pitch_deg, float& yaw_deg); 
void calibrateIMU(); 
// IMU-based correction function 
float imu_correction_deg(int leg_index, float roll_err, float pitch_err); 


#endif 
