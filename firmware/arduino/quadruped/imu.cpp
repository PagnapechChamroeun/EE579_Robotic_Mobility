#include "imu.h"

#include <DynamixelShield.h>
#include <stdlib.h>
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  extern SoftwareSerial soft_serial;  // declared in your .ino
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// IMU baseline ("robot is level")  -- initialization for calibration 
float imu_roll0 = 0.0f;
float imu_pitch0 = 0.0f;
float imu_yaw0 = 0.0f; 

// live IMU reading 
float imu_roll = 0.0f; 
float imu_pitch = 0.0f; 
float imu_yaw = 0.0f; 

// flags
bool imu_ok = false; 

// weight penalty/reward to error 
const float K_ROLL = 0.1f;  // deg of hip correction per deg of roll error 
const float K_PITCH = 0.05f; // deg of hip correction per deg of pitch error 

void readIMU(float& roll_deg, float& pitch_deg, float& yaw_deg) {
    // imu has problem 
    if (!imu_ok) {
        // return 0.0
        roll_deg = pitch_deg = yaw_deg = 0.0f;
        return; 
    }

    // VECTOR_EULER: yaw (x), roll (y), pitch (z), in degrees
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

    float yaw = euler.x(); 
    float roll = euler.y(); 
    float pitch = euler.z(); 

    roll_deg = roll; 
    pitch_deg = pitch; 
    yaw_deg = yaw; 
}

void calibrateIMU() {
    DEBUG_SERIAL.println("Calibrating IMU baseline... keep robot standing level.");

    const int N = 1000; // number of samples to average 
    float roll_sum = 0.0f; 
    float pitch_sum = 0.0f; 
    float yaw_sum = 0.0f; 

    for (int i=0; i<N; ++i) {
        float r, p, y; 
        readIMU(r, p, y);
        roll_sum += r; 
        pitch_sum += p; 
        yaw_sum += y; 
        delay(10); // 1000 * 10 ms = 10 seconds 
    }

    imu_roll0 = roll_sum / N; 
    imu_pitch0 = pitch_sum / N; 
    imu_yaw0 = yaw_sum / N; 

    DEBUG_SERIAL.print("IMU baseline -> roll0: ");
    DEBUG_SERIAL.print(imu_roll0);
    DEBUG_SERIAL.print(" pitch0: ");
    DEBUG_SERIAL.print(imu_pitch0);
    DEBUG_SERIAL.print(" yaw0: ");
    DEBUG_SERIAL.print(imu_yaw0); 
}

// IMU-based correction function 
float imu_correction_deg(int leg_index, float roll_err, float pitch_err) {
    // roll = left/right tilt 
    // pitch = forward/back tilt 
    // yaw = heading (no effect on falling)

    float corr = 0.0f; 

    // NOTE: leg_index starts from 0 to 7

    bool is_hip = (leg_index % 2 == 0); // 0,2,4,6 are hips 
    if (!is_hip) return 0.0f; 

    bool is_front = (leg_index == 0 || leg_index == 1 || leg_index == 2 || leg_index == 3);
    bool is_left  = (leg_index == 0 || leg_index == 1 || leg_index == 4 || leg_index == 5);

    // Lateral stabilization (roll)
    // if lean left
    if (is_left) { // tweak sign as needed
        corr -= K_ROLL * roll_err; 
    } else {
        corr += K_ROLL * roll_err; 
    }

    // Fore-aft stabilization (pitch) 
    // if lean forward
    if (is_front) {
        corr += K_PITCH * pitch_err;
    } else {
        corr -= K_PITCH * pitch_err; 
    }

    // clamp
    const float MAX_CORR = 4.0f; // degrees 
    if (corr > MAX_CORR) corr  = MAX_CORR; 
    if (corr < -MAX_CORR) corr = -MAX_CORR; 

    return corr; 
}