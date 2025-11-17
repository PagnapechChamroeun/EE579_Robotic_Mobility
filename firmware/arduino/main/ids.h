// compile once 
#pragma once 

// left front hip 
#define ID_LF_HIP   1
// left front knee
#define ID_LF_KNEE  2
// right front hip
#define ID_RF_HIP   3
// right front knee
#define ID_RF_KNEE  4
// left rear hip 
#define ID_LR_HIP   5
// left rear knee
#define ID_LR_KNEE  6
// right rear hip 
#define ID_RR_HIP   7
// right rear knee
#define ID_RR_KNEE  8


// Force Sense Resistor (FSR) analog pins
#define PIN_FSR_LF  A0  // left front 
#define PIN_FSR_RF  A1  // right front 
#define PIN_FSR_LR  A2  // left rear 
#define PIN_FSR_RR  A3  // right rear 

// FSR contact threshold
#define FSR_CONTACT_THRESH 200 

// Gait / Joint limits (degrees)
#define HIP_MIN_DEG     -35
#define HIP_MAX_DEG      35
#define KNEE_MIN_DEG    -40 
#define KNEE_MAX_DEG     5

// Default neutral angles (degrees)
#define HIP_NEUTRAL_DEG      0 
#define KNEE_NEUTRAL_DEG    -15

// CPG base frequency (Hz) and amplitude (deg) 
#define CPG_FREQ_HZ         0.8
#define CPG_HIP_AMP_DEG     18 
#define CPG_KNEE_AMP_DEG    20 

// IMU tilt gains (deg/deg)
#define PITCH_TRIM_GAIN     0.6f
#define ROLL_TRIM_GAIN      0.6f 