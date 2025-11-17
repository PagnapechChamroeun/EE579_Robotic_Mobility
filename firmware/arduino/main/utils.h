#pragma once 
#include <Arduino.h> 

inline float clampf(float x, float low, float high) {
    return (x < low) ? low : (x > high) ? high : x; 
}

inline float deg2rad(float d) { return d * PI / 180.0f; }
inline float rad2deg(float r) { return r * 180.0f / PI; } 

// map float -> float 
inline float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; 
}