#pragma once
#include <Arduino.h> 

// ultra-lite symmetric trot CPG using sin waves with 180 deg leg-pair offsets 
// use Hopf if needed 

struct TrotCPG {
    float freq_hz;      // base frequency
    float hip_amp_deg;  // hip swing amplitude 
    float knee_amp_deg; // knee amplitude 
    float phase;        // global phase [rad], 0 to 2* pi 

    void init(float f, float hipA, float kneeA) {
        /*
        Input:
            - f: frequency (Hz)
            - hipA: hip amplitude (cm)
            - kneeA: knee amplitude (cm)
        */
       freq_hz = f; 
       hip_amp_deg = hipA; 
       knee_amp_deg = kneeA; 
       phase = 0.0f; 
    }

    // advance by dt seconds
    void step(float dt) {
        phase += 2.0f * PI * freq_hz * dt; 
        if (phase > TWO_PI) {
            phase -= TWO_PI; 
        }
    }

    // For each leg, provide per-leg phase offsets for a trot 
    // LF-RR (in-phase) and RF-LR opposite (pi phase difference) 
    // return (hipDeg, kneeDeg) around neutral, before trimming/clamping 
    void legTargets(float basePhase, float hipNeutral, float kneeNeutral, 
                    float& hipDeg, float& kneeDeg) const {
        float s = sin(basePhase);
        float c = sin(basePhase + PI / 2.0f); // knee is slightly phase-shifted from hip for clearance
        hipDeg  = hipNeutral + hip_amp_deg * s; 
        kneeDeg = kneeNeutral + knee_amp_deg * c; 
    }

    // per-leg phases
    float phaseLF() const { return phase; }
    float phaseRR() const { return phase; }
    float phaseRF() const { return phase + PI; }
    float phaseLR() const { return phase + PI; } 

};

