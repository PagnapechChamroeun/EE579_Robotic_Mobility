#include <Arduino.h> 
#include <DynamixelShield.h> 
#include <Adafruit_BNO055.h> 
#include <Adafruit_Sensor.h> 

#include "ids.h"
#include "utils.h"
#include "cpg.h"

// ---------- Globals ---------- 
DynamixelShield dxl; 
Adafruit_BNO055 bno = Adafruit_BNO055(55); // 0x28 or 0x29 
TrotCPG cpg; 

unsigned long lastMicros = 0; 
const float loopHz = 100.0f; // control loop 
const float dt_s = 1.0f / loopHz; 

// helpers
bool contactLF = false; // left front 
bool contactRF = false; // right front 
bool contactLR = false; // left rear
bool contactRR = false; // right rear 

// XL-320 dynamxiel 
const float PROTOCOL_VERSION = 2.0; 

// 0 to 300 deg for XL-320 
uint16_t degToPosXL320(float deg) {
  // XL-320 has 0 -> 1023 maps 0 -> 300 deg 
  float d = clampf(deg, 0.0f, 300.0f);
  return (uint16_t) roundf(d * (1023.0f / 300.0f));   
}

// one joint (deg) with clamp to per-joint limit 
void writeJoint(uint8_t id, float deg, float lowDeg, float highDeg) {
  float clamped = clampf(deg, lowDeg, highDeg); 
  dxl.setGoalPosition(id, degToPosXL320(clamped)); 
}

// enable torque for all IDs 
void enableTorqueAll(bool on=true) {
  for (uint8_t id=1; id<=8; id++) {
    dxl.torqueOn(id, on); 
    delay(5); 
  }
}

// // read FSRs
// void updateFSR() {
//   contactLF = analogRead(PIN_FSR_LF) > FSR_CONTACT_THRESH; 
//   contactRF = analogRead(PIN_FSR_RF) > FSR_CONTACT_THRESH; 
//   contactLR = analogRead(PIN_FSR_LR) > FSR_CONTACT_THRESH; 
//   contactRR = analogRead(PIN_FSR_RR) > FSR_CONTACT_THRESH; 
// }

// IMU tilt (deg)
void getIMUTilt(float& pitchDeg, float& rollDeg) {
  // sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
  // bno.getEvent(&orientationData, Adafruit_BNO055::???) // VECTOR_EULER or QUATERNION???
  /*
    pitchDeg = orientationData.orientation.z; // Adafruit's Euler: x=heading, y=roll, z=pitch (deg)
    rollDeg = orientationData.orientation.y; 
  */
  imu::Quaternion quat = bno.getQuat();
  // check formula again 
  pitchDeg = atan2f(2.0f * (quat.w() * quat.y() - quat.z() * quat.x()),
                     1.0f - 2.0f * (quat.y() * quat.y() + quat.x() * quat.x())) * (180.0f / PI);
  rollDeg = asinf(2.0f * (quat.w() * quat.x() + quat.y() * quat.z())) * (180.0f / PI);
}

// apply small tilt trims to hips to resist tipping 
// Positive pitch -> nose up -> bias hips to push forward slightly 
void applyIMUTrims(float pitchDeg, float rollDeg, 
                  float& hipLF, float& hipRF, float& hipLR, float& hipRR) {
  float pTrim = PITCH_TRIM_GAIN * pitchDeg; 
  float rTrim = ROLL_TRIM_GAIN * rollDeg; 
  
  // Roll: +roll tips right side down -> push right hips outward (neg on left, pos on right)
  hipLF -= rTrim; hipLR -= rTrim;  // left legs
  hipRF += rTrim; hipRR += rTrim;  // right legs

  // Pitch: +pitch = nose up -> push front hips forward (increase)
  hipLF += pTrim; hipRF += pTrim;  // front
  hipLR -= pTrim; hipRR -= pTrim;  // rear
}

// void centerAll() {
//   writeJoint(ID_LF_HIP, HIP_NEUTRAL_DEG, HIP_MIN_DEG, HIP_MAX_DEG);
//   writeJoint(ID_LF_KNEE, KNEE_NEUTRAL_DEG, KNEE_MIN_DEG, KNEE_MAX_DEG);
//   writeJoint(ID_RF_HIP, HIP_NEUTRAL_DEG, HIP_MIN_DEG, HIP_MAX_DEG);
//   writeJoint(ID_RF_KNEE, KNEE_NEUTRAL_DEG, KNEE_MIN_DEG, KNEE_MAX_DEG);
//   writeJoint(ID_LR_HIP, HIP_NEUTRAL_DEG, HIP_MIN_DEG, HIP_MAX_DEG);
//   writeJoint(ID_LR_KNEE, KNEE_NEUTRAL_DEG, KNEE_MIN_DEG, KNEE_MAX_DEG);
//   writeJoint(ID_RR_HIP, HIP_NEUTRAL_DEG, HIP_MIN_DEG, HIP_MAX_DEG);
//   writeJoint(ID_RR_KNEE, KNEE_NEUTRAL_DEG, KNEE_MIN_DEG, KNEE_MAX_DEG);
// }
void centerAll() {
  for (uint8_t id=1; id<=8; id+=2) { // hips
    writeJoint(id, HIP_NEUTRAL_DEG, HIP_MIN_DEG, HIP_MAX_DEG);
  }
  for (uint8_t id=2; id<=8; id+=2) { // knees
    writeJoint(id, KNEE_NEUTRAL_DEG, KNEE_MIN_DEG, KNEE_MAX_DEG);
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial) {}

  // Dynamixel bus 
  dxl.begin(...);
  dxl.setPortProtocolVersion(PROTOCOL_VERSION); // Protocol 2.0 

  // XL-320 in joint mode (position)
  for (uint8_t id=1; id<=8; id++) {
    dxl.setOperatingMode(id, OP_POSITION);
    delay(5);
  }
  enableTorqueAll(true);
  centerAll(); 

  // IMU init
  if(!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring/address.");
    while(1);
  } else {
    bno.setExtCrystalUse(true);
  }

  // CPG init
  cpg.init(CPG_FREQ_HZ, CPG_HIP_AMP_DEG, CPG_KNEE_AMP_DEG); 

  // FSR pins 
  pinMode(PIN_FSR_LF, INPUT);
  pinMode(PIN_FSR_RF, INPUT);
  pinMode(PIN_FSR_LR, INPUT);
  pinMode(PIN_FSR_RR, INPUT);
  
  lastMicros = micros(); 
  Serial.println("Quadruped: Ready."); 

}

void loop() {
  // timing
  unsigned long now = micros(); 
  if ((now - lastMicros) < (unsigned long) (1e6f * dt_s)) {
    return;
  } 
  lastMicros = now; 

  // Sensors
  updateFSR(); 
  float pitchDeg = 0;
  float rollDeg = 0;
  getIMUTilt(pitchDeg, rollDeg); 

  // CPG advance 
  cpg.step(dt_s);

  // targets (before trims)
  float hipLF, kneeLF, hipRF, kneeRF, hipLR, kneeLR, hipRR, kneeRR; 
  cpg.legTargets(cpg.phaseLF(), HIP_NEUTRAL_DEG, KNEE_NEUTRAL_DEG, hipLF, kneeLF);
  cpg.legTargets(cpg.phaseRF(), HIP_NEUTRAL_DEG, KNEE_NEUTRAL_DEG, hipRF, kneeRF);
  cpg.legTargets(cpg.phaseLR(), HIP_NEUTRAL_DEG, KNEE_NEUTRAL_DEG, hipLR, kneeLR);
  cpg.legTargets(cpg.phaseRR(), HIP_NEUTRAL_DEG, KNEE_NEUTRAL_DEG, hipRR, kneeRR);

  // simple contact gating: if a foot is in contact, slightly reduce knee flex to stiffen stance
  const float stanceKneeReduce = 8.0f; 
  if (contactLF) kneeLF = kneeLF - stanceKneeReduce;
  if (contactRF) kneeRF = kneeRF - stanceKneeReduce;
  if (contactLR) kneeLR = kneeLR - stanceKneeReduce;
  if (contactRR) kneeRR = kneeRR - stanceKneeReduce;

  // IMU trims (tilt compensation)
  applyIMUTrims(pitchDeg, rollDeg, hipLF, hipRF, hipLR, hipRR); 

  // Write hips
  writeJoint(ID_LF_HIP, hipLF, HIP_MIN_DEG, HIP_MAX_DEG);
  writeJoint(ID_RF_HIP, hipRF, HIP_MIN_DEG, HIP_MAX_DEG);
  writeJoint(ID_LR_HIP, hipLR, HIP_MIN_DEG, HIP_MAX_DEG);
  writeJoint(ID_RR_HIP, hipRR, HIP_MIN_DEG, HIP_MAX_DEG);

  // Write knees
  writeJoint(ID_LF_KNEE, kneeLF, KNEE_MIN_DEG, KNEE_MAX_DEG);
  writeJoint(ID_RF_KNEE, kneeRF, KNEE_MIN_DEG, KNEE_MAX_DEG);
  writeJoint(ID_LR_KNEE, kneeLR, KNEE_MIN_DEG, KNEE_MAX_DEG);
  writeJoint(ID_RR_KNEE, kneeRR, KNEE_MIN_DEG, KNEE_MAX_DEG);

  // Optional: debug at 10 Hz
  static int dbg = 0;
  if (++dbg >= 10) {
    dbg = 0;
    Serial.print("Pitch: "); Serial.print(pitchDeg);
    Serial.print("  Roll: "); Serial.print(rollDeg);
    Serial.print(" | FSR: ");
    Serial.print(contactLF); Serial.print(contactRF);
    Serial.print(contactLR); Serial.println(contactRR);
  }

}
