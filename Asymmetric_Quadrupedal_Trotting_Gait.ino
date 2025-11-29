/*
 * Symmetric Trotting Gait with Inverse Kinematics for Climbing
 * 
 * Coordinate system:
 *   - X: positive = forward (anterior)
 *   - Y: positive = down (ventral, toward ground)
 *   - theta 1 and theta 2 positive CCW, theta 2 = 0 when fully extended
 * 
 * Forward Kinematics:
 *   x = L1*sin(theta1) + L2*sin(theta1 + theta2)
 *   y = L1*cos(theta1) + L2*cos(theta1 + theta2)
*/

#include <DynamixelShield.h>
#include <math.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8);
  #define DEBUG_SERIAL soft_serial
#else
  #define DEBUG_SERIAL Serial
#endif

const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;


// ==============================================================
// HARDWARE CONFIG

const int TOTAL_MOTORS = 8;
uint8_t motor_IDs[] = {1, 2, 3, 4, 5, 6, 7, 8};
const char* motor_names[] = {"LF_Hip", "LF_Knee", "RF_Hip", "RF_Knee", 
                             "LR_Hip", "LR_Knee", "RR_Hip", "RR_Knee"};
const char* leg_names[] = {"LF", "RF", "LR", "RR"};

// Motor directions: 1 = reversed, 0 = normal
// Left side motors are reversed (negative torque for extension)
uint8_t motor_directions[] = {1, 1, 0, 0, 1, 1, 0, 0};

// Zero position offsets for each motor (deg)
int motor_zero_offsets[] = {58, 146, 239, 148, 148, 241, 154, 149};

// Leg link lengths (cm)
const float L1 = 6.9;   // Femur length (6.6)
const float L2 = 8.8;   // Tibia length (8.5)

const float PI_F = 3.14159265;


// ==============================================================
// GAIT PARAMS

// Trajectory parameters (cm)
float stride_length = 8.0;          // Total horizontal travel per stride
float stance_depth = 13.4;          // Foot depth below hip during stance
float swing_height = 8.0;           // Maximum lift during swing
float swing_peak_position = 0.15;   // Normalized position of max height in swing (0-1)

// Timing parameters
float gait_period = 8.0;            // Duration of one gait cycle (seconds)
float duty_cycle = 0.95;            // Proportion of cycle spent in stance

// ==============================================================
// CLIMBING PARAMS

// Body pitch compensation - angles front of robot down toward incline
// Positive = nose down. Front legs get shallower trajectory, back legs deeper.
float body_pitch_offset = -2.0;     // deg

// Force angle offset - rotates trajectory to "dig in" on slopes
// Positive = legs start stance phase lower, then gradually rise (negative better for climbing)
float force_angle_offset = -2.0;    // deg

// Horizontal offset for all trajectories
// Positive = shift trajectories forward, Negative = shift backward
float gait_x_offset_front = -2.0;   // cm - offset for front legs (LF, RF)
float gait_x_offset_back = -6.0;    // cm - offset for back legs (LR, RR)

// Inter-diagonal phase offset (asymmetric trot parameter)
// Phase difference between diagonal leg pairs:
//   delta_phi_a = phi_second_pair - phi_first_pair (in degrees)
// First pair: LF + RR (reference, always starts at phase 0)
// Second pair: RF + LR
// delta_phi_a = 180° -> symmetric trot (default)
// delta_phi_a < 180° -> second pair enters stance earlier
// delta_phi_a > 180° -> second pair enters stance later
float delta_phi_a = 200.0;          // deg (default 180)

// Body geometry for pitch calculations
const float BODY_HALF_LENGTH = 9.0;  // Distance from body center to front/rear hips (cm)


/* Used params for 0 deg
// ==============================================================
// GAIT PARAMS

// Trajectory parameters (cm)
float stride_length = 10.0;         // Total horizontal travel per stride
float stance_depth = 13.0;          // Foot depth below hip during stance
float swing_height = 8.0;           // Maximum lift during swing
float swing_peak_position = 0.3;    // Normalized position of max height in swing (0-1)

// Timing parameters
float gait_period = 10.0;           // Duration of one gait cycle (seconds)
float duty_cycle = 0.95;            // Proportion of cycle spent in stance

// ==============================================================
// CLIMBING PARAMS

// Body pitch compensation - angles front of robot down toward incline
// Positive = nose down. Front legs get shallower trajectory, back legs deeper.
float body_pitch_offset = 5.0;      // deg

// Force angle offset - rotates trajectory to "dig in" on slopes
// Positive = legs start stance phase lower, then gradually rise (negative better for climbing)
float force_angle_offset = -2.0;    // deg

// Horizontal offset for all trajectories
// Positive = shift trajectories forward, Negative = shift backward
float gait_x_offset_front = -1.0;   // cm - offset for front legs (LF, RF)
float gait_x_offset_back = -5.0;    // cm - offset for back legs (LR, RR)

// Inter-diagonal phase offset (asymmetric trot parameter)
// Phase difference between diagonal leg pairs:
//   delta_phi_a = phi_second_pair - phi_first_pair (in degrees)
// First pair: LF + RR (reference, always starts at phase 0)
// Second pair: RF + LR
// delta_phi_a = 180° -> symmetric trot (default)
// delta_phi_a < 180° -> second pair enters stance earlier
// delta_phi_a > 180° -> second pair enters stance later
float delta_phi_a = 180.0;          // deg (default 180)

// Body geometry for pitch calculations
const float BODY_HALF_LENGTH = 9.0;  // Distance from body center to front/rear hips (cm)
*/


/* Used params for 5 deg
// ==============================================================
// GAIT PARAMS

// Trajectory parameters (cm)
float stride_length = 10.0;          // Total horizontal travel per stride
float stance_depth = 13.2;          // Foot depth below hip during stance
float swing_height = 8.0;           // Maximum lift during swing
float swing_peak_position = 0.15;   // Normalized position of max height in swing (0-1)

// Timing parameters
float gait_period = 8.0;            // Duration of one gait cycle (seconds)
float duty_cycle = 0.95;            // Proportion of cycle spent in stance

// ==============================================================
// CLIMBING PARAMS

// Body pitch compensation - angles front of robot down toward incline
// Positive = nose down. Front legs get shallower trajectory, back legs deeper.
float body_pitch_offset = -5.0;     // deg

// Force angle offset - rotates trajectory to "dig in" on slopes
// Positive = legs start stance phase lower, then gradually rise (negative better for climbing)
float force_angle_offset = -2.0;    // deg

// Horizontal offset for all trajectories
// Positive = shift trajectories forward, Negative = shift backward
float gait_x_offset_front = 1.0;    // cm - offset for front legs (LF, RF)
float gait_x_offset_back = -5.0;    // cm - offset for back legs (LR, RR)

// Inter-diagonal phase offset (asymmetric trot parameter)
// Phase difference between diagonal leg pairs:
//   delta_phi_a = phi_second_pair - phi_first_pair (in degrees)
// First pair: LF + RR (reference, always starts at phase 0)
// Second pair: RF + LR
// delta_phi_a = 180° -> symmetric trot (default)
// delta_phi_a < 180° -> second pair enters stance earlier
// delta_phi_a > 180° -> second pair enters stance later
float delta_phi_a = 180.0;          // deg (default 180)

// Body geometry for pitch calculations
const float BODY_HALF_LENGTH = 9.0;  // Distance from body center to front/rear hips (cm)
*/


/* Used params for 10 deg
// ==============================================================
// GAIT PARAMS

// Trajectory parameters (cm)
float stride_length = 8.0;          // Total horizontal travel per stride
float stance_depth = 13.4;          // Foot depth below hip during stance
float swing_height = 8.0;           // Maximum lift during swing
float swing_peak_position = 0.15;   // Normalized position of max height in swing (0-1)

// Timing parameters
float gait_period = 8.0;            // Duration of one gait cycle (seconds)
float duty_cycle = 0.95;            // Proportion of cycle spent in stance

// ==============================================================
// CLIMBING PARAMS

// Body pitch compensation - angles front of robot down toward incline
// Positive = nose down. Front legs get shallower trajectory, back legs deeper.
float body_pitch_offset = -2.0;     // deg

// Force angle offset - rotates trajectory to "dig in" on slopes
// Positive = legs start stance phase lower, then gradually rise (negative better for climbing)
float force_angle_offset = -2.0;    // deg

// Horizontal offset for all trajectories
// Positive = shift trajectories forward, Negative = shift backward
float gait_x_offset_front = -2.0;   // cm - offset for front legs (LF, RF)
float gait_x_offset_back = -6.0;    // cm - offset for back legs (LR, RR)

// Inter-diagonal phase offset (asymmetric trot parameter)
// Phase difference between diagonal leg pairs:
//   delta_phi_a = phi_second_pair - phi_first_pair (in degrees)
// First pair: LF + RR (reference, always starts at phase 0)
// Second pair: RF + LR
// delta_phi_a = 180° -> symmetric trot (default)
// delta_phi_a < 180° -> second pair enters stance earlier
// delta_phi_a > 180° -> second pair enters stance later
float delta_phi_a = 180.0;          // deg (default 180)

// Body geometry for pitch calculations
const float BODY_HALF_LENGTH = 9.0;  // Distance from body center to front/rear hips (cm)
*/


// ==============================================================
// DEADZONE TRACKING

// Track if any motor exceeded safe range during the cycle
bool deadzone_warnings[4] = {false, false, false, false};  // Per leg
float max_motor_position[8] = {0};  // Track max position per motor this cycle


// ==============================================================
// LEG STRUCTURE

struct LegAngles {
  float theta1;  // Hip angle (rad)
  float theta2;  // Knee angle (rad)
  bool valid;    // IK soln exists
};

struct FootPosition {
  float x;  // Forward position (cm)
  float y;  // Downward position (cm)
};


// ==============================================================
// FORWARD KINEMATICS

FootPosition forwardKinematics(float theta1, float theta2) {
  FootPosition foot;
  float theta_total = theta1 + theta2;
  
  foot.x = L1 * sin(theta1) + L2 * sin(theta_total);
  foot.y = L1 * cos(theta1) + L2 * cos(theta_total);
  
  return foot;
}


// ==============================================================
// INVERSE KINEMATICS

LegAngles inverseKinematics(float x, float y) {
  LegAngles result;
  result.valid = false;
  
  // Dist from hip to target
  float r_sq = x * x + y * y;
  float r = sqrt(r_sq);
  
  // Check reachability
  float r_max = L1 + L2 - 0.1;  // Small margin from singularity
  float r_min = fabs(L1 - L2) + 0.1;
  
  if (r > r_max || r < r_min) {
    return result;
  }
  
  // theta 2 via law of cosines
  float cos_theta2 = (r_sq - L1*L1 - L2*L2) / (2.0 * L1 * L2);
  cos_theta2 = constrain(cos_theta2, -1.0, 1.0);
  result.theta2 = acos(cos_theta2);
  
  // theta1
  float phi = atan2(x, y);
  float beta = atan2(L2 * sin(result.theta2), L1 + L2 * cos(result.theta2));
  result.theta1 = phi - beta;
  
  result.valid = true;
  return result;
}


// ==============================================================
// TRAJECTORY GENERATION

// Apply rotation to a point (force_angle_offset)
void rotatePoint(float& x, float& y, float angle_rad) {
  float cos_a = cos(angle_rad);
  float sin_a = sin(angle_rad);
  float x_new = x * cos_a - y * sin_a;
  float y_new = x * sin_a + y * cos_a;
  x = x_new;
  y = y_new;
}

// Get base foot position at a given phase before leg-specific adjustments
FootPosition getBaseFootPosition(float phase) {
  FootPosition foot;
  
  // Stance boundaries
  float x_front = stride_length / 2.0;
  float x_back = -stride_length / 2.0;
  
  if (phase < duty_cycle) {
    // STANCE PHASE
    float stance_progress = phase / duty_cycle;
    foot.x = x_front - stride_length * stance_progress;
    foot.y = stance_depth;
  } else {
    // SWING PHASE
    float swing_progress = (phase - duty_cycle) / (1.0 - duty_cycle);
    
    // Linear x motion
    foot.x = x_back + stride_length * swing_progress;
    
    // Parabolic height
    float t = swing_progress;
    float p = swing_peak_position;
    
    float height_factor;
    if (fabs(p - 0.5) < 0.01) {
      height_factor = 4.0 * t * (1.0 - t);
    } else {
      float a = 1.0 / (p * (p - 1.0));
      float b = -a;
      height_factor = a * t * t + b * t;
      height_factor = constrain(height_factor, 0.0, 1.0);
    }
    
    foot.y = stance_depth - swing_height * height_factor;
  }
  
  return foot;
}


// Get foot position for a specific leg with all climbing adjustments
// leg_index: 0=LF, 1=RF, 2=LR, 3=RR
FootPosition getFootPositionForLeg(float phase, int leg_index) {
  FootPosition foot = getBaseFootPosition(phase);
  
  // Determine if front or back leg
  bool is_front_leg = (leg_index == 0 || leg_index == 1); 
  
  // BODY PITCH ADJUSTMENT
  // Positive body_pitch_offset = nose down
  // Front legs: hip is higher relative to ground -> reduce stance_depth
  // Back legs: hip is lower relative to ground -> increase stance_depth
  if (fabs(body_pitch_offset) > 0.01) {
    float pitch_rad = body_pitch_offset * DEG_TO_RAD;
    
    // Vertical offset due to body pitch
    // Front hips rise, back hips drop when pitched forward
    float depth_adjustment = BODY_HALF_LENGTH * sin(pitch_rad);
    
    if (is_front_leg) {
      // Front legs: hip is higher, so foot needs to reach less far
      foot.y -= depth_adjustment;
    } else {
      // Back legs: hip is lower, so foot needs to reach further
      foot.y += depth_adjustment;
    }
  }
  
  // HORIZONTAL OFFSET
  if (is_front_leg) {
    foot.x += gait_x_offset_front;
  } else {
    foot.x += gait_x_offset_back;
  }
  
  // FORCE ANGLE ROTATION
  // Rotate entire trajectory around the hip to "dig in" on slopes
  // Positive angle rotates trajectory so stance pushes into slope
  if (fabs(force_angle_offset) > 0.01) {
    float force_rad = force_angle_offset * DEG_TO_RAD;
    rotatePoint(foot.x, foot.y, force_rad);
  }
  
  return foot;
}


// ==============================================================
// MOTOR CONTROL

// Convert joint angle to motor position
float angleToMotorPosition(int motor_index, float angle_rad) {
  float angle_deg = angle_rad * RAD_TO_DEG;
  
  // Apply direction
  if (motor_directions[motor_index]) {
    angle_deg = -angle_deg;
  }
  
  // Apply offset
  float position_deg = motor_zero_offsets[motor_index] + angle_deg;
  
  // Wrap to 0-360 range
  while (position_deg < 0) position_deg += 360.0;
  while (position_deg >= 360) position_deg -= 360.0;
  
  return position_deg;
}

// Set a single motor to target position and track for deadzone
void setMotorAngle(int motor_index, float angle_rad, int leg_index) {
  float position_deg = angleToMotorPosition(motor_index, angle_rad);
  
  // Track maximum position for deadzone warning
  if (position_deg > max_motor_position[motor_index]) {
    max_motor_position[motor_index] = position_deg;
  }
  
  // Check deadzone (> 300 degrees)
  if (position_deg > 300.0) {
    deadzone_warnings[leg_index] = true;
  }
  
  dxl.setGoalPosition(motor_IDs[motor_index], position_deg, UNIT_DEGREE);
}

// Set both motors of a leg
void setLegAngles(int leg_index, float theta1, float theta2) {
  int hip_index = leg_index * 2;
  int knee_index = leg_index * 2 + 1;
  
  setMotorAngle(hip_index, theta1, leg_index);
  setMotorAngle(knee_index, theta2, leg_index);
}


// ==============================================================
// GAIT COORDINATION

// leg_index: 0=LF, 1=RF, 2=LR, 3=RR
// First pair (LF + RR): phase offset = 0
// Second pair (RF + LR): phase offset = delta_phi_a / 360
float getLegPhaseOffset(int leg_index) {
  // Convert delta_phi_a from degrees to normalized phase (0-1)
  float second_pair_offset = (360.0 - delta_phi_a) / 360.0;
  if (second_pair_offset >= 1.0) second_pair_offset -= 1.0;

  switch(leg_index) {
    case 0: return 0.0;                 // LF - first diagonal pair (ref)
    case 1: return second_pair_offset;  // RF - second diagonal pair
    case 2: return second_pair_offset;  // LR - second diagonal pair
    case 3: return 0.0;                 // RR - first diagonal pair
    default: return 0.0;
  }
}


// ==============================================================
// DEBUG / DEADZONE REPORTING

void printDeadzoneReport() {
  bool any_warning = false;
  
  for (int leg = 0; leg < 4; leg++) {
    if (deadzone_warnings[leg]) {
      any_warning = true;
    }
  }
  
  if (any_warning) {
    DEBUG_SERIAL.println("\n*** DEADZONE WARNING ***");
    for (int leg = 0; leg < 4; leg++) {
      if (deadzone_warnings[leg]) {
        DEBUG_SERIAL.print("  ");
        DEBUG_SERIAL.print(leg_names[leg]);
        DEBUG_SERIAL.print(": Hip max=");
        DEBUG_SERIAL.print(max_motor_position[leg * 2], 1);
        DEBUG_SERIAL.print("°, Knee max=");
        DEBUG_SERIAL.print(max_motor_position[leg * 2 + 1], 1);
        DEBUG_SERIAL.println("°");
      }
    }
    DEBUG_SERIAL.println("************************\n");
  }
  
  // Reset tracking for next cycle
  for (int leg = 0; leg < 4; leg++) {
    deadzone_warnings[leg] = false;
  }
  for (int i = 0; i < 8; i++) {
    max_motor_position[i] = 0;
  }
}


// ==============================================================
// MAIN SETUP & LOOP

void setup() {
  DEBUG_SERIAL.begin(115200);
  delay(1000);
  
  DEBUG_SERIAL.println("\nSymmetric Trot with IK for Climbing");
  
  // Print parameters
  DEBUG_SERIAL.println("Leg Parameters");
  DEBUG_SERIAL.print("L1 (femur): "); DEBUG_SERIAL.print(L1); DEBUG_SERIAL.println(" cm");
  DEBUG_SERIAL.print("L2 (tibia): "); DEBUG_SERIAL.print(L2); DEBUG_SERIAL.println(" cm");
  DEBUG_SERIAL.print("Max reach: "); DEBUG_SERIAL.print(L1+L2); DEBUG_SERIAL.println(" cm");
  DEBUG_SERIAL.print("Min reach: "); DEBUG_SERIAL.print(fabs(L1-L2)); DEBUG_SERIAL.println(" cm");
  
  DEBUG_SERIAL.println("\nGait Parameters");
  DEBUG_SERIAL.print("Stride length: "); DEBUG_SERIAL.print(stride_length); DEBUG_SERIAL.println(" cm");
  DEBUG_SERIAL.print("Stance depth: "); DEBUG_SERIAL.print(stance_depth); DEBUG_SERIAL.println(" cm");
  DEBUG_SERIAL.print("Swing height: "); DEBUG_SERIAL.print(swing_height); DEBUG_SERIAL.println(" cm");
  DEBUG_SERIAL.print("Period: "); DEBUG_SERIAL.print(gait_period); DEBUG_SERIAL.println(" s");
  DEBUG_SERIAL.print("Duty cycle: "); DEBUG_SERIAL.print(duty_cycle * 100); DEBUG_SERIAL.println("%");
  
  DEBUG_SERIAL.println("\nClimbing Parameters");
  DEBUG_SERIAL.print("Body pitch offset: "); DEBUG_SERIAL.print(body_pitch_offset); DEBUG_SERIAL.println(" deg");
  DEBUG_SERIAL.print("Force angle offset: "); DEBUG_SERIAL.print(force_angle_offset); DEBUG_SERIAL.println(" deg");
  DEBUG_SERIAL.print("Gait X offset (front): "); DEBUG_SERIAL.print(gait_x_offset_front); DEBUG_SERIAL.println(" cm");
  DEBUG_SERIAL.print("Gait X offset (back):  "); DEBUG_SERIAL.print(gait_x_offset_back); DEBUG_SERIAL.println(" cm");
  DEBUG_SERIAL.print("Delta phi_a:           "); DEBUG_SERIAL.print(delta_phi_a); DEBUG_SERIAL.println(" deg");
  
  // VERIFY IK AT TRAJECTORY EXTREMES FOR EACH LEG
  DEBUG_SERIAL.println("\nIK Verification Per Leg");
  
  for (int leg = 0; leg < 4; leg++) {
    DEBUG_SERIAL.print("\n");
    DEBUG_SERIAL.print(leg_names[leg]);
    DEBUG_SERIAL.println(" trajectory check:");
    
    // Check at key phases
    float test_phases[] = {0.0, duty_cycle/2, duty_cycle - 0.01, duty_cycle + 0.01, 0.8, 0.99};
    int num_tests = 6;
    
    bool leg_ok = true;
    for (int i = 0; i < num_tests; i++) {
      FootPosition foot = getFootPositionForLeg(test_phases[i], leg);
      LegAngles angles = inverseKinematics(foot.x, foot.y);
      
      if (!angles.valid) {
        DEBUG_SERIAL.print("  FAIL at phase ");
        DEBUG_SERIAL.print(test_phases[i], 2);
        DEBUG_SERIAL.print(": unreachable (");
        DEBUG_SERIAL.print(foot.x, 2);
        DEBUG_SERIAL.print(", ");
        DEBUG_SERIAL.print(foot.y, 2);
        DEBUG_SERIAL.println(")");
        leg_ok = false;
      }
    }
    
    if (leg_ok) {
      // Show trajectory bounds
      FootPosition front = getFootPositionForLeg(0.0, leg);
      FootPosition back = getFootPositionForLeg(duty_cycle, leg);
      FootPosition peak = getFootPositionForLeg(0.8, leg);
      
      DEBUG_SERIAL.print("  OK - X range: [");
      DEBUG_SERIAL.print(back.x, 2);
      DEBUG_SERIAL.print(" to ");
      DEBUG_SERIAL.print(front.x, 2);
      DEBUG_SERIAL.print("], Y range: [");
      DEBUG_SERIAL.print(peak.y, 2);
      DEBUG_SERIAL.print(" to ");
      DEBUG_SERIAL.print(front.y, 2);
      DEBUG_SERIAL.println("]");
    }
  }
  
  // INITIALIZE MOTORS
  DEBUG_SERIAL.println("\nMotor Initialization");
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  
  for (int i = 0; i < TOTAL_MOTORS; i++) {
    if (dxl.ping(motor_IDs[i])) {
      DEBUG_SERIAL.print(motor_names[i]);
      DEBUG_SERIAL.println(" - OK");
      dxl.torqueOff(motor_IDs[i]);
      dxl.setOperatingMode(motor_IDs[i], OP_POSITION);
      dxl.torqueOn(motor_IDs[i]);
    } else {
      DEBUG_SERIAL.print(motor_names[i]);
      DEBUG_SERIAL.println(" - NOT FOUND");
    }
  }
  
  // Move to initial stance position
  DEBUG_SERIAL.println("\nMoving to initial position");
  for (int leg = 0; leg < 4; leg++) {
    float phase_offset = getLegPhaseOffset(leg);
    FootPosition init_pos = getFootPositionForLeg(phase_offset, leg);
    LegAngles init_angles = inverseKinematics(init_pos.x, init_pos.y);
    
    if (init_angles.valid) {
      setLegAngles(leg, init_angles.theta1, init_angles.theta2);
    }
  }
  
  DEBUG_SERIAL.println("Ready");
  delay(2000);
  DEBUG_SERIAL.println("\nStarting gait\n");
}

void loop() {
  static unsigned long start_time = millis();
  static int last_cycle = -1;
  
  float current_time = (millis() - start_time) / 1000.0;
  int current_cycle = (int)(current_time / gait_period);
  
  // End of cycle detection for deadzone report
  if (current_cycle > last_cycle) {
    if (last_cycle >= 0) {
      printDeadzoneReport();
    }
    last_cycle = current_cycle;
    
    DEBUG_SERIAL.print("Cycle ");
    DEBUG_SERIAL.print(current_cycle);
    DEBUG_SERIAL.println(" ");
  }
  
  // Process each leg
  for (int leg = 0; leg < 4; leg++) {
    // Get this leg's phase in the gait cycle
    float phase_offset = getLegPhaseOffset(leg);
    float leg_phase = fmod(current_time / gait_period + phase_offset, 1.0);
    
    // Get target foot position (with all climbing adjustments)
    FootPosition foot = getFootPositionForLeg(leg_phase, leg);
    
    // Compute joint angles
    LegAngles angles = inverseKinematics(foot.x, foot.y);
    
    if (angles.valid) {
      setLegAngles(leg, angles.theta1, angles.theta2);
    }
  }
  
  // Debug output every second
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 1000) {
    last_debug = millis();
    
    float phase = fmod(current_time / gait_period, 1.0);
    
    DEBUG_SERIAL.print("t=");
    DEBUG_SERIAL.print(current_time, 1);
    DEBUG_SERIAL.print("s phase=");
    DEBUG_SERIAL.print(phase, 2);
    
    // Show one leg's trajectory (LF)
    FootPosition foot = getFootPositionForLeg(phase, 0);
    LegAngles angles = inverseKinematics(foot.x, foot.y);
    
    DEBUG_SERIAL.print(" LF:(");
    DEBUG_SERIAL.print(foot.x, 1);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(foot.y, 1);
    DEBUG_SERIAL.print(") θ=(");
    DEBUG_SERIAL.print(angles.theta1 * RAD_TO_DEG, 0);
    DEBUG_SERIAL.print(",");
    DEBUG_SERIAL.print(angles.theta2 * RAD_TO_DEG, 0);
    DEBUG_SERIAL.println(")");
  }
  
  delay(20);  // Update rate in Hz
}
