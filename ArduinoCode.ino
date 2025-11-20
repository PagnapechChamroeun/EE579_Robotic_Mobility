

#include <DynamixelShield.h>
#include <stdlib.h>
#include <math.h>
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif
const float DXL_PROTOCOL_VERSION = 2.0;
DynamixelShield dxl;

// ==============================================================

int total_legs=8; //number of legs

uint8_t IDs[]={1,2,3,4,5,6,7,8}; // Leg ID corresponding to [LF,RF,LR,RR] legs.  ODD = Hip, EVEN = Knee
uint8_t Directions[]={1,0,0,1,1,0,0,1};
float gait[]={0.5,0.5,0}; // (\phi_1, \phi_2, \phi_3) = [LF-RF,LF-LR,LF-RR]; 
float gait_deg[]={0,0,0,0};  //gait in deg; [RF,LF,LR,RR]; the value for RF will always be 0, the value for LF= 360*\phi_1, LR=360*\phi_2, RR=360*\phi_3; calculated in translate_gait_deg();
int Leg_zeroing_offset[]={175, 160,   120, 210,   105, 135,   45, 190}; // [1,2,3,4,5,6,7,8] 
float leg_ang[]={0,0,0,0,0,0,0,0};
 
float clock_period=3.5; // MODIFIED: Increased from 2 to 3.5 seconds for stability and motor torque
float pi = 3.14;

float L1 = 6.5;
float L2 = 9.0;

// ==============================================================

float stride_length = 12.0;    // Total horizontal travel (cm) - derived from your x_coord
float stance_depth = 7.0;      // Default foot depth below hip (cm)
float step_height = 3.0;       // Maximum lift during swing (cm)
float body_pitch_offset = 5.0; // Body pitch for inclines (degrees)
float force_angle_offset = 10.0; // Additional "digging in" angle (degrees)

float dc = 0.65;  // Duty cycle (65% stance, 35% swing)
float time_s = dc * clock_period;     // Stance duration
float time_c = (1-dc)*clock_period;   // Swing duration

bool is_front_leg[] = {true, true, true, true, false, false, false, false};

// ==============================================================

float enhanced_inverse_kinematics(float x, float y, bool is_hip, bool is_back_leg) {
    float angle;
    
    // MODIFIED: Account for back legs being mounted reversed
    if (is_back_leg) {
        x = -x;  // Reverse x for back legs
    }
    
    float distance = sqrt(x*x + y*y);
    
    // Check reachability
    if (distance > (L1 + L2) || distance < abs(L1 - L2)) {
        // Return neutral position if unreachable
        return is_hip ? 90 : 90;
    }
    
    if (is_hip) {
        // Hip angle calculation (theta1)
        float cos_theta2 = (L1*L1 + L2*L2 - distance*distance) / (2*L1*L2);
        float theta2 = acos(constrain(cos_theta2, -1.0, 1.0));
        
        float alpha = atan2(x, y);  // Angle from vertical
        float beta = acos((L1*L1 + distance*distance - L2*L2) / (2*L1*distance));
        
        angle = (alpha + beta) * 180 / pi;
        
        // Adjust for back legs
        if (is_back_leg) {
            angle = 180 - angle;
        }
    } else {
        // Knee angle calculation (theta2)
        float cos_theta2 = (L1*L1 + L2*L2 - distance*distance) / (2*L1*L2);
        angle = acos(constrain(cos_theta2, -1.0, 1.0)) * 180 / pi;
    }
    
    return angle;
}

void get_stance_position(float phase, bool is_front, float& x_out, float& y_out) {
    // MODIFIED: Linear motion but with different strategies for front/back
    if (is_front) {
        // Front legs: Pull motion (start forward, pull back)
        x_out = (stride_length / 2.0) * (1.0 - 2.0 * phase);
    } else {
        // Back legs: Push motion (start back, push forward)
        // Since back legs are reversed, this creates pushing force
        x_out = (stride_length / 2.0) * (2.0 * phase - 1.0);
    }
    
    // Keep constant depth for stability
    y_out = stance_depth;
    
    // NEW: Apply force angle for "digging in" effect on slopes
    // Slightly adjust y based on x position to create angled force vector
    float slope_compensation = x_out * tan(force_angle_offset * pi / 180);
    y_out += slope_compensation * 0.1;  // Small adjustment
}

void get_swing_position(float phase, bool is_front, float& x_out, float& y_out) {
    // MODIFIED: Improved swing trajectory
    if (is_front) {
        // Front legs swing forward (from back to front)
        x_out = (stride_length / 2.0) * (2.0 * phase - 1.0);
    } else {
        // Back legs swing backward (from front to back in their frame)
        x_out = (stride_length / 2.0) * (1.0 - 2.0 * phase);
    }
    
    // NEW: Better parabolic trajectory with peak at 30% of swing
    float swing_peak = 0.3;  // Peak height occurs at 30% of swing phase
    float height_factor;
    if (phase <= swing_peak) {
        height_factor = phase / swing_peak;
    } else {
        height_factor = (1.0 - phase) / (1.0 - swing_peak);
    }
    
    // Smooth parabolic lift
    y_out = stance_depth - step_height * 4.0 * height_factor * (1.0 - height_factor);
}

float get_desired_angle(int motor_id, long elapsed) {
    // Determine which leg and whether hip or knee
    int leg_num = motor_id / 2;  // 0=LF, 1=RF, 2=LR, 3=RR
    bool is_hip = (motor_id % 2 == 0);
    bool is_front = (leg_num < 2);
    bool is_back = !is_front;
    
    // Calculate phase for this specific leg
    float period = fmod(elapsed / 1000.0, clock_period);
    
    // Apply phase offsets for trotting gait
    switch (leg_num) {
        case 0: // LF - phase 0
            break;
        case 1: // RF - phase π (opposite of LF)
            period = fmod(period + (clock_period * 0.5), clock_period);
            break;
        case 2: // LR - phase π (diagonal with LF)
            period = fmod(period + (clock_period * 0.5), clock_period);
            break;
        case 3: // RR - phase 0 (diagonal with RF)
            break;
    }
    
    // Determine if in stance or swing phase
    float x, y;
    if (period < time_s) {
        // Stance phase
        float stance_phase = period / time_s;
        get_stance_position(stance_phase, is_front, x, y);
    } else {
        // Swing phase
        float swing_phase = (period - time_s) / time_c;
        get_swing_position(swing_phase, is_front, x, y);
    }
    
    // Calculate joint angle using enhanced IK
    float angle = enhanced_inverse_kinematics(x, y, is_hip, is_back);
    
    // Store for debugging
    leg_ang[motor_id] = angle;
    
    // Apply body pitch compensation for climbing
    if (is_front) {
        angle -= body_pitch_offset;  // Extend front legs
    } else {
        angle += body_pitch_offset;  // Compress rear legs
    }
    
    // Apply motor zero offset
    angle += Leg_zeroing_offset[motor_id];
    
    // Ensure angle is in valid range
    angle = fmod(angle, 360);
    if (angle < 0) angle += 360;
    
    return angle;
}

// =============================================================

long start;

void setup() {
    DEBUG_SERIAL.begin(115200);
    
    dxl.begin(1000000);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    
    // Initialize all motors
    for (int i=0; i<total_legs; i++){
        dxl.torqueOff(IDs[i]);
        dxl.setOperatingMode(IDs[i], OP_POSITION);
        dxl.torqueOn(IDs[i]);
        delay(100);
    }
    
    start = millis();
    
    // Print configuration
    DEBUG_SERIAL.println("Enhanced Climbing Gait Controller Started");
    DEBUG_SERIAL.println("Commands: STRIDE:val, HEIGHT:val, DEPTH:val, DUTY:val, PERIOD:val, PITCH:val");
}

long last_time=0;
int time_step=50;  // MODIFIED: Faster update rate (was 100ms, now 50ms)

void loop() {
    long elapsed = millis() - start;
    
    // Update motor positions at regular intervals
    if (elapsed - last_time > time_step) {
        last_time = elapsed;
        
        for (int i=0; i<total_legs; i++){
            float desired_pos = get_desired_angle(i, elapsed);
            
            // Apply direction reversal if needed
            if (Directions[i] == 1) {
                desired_pos = 360.0 - desired_pos;
            }
            
            dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        }
    }
}









/*


float get_angle(float x, float y, int leg){
  float angle;

  if(leg == 0){
      angle = (atan2(y,x) * 180 / pi) - (acos( ((pow(x,2) + pow(y,2) - pow(LL_1[1],2) + pow(LL_1[0],2)) / (2*LL_1[0]* sqrt(pow(x,2)+pow(y,2)))) ) * 180 / pi);
    }else{
      angle = 180 - acos( ((pow(LL_1[1],2) + pow(LL_1[0],2) - pow(x,2) - pow(y,2)) / (2*LL_1[0]*LL_1[1]))) * 180/pi ;  
    }
  
  return angle;
}

int x_coord[]={9,-3};         // MODIFIED: Changed from {8,0} to {9,-3} for 12cm stride length
int y_coord[]={7,7};          // MODIFIED: Changed from {10,10} to {7,7} for lower body height and better stability
float dc = 0.65;              // MODIFIED: Manually set to 0.65 (65% duty cycle) instead of calculating - critical for heavy robot
// REMOVED CALCULATION: float dc = (x_coord[1] - x_coord[0]) / (((x_coord[1] - x_coord[0]) + (((x_coord[1] - x_coord[0])/2) * pi)));
float time_s = dc * clock_period;
float time_c = (1-dc)*clock_period;
int rad = abs(x_coord[1] - x_coord[0]) / 2;
int x_cen =(x_coord[0] + x_coord[1]) / 2;
int y_cen = y_coord[0];

float body_pitch_offset = 5;  // ADDED: Body pitch in degrees for climbing inclines (5° for 10° incline, 8° for 20° incline)


// MODIFIED: Replaced circular trajectory with parabolic swing for better efficiency
float get_swing_trajectory(float x){
  // Parabolic arc with adjustable ground clearance
  float clearance = 3.0;  // 3cm ground clearance during swing phase
  float x_range = x_coord[0] - x_coord[1];
  float x_normalized = (x - x_coord[1]) / x_range;  // Normalize to 0-1 range
  float y = y_coord[0] - clearance * (4 * x_normalized * (1 - x_normalized));  // Parabola peaks at 0.5
  return y;
}

// DEPRECATED: Old circular trajectory function - kept for reference but no longer used
float get_circle(float x){
  float ans = -sqrt(pow(rad,2) - pow((x - x_cen),2)) + y_cen;
  return ans;
}


// compute desired motor angle at any time instance
float get_desired_angle(int leg, long elapsed){ 
    
    //FIXME
    float period = fmod(elapsed / 1000.0, clock_period);
    float angle;

//    if(leg != 0 && leg != 1){period = fmod(period + (clock_period * (1-gait[leg-1])), clock_period);}

    int gait_leg = leg+1;

    switch (gait_leg) {
      case 1:
      case 2: break;
      case 3:
      case 4:
        period = fmod(period + (clock_period * (1-gait[0])), clock_period);
        break;
      case 5:
      case 6:
        period = fmod(period + (clock_period * (1-gait[1])), clock_period);
        period = clock_period - period;
        break;
      case 7:
      case 8:
        period = fmod(period + (clock_period * (1-gait[2])), clock_period);
        period = clock_period - period;
        break;
      default:
        break;
    }

//    if(leg > 1){period = fmod(period + (clock_period * 0.5), clock_period);}

    if(period < time_s){
      float x = (time_s - period) / time_s * (x_coord[0] - x_coord[1]);
      if(leg % 2 == 0){angle = get_angle(x, y_coord[0], 0);}
      else {angle = get_angle(x, y_coord[0], 1);}
    }else{
      float x = (period-time_s) / time_c * (x_coord[0] - x_coord[1]);
      float y = get_swing_trajectory(x);  // MODIFIED: Changed from get_circle(x) to get_swing_trajectory(x)
      if(leg % 2 == 0){angle = get_angle(x, y, 0);}
      else {angle = get_angle(x, y, 1);}
    }

    leg_ang[leg] = angle;
    
    // ADDED: Apply body pitch offset for climbing inclines
    // Front legs extend more, rear legs compress to pitch body forward
    if(leg < 4) {  // Front legs (LF hip/knee, RF hip/knee - indices 0,1,2,3)
        angle = angle - body_pitch_offset;  // Extend forward
    } else {       // Rear legs (LR hip/knee, RR hip/knee - indices 4,5,6,7)
        angle = angle + body_pitch_offset;  // Compress
    }
    
    angle = angle + Leg_zeroing_offset[leg];

    angle = fmod(angle, 360);
    
    return angle; //in deg
}


// print desired motor position and associated elapsed time to debug serial
void print_position(long t, float pos1, float pos2){

  DEBUG_SERIAL.print(fmod(t/1000.0, clock_period));
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.print(pos1);
  DEBUG_SERIAL.print(",");
  DEBUG_SERIAL.println(pos2);
  return;
}


void translate_gait_deg(){ //calculate the value of the array 'gait_deg[]' 
  //notice the unit of 'gait_deg[]' is in deg and has 4 elements
  gait_deg[0]=0;  //RF
  gait_deg[1]=gait[0]*360; //LF
  gait_deg[2]=gait[1]*360; //LB
  gait_deg[3]=gait[2]*360; //RB
  
   return;
}

int dead_zone_speed_tuning=-10; //adjustment to tune deadzone speed, MAGIC Variable as we are using position control outside the deadzone and speed control inside deadzone so the speed might be different; this variable is manually selected from observation, and it's ok that it's not working well.  
//int different_direction_offset=-110; //adjustment to compensate position offset between 2 legs with different rotating direction. Another MAGIC variable that requires manual observation. 


////////////////////////////////////////////// The code below should require none or minimal changes /////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


long start;

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 1000000bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  
  // Turn off torque when configuring items in EEPROM area
  for (int i=0;i<=0;i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
  }

  start = millis();
  clock_init();
  translate_gait_deg(); 
}


long last_time=0;
int time_step=100;
bool in_dead_zone[]={0,0,0,0}; //0=not, 1=in


void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value
  long elapsed = millis() - start;
  
  
  if (elapsed-last_time>time_step){
    last_time=elapsed;
    for (int i=0;i<total_legs;i++){
      float desired_pos=get_desired_angle(i,elapsed);
      if (Directions[i]==1) desired_pos=360.0-desired_pos;
      dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
    }
  }
}

/*
Tuning Parameters for different inclines

// For flat ground testing:
float body_pitch_offset = 0;

// For 10° incline:
float body_pitch_offset = 5;

// For 20° incline:
float body_pitch_offset = 8;
int y_coord[]={6.5,6.5};  // May need to go even lower

// If motors still struggle:
float clock_period = 4.0;  // Slow down further
float dc = 0.70;           // Increase duty cycle further
*/

