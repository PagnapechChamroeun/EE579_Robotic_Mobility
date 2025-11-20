#include <DynamixelShield.h>
#include <stdlib.h>

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

#include "imu.h" 

int total_legs=8; //number of legs
/*
LEG ID corresponding to 
  - Left Front hip:   1 
  - Left Front knee:  2
  - Right Front hip:  3 
  - Right Front knee: 4 
  - Left Rear hip:    5 
  - Left Rear knee:   6
  - Right Rear hip:   7 
  - Right Rear knee:  8 
  ODD = Hip, Even = Knee
*/
uint8_t IDs[]={1,2,3,4,5,6,7,8};
uint8_t Directions[]={1,0,0,1,1,0,0,1};
/*
 (\phi_1, \phi_2, \phi_3) = [LF-RF,LF-LR,LF-RR]; 
 LF is a reference leg
*/
float gait[]={0.5,0.5,0}; // trotting
float gait_deg[]={0,0,0,0};  //gait in deg; [LF,RF,LR,RR]; the value for RF will always be 0, the value for LF= 360*\phi_1, LR=360*\phi_2, RR=360*\phi_3; calculated in translate_gait_deg();


/* Leg Zeroing Offset ID = [1,2,3,4,5,6,7,8] */
// int Leg_zeroing_offset[]={185, 330,/*LF*/   120, 210,/*RF*/   105, 135,/*LR*/   45, 190 /*RR*/}; 
// 175, 160,   120, 210,   105, 135,   45, 190

                          // 175, 150       // 120, 210       // 120, 45       // 45, 25                               
// int Leg_zeroing_offset[]={200, 150,/*LF*/   145, 210,/*RF*/   145, 45,/*LR*/   70, 25 /*RR*/}; // [1,2,3,4,5,6,7,8] 

int Leg_zeroing_offset[]={175, 140,/*LF*/   120, 200,/*RF*/   105, 45,/*LR*/   30, 25 /*RR*/}; // [1,2,3,4,5,6,7,8] 

int Leg_neutral_pose[]={180, 270,/*LF*/   125, 335,/*RF*/   140, 120,/*LR*/   75,100 /*RR*/}; 




float leg_ang[]={0,0,0,0,0,0,0,0}; // [1,2,3,4,5,6,7,8] 
 
float clock_period=3; //in seconds, time to complete 1 rotation
float pi = 3.14;

float LL_1[] = {6.5, 8.0}; //Leg 1 Links in CM 9 hole = 7cm

// LED blink param
int NUM_OF_BLINKS = 5;  


//configure your timing parameters
void clock_init(){   
  //Insert your calculated time_slow_start and time_slow_end here. You do not have to use degree_slow_start and degree_slow_end, but they may be helpful.
  // at the beginning of each stride period, the desired angle, \phi, should be 0 degree (leg should point vertically upward if you have a leg installed). 
  // we suggest that you make sure that the deadzone (300deg to 360deg) is fully within the fast phase (i.e., 0<degree_slow_start<degree_slow_end<300). Also make sure 0<time_slow_start<time_slow_end<clock_period
  // notice degree_slow_start, degree_slow_end here are in deg  
 
  return;
}

void set_all_legs_to_neutral() {
  for (int i = 0; i < total_legs; i++) {
    float neutral = Leg_neutral_pose[i]; // "neutral leg offset angle" in your IK frame0 

    // apply mirrored direction the same way as in loop() 
    if(Directions[i] == 1) {
      neutral = 360.0f - neutral; 
    }

    // set position 
    dxl.setGoalPosition(IDs[i], neutral, UNIT_DEGREE); 
  }
}

// number of blink 
void led_blink_indicator(int NUM_OF_BLINKS){
  // initialize digital pin LED_BUILTIN as an output 
  pinMode(LED_BUILTIN, OUTPUT); 

  for (int i=0; i < NUM_OF_BLINKS; i++) {
    // turn the LED on (HIGH is the voltage level)
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(1000); // wait for a second
    // turn the LED off by making the voltage LOW 
    digitalWrite(LED_BUILTIN, LOW); 
    delay(1000);
  }
}

// float compute_neutral_angle(int i) {
//   // Base neutral angle from calibration zero leg offset 
//   float neutral = Leg_zeroing_offset[i]; 

//   // flip direction 
//   if (Directions[i] == 1) {
//     neutral = 360.0 - neutral; 
//   }
//   return neutral; 
// }

float get_angle(float x, float y, int leg_index){
  /*
  Inverse Kinematics 
  leg_index == 0: hip joint angle 
    => use a two-link IK formula (law of cosines + atan2) to compute hip angle 
  leg_index == 1: knee joint angle 
    => use law of cosines formula to compute knee angle 
  
   return in degree
  */
  float angle;

  if(leg_index == 0){
      angle = (atan2(y,x) * 180 / pi) - (acos( ((pow(x,2) + pow(y,2) - pow(LL_1[1],2) + pow(LL_1[0],2)) / (2*LL_1[0]* sqrt(pow(x,2)+pow(y,2)))) ) * 180 / pi);
    }else{
      angle = 180 - acos( ((pow(LL_1[1],2) + pow(LL_1[0],2) - pow(x,2) - pow(y,2)) / (2*LL_1[0]*LL_1[1]))) * 180/pi ;  
    }
  
  return angle;
}

/*
foot movement
x_coord[] and y_coord[]: start and end points of the foot in the stride cycle 
stance phase is a line along y = 10 
swing is a semi-circle from (0,10) to (8,10)  
*/
int x_coord[]={8,0};         
int y_coord[]={10,10};      

/* duty cycle of stance vs swing */
float dc = (x_coord[1] - x_coord[0]) / (((x_coord[1] - x_coord[0]) + (((x_coord[1] - x_coord[0])/2) * pi))); //0.3889845296
float time_s = dc * clock_period;   // stance phase (foot on gnd)
float time_c = (1-dc)*clock_period; // swing phase (foot moving thru the air) 

/* circular arc used during swing */
int rad = abs(x_coord[1] - x_coord[0]) / 2;
/* center coordinate */
int x_cen =(x_coord[0] + x_coord[1]) / 2;
int y_cen = y_coord[0];

/* given x position, calculate y position
   (-) sqrt: indicates that it has a forward position of +x 
   curved swing trajectory (lifting and lowering the foot) 
*/
float get_circle(float x){
  float ans = -sqrt(pow(rad,2) - pow((x - x_cen),2)) + y_cen;
  return ans;
}


/* Elliptical swing parameters */
// Half step length in x (foot moves from x=0 to x=8)
const float SWING_A = 4.0f; 

// Vertical height (in "y" units) for flat and slope 
const float SWING_B_FLAT = 2.0f;  // lower, energy-saving on flat 
const float SWING_B_SLOPE = 3.5f; // higher clearance on slope 

// Extra lift offset for slope (shifts whole swing arc up)
const float SWING_Y_OFFSET_SLOPE = -0.7f;   // negative = more "up" (smaller y)

// Global flag (inside loop based on IMU pitch)
bool slope_mode = false; 

// compute elliptical swing foot position for given phase in [0,1]
// phase = 0 => start of swing at x = 0 
// phase = 1 => end of swing at x = 8 
void get_swing_xy(float phase, bool useSlope, float& x, float& y) {
  // elipse center 
  float x_c = (x_coord[0] + x_coord[1]) * 0.5f; // = 4 
  float y_c = y_coord[0];                       // = 10 
  
  // horizontal half-length 
  float a = SWING_A; 

  // vertical half-height 
  float b = useSlope ? SWING_B_SLOPE : SWING_B_FLAT; 
  float y_offset = useSlope ? SWING_Y_OFFSET_SLOPE : 0.0f; 

  // theta goes from pi to 2*pi so x runs 0 -> 8, y makes a smooth arch
  float theta = pi + pi * phase; // pi ... 2*pi 

  // Ellipse param 
  // x = x_c + a cos(theta), y = (y_c + offset) + b sin(theta) 
  x = x_c + a * cos(theta); 
  y = (y_c + y_offset) + b * sin(theta); 

  // Note: in this coordinate system, smaller y means foot is higher

}


// compute desired motor angle at any time instance
float get_desired_angle(int leg_index, long elapsed){ 

    float period = fmod(elapsed / 1000.0, clock_period);
    float angle;

//    if(leg != 0 && leg != 1){period = fmod(period + (clock_period * (1-gait[leg-1])), clock_period);}

    int gait_leg = leg_index +1;

    switch (gait_leg) {
      // Left Front (hip/knee) (reference leg)
      case 1:
      case 2: break;
      // Right Front (hip/knee)
      case 3:
      case 4:
        period = fmod(period + (clock_period * (1-gait[0])), clock_period);
        break;
      // Left Rear (hip/knee)
      case 5:
      case 6:
        period = fmod(period + (clock_period * (1-gait[1])), clock_period);
        period = clock_period - period;
        break;
      // Right Rear (hip/knee)
      case 7:
      case 8:
        period = fmod(period + (clock_period * (1-gait[2])), clock_period);
        period = clock_period - period;
        break;
      default:
        break;
    }

//    if(leg > 1){period = fmod(period + (clock_period * 0.5), clock_period);}

    // in stance phase 
    if(period < time_s){
      float x = (time_s - period) / time_s * (x_coord[0] - x_coord[1]);
      float y = y_coord[0];  // constant stance height 
      // hip joint (even index: 0,2,4,6) 
      if(leg_index % 2 == 0){
        angle = get_angle(x, y, 0);
      }
      // knee joint (odd index: 1,3,5,7)
      else {
        angle = get_angle(x, y, 1);
      }
    }
    // // in swing phase (semi-circle)
    // else{
    //   float x = (period-time_s) / time_c * (x_coord[0] - x_coord[1]);
    //   float y = get_circle(x);
    //   // hip joint (even index: 0,2,4,6) 
    //   if(leg_index % 2 == 0){angle = get_angle(x, y, 0);}
    //   // knee joint (odd index: 1,3,5,7)
    //   else {angle = get_angle(x, y, 1);}
    // }

    // in swing phase (elliptical)
    else {
      // normalized swing phase in [0, 1] 
      float swing_phase = (period - time_s) / time_c;

      float x, y;
      // Use higher ellipse when on slope, lower ellipse on flat 
      get_swing_xy(swing_phase, slope_mode, x, y); 

      if (leg_index % 2 == 0) { // hip
        angle = get_angle(x, y, 0);
      } else {                  // knee 
        angle = get_angle(x, y, 1); 
      }

    }

    // save the raw angle after calculation 
    leg_ang[leg_index] = angle;
    // raw angle + its original offset
    angle = angle + Leg_zeroing_offset[leg_index];
    // mod 360 
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
  gait_deg[0]=0;  //LF
  gait_deg[1]=gait[0]*360; //RF
  gait_deg[2]=gait[1]*360; //LR
  gait_deg[3]=gait[2]*360; //RR
  
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
  Serial.begin(115200); 

  //----- initialize IMU ----- 
  if (!bno.begin()) {
    DEBUG_SERIAL.println("No BNO055 detected ... Check wiring / I2C address");
    while(1); 
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  imu_ok = true; 

  // ----- initialize DXL -----  
  // Set Port baudrate to 1000000bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(1000000);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information

  // Turn off torque when configuring items in EEPROM area
  for (int i = 0; i < total_legs; i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
  }

  // ----- move legs to neutral pose -----
  set_all_legs_to_neutral(); 

  // set LED blink to indicate that it is about to start calibrating when the LED stops blinking
  led_blink_indicator(NUM_OF_BLINKS);  
  
  // ----- sampling imu data ----- 
  calibrateIMU(); // capture level reference posture // print initial value

  // ----- start gait timing ----- 
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
  
  
  if (elapsed - last_time > time_step){
    last_time=elapsed;

    //----- Read current IMU orientation ----- 
    readIMU(imu_roll, imu_pitch, imu_yaw);

    float roll_err = imu_roll - imu_roll0;
    float pitch_err = imu_pitch - imu_pitch0; 

    // simple slope detector 
    // if the torso pitch deviates beyond some threshold, treat as "slope mode".
    const float SLOPE_PITCH_THRESH = 5.0f; // deg, modify the threshold as desired
    float pitch_now = imu_pitch - imu_pitch0; 
    if (fabs(pitch_now) > SLOPE_PITCH_THRESH) {
      slope_mode = true;    // slope (uphill or downhill)
    } else {
      slope_mode = false;   // flat 
    }

    // See errors over serial 
    DEBUG_SERIAL.print("roll_err: "); 
    DEBUG_SERIAL.print(roll_err); 
    DEBUG_SERIAL.print(" pitch_err: "); 
    DEBUG_SERIAL.println(pitch_err); 

    //----- Gait -> IK -> IMU correction -> send to motors -----
    for (int i = 0; i < total_legs; i++) {
      
      float desired_pos = get_desired_angle(i,elapsed);

      // add IMU-based stabilizing correction 
      float corr = imu_correction_deg(i, roll_err, pitch_err); 
      desired_pos += corr;

      // handle mirrored installation 
      if (Directions[i]==1) {
        desired_pos=360.0 - desired_pos;
      }

      dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
    }
  
//    print_position(elapsed, leg_ang[0], leg_ang[1]);
  }

}