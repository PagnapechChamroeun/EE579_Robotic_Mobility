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

                                                                              // 50 25
int Leg_zeroing_offset[]={175, 150,/*LF*/   120, 210,/*RF*/   120, 45,/*LR*/   45, 25 /*RR*/}; // [1,2,3,4,5,6,7,8] 




float leg_ang[]={0,0,0,0,0,0,0,0}; // [1,2,3,4,5,6,7,8] 
 
float clock_period=4; //in seconds, time to complete 1 rotation
float pi = 3.14;

float LL_1[] = {6.5, 7.5}; //Leg 1 Links in CM 9 hole = 7cm


//configure your timing parameters
void clock_init(){   
  //Insert your calculated time_slow_start and time_slow_end here. You do not have to use degree_slow_start and degree_slow_end, but they may be helpful.
  // at the beginning of each stride period, the desired angle, \phi, should be 0 degree (leg should point vertically upward if you have a leg installed). 
  // we suggest that you make sure that the deadzone (300deg to 360deg) is fully within the fast phase (i.e., 0<degree_slow_start<degree_slow_end<300). Also make sure 0<time_slow_start<time_slow_end<clock_period
  // notice degree_slow_start, degree_slow_end here are in deg  
 
  return;
}

// void set_all_legs_to_neutral() {
//   for (int i = 0; i < total_legs; i++) {
//     float neutral = Leg_zeroing_offset[i]; // "zero leg offset angle" in your IK frame0 

//     // apply mirrored direction the same way as in loop() 
//     if(Directions[i] == 1) {
//       neutral = 360.0f - neutral; 
//     }

//     // set position 
//     dxl.setGoalPosition(IDs[i], neutral, UNIT_DEGREE); 
//   }
// }

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
      // hip joint (even index: 0,2,4,6) 
      if(leg_index % 2 == 0){angle = get_angle(x, y_coord[0], 0);}
      // knee joint (odd index: 1,3,5,7)
      else {angle = get_angle(x, y_coord[0], 1);}
    }
    // in swing phase 
    else{
      float x = (period-time_s) / time_c * (x_coord[0] - x_coord[1]);
      float y = get_circle(x);
      // hip joint (even index: 0,2,4,6) 
      if(leg_index % 2 == 0){angle = get_angle(x, y, 0);}
      // knee joint (odd index: 1,3,5,7)
      else {angle = get_angle(x, y, 1);}
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

    // float neutral = compute_neutral_angle(i); 
    // dxl.setGoalPosition(IDs[i], neutral, UNIT_DEGREE); 
  }

  // ----- move legs to neutral pose -----
//  set_all_legs_to_neutral(); 
  // delay(4000); // wait for 2 sec 
  
  // ----- samples imu data ----- 
  // calibrateIMU(); // capture level reference posture // print initial value

  // ----- start gait timing ----- 
  start = millis();
  clock_init();
  translate_gait_deg(); 
}


long last_time=0;
int time_step=100;
bool in_dead_zone[]={0,0,0,0}; //0=not, 1=in
bool imu_calibrated = false; 
unsigned long imu_calib_time_ms = 5000; // wait 5 s after start 

void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value
  
  // calibration 
  if(!imu_calibrated) {
    if (millis() - start > imu_calib_time_ms) {
      DEBUG_SERIAL.println("Auto IMU calibration...");
      calibrateIMU(); 
      imu_calibrated = true; 
      DEBUG_SERIAL.println("IMU calibration done."); 
    }
  }
  
  long elapsed = millis() - start;
  
  
  if (elapsed - last_time > time_step){
    last_time=elapsed;

    //----- Read current IMU orientation ----- 
    readIMU(imu_roll, imu_pitch, imu_yaw);

    float roll_err = imu_roll - imu_roll0; 
    float pitch_err = imu_pitch - imu_pitch0; 

    // See errors over serial 
    DEBUG_SERIAL.print("roll_err: "); 
    DEBUG_SERIAL.print(roll_err); 
    DEBUG_SERIAL.print(" pitch_err: "); 
    DEBUG_SERIAL.println(pitch_err); 

    //----- Gait -> IK -> IMU correction -> send to motors -----
    for (int i = 0; i < total_legs; i++) {
      
      float desired_pos = get_desired_angle(i,elapsed);

      // add IMU-based stabilizing correction 
      // if (imu_calibrated) {
      //   desired_pos += imu_correction_deg(i, roll_err, pitch_err); 

      // }

      // handle mirrored installation 
      if (Directions[i]==1) {
        desired_pos=360.0 - desired_pos;
      }

      dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
    }
  
//    print_position(elapsed, leg_ang[0], leg_ang[1]);
  }

}
