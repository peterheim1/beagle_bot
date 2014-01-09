/*This is version 2 off omni_bot controll 
it listens to cmd_vel and 
using arduino PID and the drive motors
pose to be done in python
front right steering is very flacky
drive motors pulse
version 2_2 uses a 2 stage PID controller

*/

#include <math.h>
#include <Servo.h>
#include <PID_v1.h>

#include <Messenger.h>
#include "Arduino.h"
//#include <RobotParams.h>
#include <TimeInfo.h>
#include <BatteryMonitor.h>



//Front Left Drive motor
#define F_L_drive_InA1            29                       // INA motor pin
#define F_L_drive_InB1            25                       // INB motor pin
#define F_L_drive_PWM1            2                        // PWM motor pin
#define encodPinA1                21                       // encoder A pin
#define encodPinB1                34                       // encoder B pin
#define left_enable               23                       // enable motor controller
//Front Right Drive motor
#define F_R_drive_InA2            31                       // INA motor pin
#define F_R_drive_InB2            33                       // INB motor pin
#define F_R_drive_PWM2            3                        // PWM motor pin
#define encodPinA2                20                       // encoder A pin
#define encodPinB2                35                      // encoder B pin

//rear left steering pod
#define R_L_InA1                  44                       // INA motor pin
#define R_L_InB1                  42                       // INB motor pin
#define R_L_PWM1                  5                        // PWM motor pin
#define R_L_encodPinA1            18                       // encoder A pin
#define R_L_encodPinB1            36                       // encoder b pin
//rear Right steering pod
#define R_R_InA2                  43                       // INA motor pin
#define R_R_InB2                  41                       // INB motor pin
#define R_R_PWM2                  7                        // PWM motor pin
#define R_R_encodPinA2            19                       // encoder A pin
#define R_R_encodPinB2            37                       // encoder B pin

#define LOOPTIME                  100                     // pose loop time
//#define NUMREADINGS               10                      // samples for Amp average
//#define PI 3.14159265
#define TwoPI 6.28318531


int steer_F_R ;
int steer_F_L ;
int steer_R_R ;
int steer_R_L ;

//Calibration K factors for steering
int F_L_K = 0;
int F_R_K = 0;
int R_L_K = 0;
int R_R_K = 0;

//float R_Cal = 1.055;


//float Drive_angle =0.0;
//RobotParams _RobotParams = RobotParams();
TimeInfo _TimeInfo = TimeInfo();
// added for battery
#define c_ScaledBatteryVInPin 4 // analog input pin for the battery voltage divider
#define c_VInToVBatteryRatio 3.35
BatteryMonitor _BatteryMonitor(c_ScaledBatteryVInPin, c_VInToVBatteryRatio);
// added for battery
#define c_UpdateInterval 100  // update interval in milli seconds
//#define LOOPTIME         60                     // PID loop time
//hb25

Servo leftRotation;
Servo rightRotation;
Servo Front_leftRotation;
Servo Front_rightRotation;
//int minPulse = 600;   // minimum servo position, us (microseconds)
//int maxPulse = 1070; // maximum servo position, us

//int Front_Left_Pod ; 
double Front_Left_Steer_Enc = 0;  // changed from int to double for pid
//int Front_Right_Pod ;
double Front_Right_Steer_Enc = 0;  
//int Rear_Left_Pod ;
double Rear_Left_Steer_Enc = 0;  
//int Rear_Right_Pod ;
double Rear_Right_Steer_Enc = 0;  
//int readings[NUMREADINGS];
unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing

/*working variables
unsigned long R_R_lastTime;
double R_R_errSum, R_R_lastErr;
double R_R_ITerm, R_R_lastInput;
unsigned long R_L_lastTime;
double R_L_errSum, R_L_lastErr;
double R_L_ITerm, R_L_lastInput;
unsigned long F_R_lastTime;
double F_R_errSum, F_R_lastErr;
double F_R_ITerm, F_R_lastInput;
unsigned long F_L_lastTime;
double F_L_errSum, F_L_lastErr;
double F_L_ITerm, F_L_lastInput;
//double Front_Kp, ki, Front_Kd;
int SampleTime = 100; //5 hertz

double R_R_kp = 1.8;
double R_R_ki = .03;
double R_R_kd = .01;
double R_R_outMin = -90 ;
double R_R_outMax = 90;*/

/*
double R_L_kp = 1.8;
double R_L_ki = .02;
double R_L_kd = .01;
double R_L_outMin = -90 ;
double R_L_outMax = 90;*/


//Front right
/*
double F_R_kp = 1.6;
double F_R_ki = .03;
double F_R_kd = .01;
double F_R_outMin = -90 ;
double F_R_outMax = 90;*/


//front left
/*
double F_L_kp = 1.6;
double F_L_ki = .03;
double F_L_kd = 01;
double F_L_outMin = -90 ;
double F_L_outMax = 90;
*/

//Steering PID output
double F_L_Output =0;
double F_R_Output = 0;
double R_L_Output =0;
double R_R_Output =0;


//Drive Wheel angle and speed
double Wheel_A_F_L =0;
double Wheel_A_F_R =0;
double Wheel_A_R_R =0;
double Wheel_A_R_L =0;
double Wheel_S_F_L = 0;
double Wheel_S_F_R = 0;
double Wheel_S_R_L = 0;
double Wheel_S_R_R = 0;

double speed_act_F_L = 0;                              // speed (actual value)
double speed_act_F_R = 0;
double speed_act_R_L = 0;   
double speed_act_R_R = 0;                              // speed (actual value) 
                              
double PWM_val_F_L = 0;    // speed (actual value)
double PWM_val_F_R = 0;    // speed (actual value)                            
double PWM_val_R_L = 0;    // speed (actual val
double PWM_val_R_R = 0;    // speed (actual value)                          
//int voltage = 0;                                // in mV
//int current = 0;                                // in mA
volatile long count_L = 0;                        // rev counter
volatile long count_R = 0;                       // rev counter
volatile long count_R_L =0;
volatile long count_R_R =0;
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
/*
float Front_Kp = 1;                                // PID proportional control Gain.4
float Front_Kd = .5; 
float Front_R_Kp = 100;                                // PID proportional control Gain.4
float Front_R_Kd = 0; 
float Rear_Kp = 90;                                // PID proportional control Gain.4
float Rear_Kd = 0;
*/

// PID Derivitave control gain   1
//this is a place holder for user input
//float userInput[3]= {0, 0, 0}; // we will change this to x y z later
//float  test;
int Steer_enc[] = { 0, 1, 2, 3,};
// variables for distance calc@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
float X_dist= 0;
float Y_dist;
double drive_F_L = 0;
double drive_F_R = 0;
double drive_R_L = 0;
double drive_R_R = 0;
double DriveDist_Front = 0;
double DriveDist_Rear = 0;
float BaseRotation = 0;
float Heading = 0;
float Hed = 0;
float Delta_X = 0;
float Delta_Y = 0;
double Prev_count_F_L = 0;
double Prev_count_F_R = 0;
double Prev_count_R_L = 0;
double Prev_count_R_R = 0;
float DriveDist = 0;
//new variables for adaptive PID
double Front_left_gap = 0;
double Front_right_gap = 0;
double Rear_left_gap = 0;
double Rear_right_gap = 0;

/*
float Heading = 0;
float driveL = 0;
float driveR = 0;
float Prev_count_L = 0;
float Prev_count_R = 0;
*/
//long R_R_countInit =0;
//long R_L_countInit =0;
//float DriveAngle =0;
//float PreviousHeading =0;
//int RPM;
//int LeftRPM = 0;
//int RightRPM = 0;
//float WheelVel_L;
//float WheelVel_R;
//float Hed =0;




// STEERING CAL

//double STR = 0;//userInput[1];
//double FWD = 1;//userInput[0];
//double RCW = 0;//userInput[2];
//double R = 48.4148737476408;
int RevD = 1;// to reverse direction without altering wheel direction

// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger _Messenger = Messenger();

bool _IsInitialized = false;
//arduino pid*******************************************************************************************************************PID  PID
PID Front_right(&speed_act_F_R, &PWM_val_F_R, &Wheel_S_F_R, 5,9,0, DIRECT);
PID Front_left(&speed_act_F_L, &PWM_val_F_L, &Wheel_S_F_L, 5,9,0, DIRECT);
PID Rear_left(&speed_act_R_L, &PWM_val_R_L, &Wheel_S_R_L, 5,9,0, DIRECT);
PID Rear_right(&speed_act_R_R, &PWM_val_R_R, &Wheel_S_R_R, 4,8,0, DIRECT);
//new  wheel steering pid loop
PID Steer_F_R(&Front_Right_Steer_Enc, &F_R_Output, &Wheel_A_F_R, 5,1,0, DIRECT);
PID Steer_F_L(&Front_Left_Steer_Enc, &F_L_Output, &Wheel_A_F_L, 5,1,0, DIRECT);
//rear pods
PID Steer_R_R(&Rear_Right_Steer_Enc, &R_R_Output, &Wheel_A_R_R, 4,1,0, DIRECT);
PID Steer_R_L(&Rear_Left_Steer_Enc, &R_L_Output, &Wheel_A_R_L, 4,1,0, DIRECT);

/*
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

                 Set up
                 
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

*/
void setup()
{
  //initialize the variables we're linked to
  Serial.begin(115200);
  _Messenger.attach(OnMssageCompleted);
  
  //TCCR3B =0x00;
  //TCCR3B =0x01;
  //TCCR3C =0x00;
  //TCCR3C =0x01;
  TCCR3B = (TCCR3B & 0xF8) | 0x01 ;
  TCCR3C = (TCCR3C & 0xF8) | 0x01 ;// front right
  
  //TCCR3A =0x00;
  //TCCR3A =0x01;
  //TCCR4B =0x00;
  //TCCR4B =0x01;
  TCCR3A = (TCCR3A & 0xF8) | 0x01 ; //rear left
  TCCR4B = (TCCR4B & 0xF8) | 0x01 ;// rear right
  pinMode(F_L_drive_InA1, OUTPUT);
  pinMode(F_L_drive_InB1, OUTPUT);
  pinMode(F_L_drive_PWM1, OUTPUT);
  pinMode(left_enable, OUTPUT);
  pinMode(encodPinA1, INPUT);
  pinMode(encodPinB1, INPUT);
  digitalWrite(encodPinA1, HIGH);                      // turn on pullup resistor
  digitalWrite(encodPinB1, HIGH);
  pinMode(F_R_drive_InA2, OUTPUT);
  pinMode(F_R_drive_InB2, OUTPUT);
  pinMode(F_R_drive_PWM2, OUTPUT);
  pinMode(encodPinA2, INPUT);
  pinMode(encodPinB2, INPUT);
  digitalWrite(encodPinA2, HIGH);                      // turn on pullup resistor
  digitalWrite(encodPinB2, HIGH);
   //init rear right
  pinMode(R_R_InA2, OUTPUT);
  pinMode(R_R_InB2, OUTPUT);
  pinMode(R_R_PWM2, OUTPUT);
  pinMode(R_R_encodPinA2, INPUT);
  pinMode(R_R_encodPinB2, INPUT);
  digitalWrite(R_R_encodPinA2, HIGH);                      // turn on pullup resistor
  digitalWrite(R_R_encodPinB2, HIGH);
  //init rear left
  pinMode(R_L_InA1, OUTPUT);
  pinMode(R_L_InB1, OUTPUT);
  pinMode(R_L_PWM1, OUTPUT);
  pinMode(R_L_encodPinA1, INPUT);
  pinMode(R_L_encodPinB1, INPUT); 
  digitalWrite(R_L_encodPinA1, HIGH);                      // turn on pullup resistor
  digitalWrite(R_L_encodPinB1, HIGH);
  digitalWrite(F_L_drive_InA1, LOW);
  digitalWrite(F_L_drive_InB1, HIGH);
  
  digitalWrite(F_R_drive_InA2, LOW);
  digitalWrite(F_R_drive_InB2, HIGH);
  
  digitalWrite(R_L_InA1, HIGH);
  digitalWrite(R_L_InB1, LOW);
  
  digitalWrite(R_R_InA2, HIGH);
  digitalWrite(R_R_InB2, LOW);
  attachInterrupt(4, R_lencoder, FALLING);//matchs pin 19 left front motor
  attachInterrupt(5, R_rencoder, FALLING);//matchs pin 18 right front motor
  attachInterrupt(2, lencoder, FALLING);//matchs pin 21 left front motor
  attachInterrupt(3, rencoder, FALLING);//matchs pin 20 right front motor
  //for(int i=0; i<NUMREADINGS; i++)   readings[i] = 0;  // initialize readings to 0
  
  digitalWrite(left_enable, HIGH);
  //hb25
  leftRotation.attach(9, 1000, 2000); //minPulse, maxPulse);//10
  rightRotation.attach(11, 1000, 2000);// minPulse, maxPulse);//11
  Front_leftRotation.attach(8, 1000, 2000); //minPulse, maxPulse);//10
  Front_rightRotation.attach(10, 1000, 2000);//, minPulse, maxPulse);//11
  _TimeInfo.Update();
  //arduino pid
  Front_right.SetMode(AUTOMATIC);
  Front_left.SetMode(AUTOMATIC);
  Rear_right.SetMode(AUTOMATIC);
  Rear_left.SetMode(AUTOMATIC);
  Steer_F_R.SetMode(AUTOMATIC);
  Steer_F_L.SetMode(AUTOMATIC);
  Steer_R_R.SetMode(AUTOMATIC);
  Steer_R_L.SetMode(AUTOMATIC);
}

/*
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

timing loop
@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

*/
void loop()
{
   ReadSerial();
   Steer_enc[0] = analogRead(0);//front left
   Steer_enc[1] = analogRead(3);//rear left 
   Steer_enc[2] = analogRead(1);// front right
   Steer_enc[3] = analogRead(2);// rear right
    
   Front_Left_Steer_Enc = map (Steer_enc[0],0, 1024, -150, 150); 
   Front_Right_Steer_Enc = map (Steer_enc[2],0, 1024, -150, 150); 
   Rear_Left_Steer_Enc = map (Steer_enc[1],0, 1024, -150, 150);
   Rear_Right_Steer_Enc = map (Steer_enc[3],0, 1024, -150, 150);
   //Pose();
   
   //timing loop
  unsigned long milliSecsSinceLastUpdate = millis() - _TimeInfo.LastUpdateMillisecs;
  if(milliSecsSinceLastUpdate >= c_UpdateInterval)
  {
    //Serial.println(milliSecsSinceLastUpdate);
    // time for another update
    _TimeInfo.Update();
    if (_IsInitialized)
    {
      DoWork();
      Pose();
      move_F_L(); 
      move_F_R();
      move_R_L();
      move_R_R();
      
      Compute_F_L();
      Compute_F_R();
      Compute_R_L();
      Compute_R_R();
      //new_motor();
    }
    else
    {
      RequestInitialization();
    }
  }
 
   
   Front_rightRotation.write(90 + F_R_Output);
   Front_leftRotation.write(90 + F_L_Output);
   rightRotation.write(90+ R_R_Output);
   leftRotation.write(90+ R_L_Output);
   
   
  
}
/*
##################################################################################################3

                   End Timing loop
                   

##################################################################################################
*/
// from messenger

void ReadSerial()
{
  while (Serial.available())
  {
    _Messenger.process(Serial.read());
  }
}

// Define messenger function
void OnMssageCompleted()
{
  if (_Messenger.checkString("s"))
  {
    //SetSpeed();
    OmniAngle();
    
    return;
    
  }   
  if (_Messenger.checkString("v"))
  {
    //SetSpeed();
    OmniSpeed();
    
    return;
    
  }
  if (_Messenger.checkString("d"))
  {
    InitializeDriveGeometry();
    return;
  }
  if (_Messenger.checkString("j"))
  {
    //SetSpeed();
    void(* resetFunc) (void) = 0; //declare reset function @ address 0
    resetFunc();
    return;
    
  }
  

 

  // clear out unrecognized content
  while(_Messenger.available())
  {
    _Messenger.readInt();
  }
}
 
void OmniSpeed()
{

  
  Wheel_S_F_L = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  Wheel_S_F_R = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  Wheel_S_R_L = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  Wheel_S_R_R = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
 
}

void OmniAngle()
{

  Wheel_A_F_L = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  Wheel_A_F_R = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  Wheel_A_R_L = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  Wheel_A_R_R = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  
 
}
float GetFloatFromBaseAndExponent(int base, int exponent)
{
  return base * pow(10, exponent);
}

long mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void DoWork()
{
  _BatteryMonitor.Update();
  Serial.print("o"); // o indicates odometry message
  Serial.print("\t");
  Serial.print(X_dist);
  Serial.print("\t");
  Serial.print(Y_dist);
  Serial.print("\t");
  Serial.print(Heading);
  Serial.print("\t");
  Serial.print(Delta_X);//vel x
  Serial.print("\t");
  Serial.print(Delta_Y);//vel y
  Serial.print("\t");
  Serial.print(Hed);//vel in z in radians
  Serial.print("\t");
  Serial.print(Hed);
  Serial.print("\t");
  Serial.print(count_R_R);
  Serial.print("\t");
  Serial.print(speed_act_F_L);
  Serial.print("\t");
  Serial.print(speed_act_F_R);
  Serial.print("\t");
  Serial.print(speed_act_R_L);
  Serial.print("\t"); 
  Serial.print(speed_act_R_R);
  Serial.print("\t");
  Serial.print(PWM_val_F_L);
  Serial.print("\t");
  Serial.print(PWM_val_F_R);
  Serial.print("\t");
  Serial.print(PWM_val_R_L);
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("e"); // o indicates battery message
  Serial.print("\t");
  Serial.print(Front_Left_Steer_Enc);
  Serial.print("\t");
  Serial.print(Front_Right_Steer_Enc);
  Serial.print("\t");
  Serial.print(Rear_Left_Steer_Enc);
  Serial.print("\t");
  Serial.print(Rear_Right_Steer_Enc);
  Serial.print("\t");
  Serial.print("\n");
  
  Serial.print("b"); // o indicates battery message
  Serial.print("\t");
  Serial.print(_BatteryMonitor.BatteryVoltage, 1);
  Serial.print("\t");
  Serial.print(_BatteryMonitor.VoltageIsTooLow);
  //Serial.print(analogRead(4));
  Serial.print("\t");
  Serial.print("\n");
  
}

void RequestInitialization()
{
    _IsInitialized = true;

    //if (!_RobotParams.IsInitialized)
    //{
      //_IsInitialized = false;
      
      //Serial.print("reset_done"); // requesting initialization of the parameters of the differential drive needed for odometry calculations
      //Serial.print("\n");
   // }
    if (_Messenger.checkString("BatteryMonitorParams"))
    {
     InitializeBatteryMonitor();
    }
    
    if (_Messenger.checkString("DriveGeometry"))
    {
     InitializeDriveGeometry();
    }
   
}
void InitializeDriveGeometry()
{
  
  // todo
  F_L_K = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  float V_Cal = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  
}

void InitializeBatteryMonitor()
{
  float voltageTooLowlimit = GetFloatFromBaseAndExponent(_Messenger.readInt(), _Messenger.readInt());
  _BatteryMonitor.InitializeLowVoltageLimit(voltageTooLowlimit);

  /*
  Serial.print("battery monitor Params: ");
  Serial.print(voltageTooLowlimit);
  Serial.print("\n");
  */
}
