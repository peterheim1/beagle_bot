/*
  auto dock program
 */

#include <Servo.h> 
#define Voltage_pin A0
#define Current_pin A1
#define L_IR_pin A2
#define R_IR_pin A3

int Drive_left = 90;
int Drive_right = 90;
int Right_Ir = 3;//green
int Left_Ir = 4;//yellow


Servo Right_motor;
Servo Left_motor;
const int Rear_bumper = 2;
int Rear_Bumper_State = 0;
int Right_Ir_State =0;
int Left_Ir_State = 0;
float Front_right_Ir =0;
float Front_left_Ir =0;
float voltage = 0;
float current = 0;


// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  pinMode(Right_Ir, INPUT);
  pinMode(Left_Ir, INPUT);
  pinMode(Rear_bumper, INPUT);
  Right_motor.attach(8);
  Left_motor.attach(7);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input pin:
  Right_Ir_State = digitalRead(Right_Ir);
  Left_Ir_State = digitalRead(Left_Ir);
  Rear_Bumper_State = digitalRead(Rear_bumper);
  //int sensorValue = analogRead();
  voltage = analogRead(Voltage_pin) * (19.0 / 1023.0);
  current = analogRead(Current_pin) * (19.0 / 1023.0);
  Front_right_Ir = analogRead(R_IR_pin);
  Front_left_Ir = analogRead(L_IR_pin);
  
  // print out the state of the button:
  Serial.print(Left_Ir_State);
  Serial.print("      ");
  Serial.print(Right_Ir_State);
  Serial.print("  rear bumper    ");
  Serial.print(Rear_Bumper_State);
  Serial.print(" voltage     ");
  Serial.print(voltage);
  Serial.print("  current    ");
  Serial.print(current);
  Serial.print(" RIGHT IR     ");
  Serial.print(Front_right_Ir);
  Serial.print("  LEFT IR    ");
  Serial.println(Front_left_Ir);
  delay(2);        // delay in between reads for stability
  //AutoDock();
  auto1();
  Left_motor.write(Drive_left);
  Right_motor.write(Drive_right);
}
void auto1(){
  
  //if (Rear_Bumper_State > 0){
  //Drive_right = 90; 
  //Drive_left = 90;
  //}
  if (Right_Ir_State < 1 && Rear_Bumper_State == 0){ 
  Drive_right = 110; }
  else{Drive_right = 90; }
  
  if (Left_Ir_State < 1 && Rear_Bumper_State == 0){ 
  Drive_left = 110; }
  else{Drive_left = 90; }
  
  
}



void AutoDock(){
  while (Rear_Bumper_State > 0){
  if (Left_Ir_State < 1){ 
  Drive_right = 110; 
}             

  if (Right_Ir_State < 1){ 
  Drive_left = 110; 
}             
  if (Right_Ir_State == 1 && Right_Ir_State ==1){ 
  Drive_left = 110; 
  Drive_right = 110;
} 

  if (Right_Ir_State < 1 && Right_Ir_State < 1){ 
  Drive_left = 90; 
  Drive_right = 90;
}               
}
}

