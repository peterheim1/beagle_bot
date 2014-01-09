#include "DualVNH5019MotorShield.h"
#include "Arduino.h"
#include "commands.h"
#include <PID_v1.h>
#include <Encoder.h>

DualVNH5019MotorShield md;

Encoder Left_Encoder(2, 6);
Encoder Right_Encoder(3, 5);
// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Serial port baud rate */
#define BAUDRATE     57600

/* variables for encoder*/
double Setpoint, Input, Output;
PID Left(&Input, &Output, &Setpoint,10,2,2, DIRECT);



/* Clear the current command parameters */
void resetCommand() {
cmd = NULL;
memset(argv1, 0, sizeof(argv1));
memset(argv2, 0, sizeof(argv2));
arg1 = 0;
arg2 = 0;

arg = 0;
index = 0;
}
void setup()
{
  Serial.begin(BAUDRATE);
  md.init();
  
  Input = analogRead(0);
  Setpoint = 250;

  //turn the PID on
  Left.SetMode(AUTOMATIC);
 
}


void loop()
{
  //md.setM1Speed(200);
  Input = analogRead(0);
  Left.Compute();
  md.setM1Speed(Output);
  
 while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
}

int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(Setpoint);
    break;
  case ANALOG_WRITE:
    //analogWrite(arg1, arg2);
    Serial.println(Output); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].write(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].read());
    
    break;
#endif
  case READ_ENCODERS:
    Serial.print(Left_Encoder.read());
    Serial.print("  L  R  ");
    Serial.print(Right_Encoder.read());
    Serial.println(" ");   
    break;
  case MOTOR_SPEEDS:
    /* Reset the auto stop timer */
    //lastMotorCommand = millis();
    if (arg1 == 0 && arg2 == 0) {
      md.setM1Speed(0);
      //moving = 0;
    }
    //else moving = 1;
    Setpoint = (arg1);//md.setM1Speed(arg1);
    Left.Compute();
    md.setM1Speed(Output);
    Serial.println("OK"); 
    break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    //Kp = pid_args[0];
    //Kd = pid_args[1];
    //Ki = pid_args[2];
    //Ko = pid_args[3];
    Serial.println("OK");
    break;
  default:
    Serial.println("Invalid Command");
    break;
  }
}
