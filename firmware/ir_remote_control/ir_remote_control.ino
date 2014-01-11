/* YourDuino.com Example Software Sketch
 IR Remote Kit Test
 Uses YourDuino.com IR Infrared Remote Control Kit 2
 http://arduino-direct.com/sunshop/index.php?l=product_detail&p=153
 based on code by Ken Shirriff - http://arcfn.com
 Get Library at: https://github.com/shirriff/Arduino-IRremote
 Unzip folder into Libraries. RENAME folder IRremote
 terry@yourduino.com */

/*-----( Import needed libraries )-----*/

#include "IRremote.h"
#define InA1            11                      // INA motor pin
#define InB1            8                       // INB motor pin
#define PWM1            10                       // PWM motor pin
#define InA2            7                      // INA motor pin
#define InB2            4                       // INB motor pin
#define PWM2            9                       // PWM motor pin


/*-----( Declare Constants )-----*/
int receiver = 12; // pin 1 of IR receiver to Arduino digital pin 11

/*-----( Declare objects )-----*/
IRrecv irrecv(receiver);           // create instance of 'irrecv'
decode_results results;            // create instance of 'decode_results'
/*-----( Declare Variables )-----*/
void setM2Speed(int speed){
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
    
  analogWrite(PWM1,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  
  if (speed == 0)
  {
    digitalWrite(InA1,LOW);   // Make the motor coast no
    digitalWrite(InB1,LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(InA1,HIGH);
    digitalWrite(InB1,LOW);
  }
  else
  {
    digitalWrite(InA1,LOW);
    digitalWrite(InB1,HIGH);
  }
}
 /// left motor
void setM1Speed(int speed){
  unsigned char reverse = 0;
  
  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 400)  // Max PWM dutycycle
    speed = 400;
    
  analogWrite(PWM2,speed * 51 / 80); // default to using analogWrite, mapping 400 to 255
  
  if (speed == 0)
  {
    digitalWrite(InA2,LOW);   // Make the motor coast no
    digitalWrite(InB2,LOW);   // matter which direction it is spinning.
  }
  else if (reverse)
  {
    digitalWrite(InA2,HIGH);
    digitalWrite(InB2,LOW);
  }
  else
  {
    digitalWrite(InA2,LOW);
    digitalWrite(InB2,HIGH);
  }
}

void setup()   /*----( SETUP: RUNS ONCE )----*/
{
  Serial.begin(9600);
  Serial.println("IR Receiver Raw Data + Button Decode Test");
  irrecv.enableIRIn(); // Start the receiver
 pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
}/*--(end setup )---*/


void loop()   /*----( LOOP: RUNS CONSTANTLY )----*/
{
  if (irrecv.decode(&results)) // have we received an IR signal?

  {
//    Serial.println(results.value, HEX);  UN Comment to see raw values
    translateIR(); 
    irrecv.resume(); // receive the next value
  }  
}/* --(end main loop )-- */

/*-----( Declare User-written Functions )-----*/
void translateIR() // takes action based on IR code received

// describing Car MP3 IR codes 

{

  switch(results.value)

  {

  case 0xc4c:  
    Serial.println(" power off           "); 
    setM1Speed(0);
    setM2Speed(0);
    break;

  case 0x44c:  
    Serial.println(" power off            "); 
    setM1Speed(0);
    setM2Speed(0);
    break;

  case 0x75529fc6:  
    Serial.println(" forward            ");
    setM1Speed(250);
    setM2Speed(250); 
    break;

  case 0x76324f07:  
    Serial.println(" forward          "); 
    setM1Speed(250);
    setM2Speed(250);
    break;

  case 0x7652a15b:  
    Serial.println(" backwards           "); 
    setM1Speed(-250);
    setM2Speed(-250);
    break;

  case 0x75324d72:  
    Serial.println(" backwards     "); 
    setM1Speed(-250);
    setM2Speed(-250);
    break;

  case 0xc5e:  
    Serial.println(" right turn           ");
    setM1Speed(200);
    setM2Speed(-200); 
    break;

  case 0x45e:  
    Serial.println(" right turn           "); 
    setM1Speed(200);
    setM2Speed(-200); 
    break;

  case 0xc5f:  
    Serial.println(" left turn            "); 
    setM1Speed(-200);
    setM2Speed(200); 
    break;

  case 0x45f:  
    Serial.println(" left turn             ");
    setM1Speed(-200);
    setM2Speed(200);  
    break;

  case 0xFF9867:  
    Serial.println(" 100+           "); 
    break;

  case 0xFFB04F:  
    Serial.println(" 200+           "); 
    break;

  case 0xFF30CF:  
    Serial.println(" 1              "); 
    break;

  case 0xFF18E7:  
    Serial.println(" 2              "); 
    break;

  case 0xFF7A85:  
    Serial.println(" 3              "); 
    break;

  case 0xFF10EF:  
    Serial.println(" 4              "); 
    break;

  case 0xFF38C7:  
    Serial.println(" 5              "); 
    break;

  case 0xFF5AA5:  
    Serial.println(" 6              "); 
    break;

  case 0xFF42BD:  
    Serial.println(" 7              "); 
    break;

  case 0xFF4AB5:  
    Serial.println(" 8              "); 
    break;

  case 0xFF52AD:  
    Serial.println(" 9              "); 
    break;

  default: 
    Serial.println(" other button   ");

  }

  delay(500);


} //END translateIR



/* ( THE END ) */
