void getMotorData_F_L()  {                                                        // calculate speed, 
static long countAnt_F_L = 0;                                                   // last count
  speed_act_F_L = ((count_L - countAnt_F_L)*(60*(1000/c_UpdateInterval)))/(2480);              // 2500 counts per output shaft rev
  Front_left_gap = abs(speed_act_F_L - Wheel_S_F_L);
  countAnt_F_L = count_L;                  
 
}

void getMotorData_F_R()  {                                                        // calculate speed, 
static long countAnt_F_R = 0;                                                   // last count
  speed_act_F_R = ((count_R - countAnt_F_R)*(60*(1000/c_UpdateInterval)))/(2480);              // 2500 counts per output shaft rev
  Front_right_gap = abs(speed_act_F_R - Wheel_S_F_R);
  countAnt_F_R = count_R;                  
  
}

void getMotorData_R_L()  {                                                        // calculate speed, 
static long countAnt_R_L = 0;                                                   // last count
  speed_act_R_L = ((count_R_L - countAnt_R_L)*(60*(1000/c_UpdateInterval)))/(3500);              // 2500 counts per output shaft rev
  Rear_left_gap = abs(speed_act_R_L - Wheel_S_R_L);
  countAnt_R_L = count_R_L;                  
 
}

void getMotorData_R_R()  {                                                        // calculate speed, 
static long countAnt_R_R = 0;                                                   // last count
  speed_act_R_R = ((count_R_R - countAnt_R_R)*(60*(1000/c_UpdateInterval)))/(3500);              // 2500 counts per output shaft rev
  Rear_right_gap = abs(speed_act_R_R - Wheel_S_R_R);
  countAnt_R_R = count_R_R;                  
  
}




void rencoder()  {                                    // pulse and direction, direct port reading to save cycles
  if (PINC & 0b00000100)    count_R++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      count_R--;                // if (digitalRead(encodPinB1)==LOW)   count --;
}

void lencoder()  {                                    // pulse and direction, direct port reading to save cycles
  if (PINC & 0b00001000)    count_L++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      count_L--;                // if (digitalRead(encodPinB1)==LOW)   count --;
}
 
void R_rencoder()  {                                    // pulse and direction, direct port reading to save cycles
  if (PINC & 1)             count_R_R--;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      count_R_R++;                // if (digitalRead(encodPinB1)==LOW)   count --;
  }


void R_lencoder()  {                                    // pulse and direction, direct port reading to save cycles
  if (PINC & 0)             count_R_L--;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      count_R_L++;                // if (digitalRead(encodPinB1)==LOW)   count --;
     
}

void move_F_R(){
    
    getMotorData_F_R();
    if(Front_right_gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    
    Front_right.SetTunings(2.1, 1.7, 0);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     Front_right.SetTunings(4, 6, .1);
  }
    Front_right.Compute();
    analogWrite(F_R_drive_PWM2, PWM_val_F_R); 
}

void move_F_L(){
   getMotorData_F_L(); 
   if(Front_left_gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    
    Front_left.SetTunings(2.1, 1.7, 0);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     Front_left.SetTunings(4, 6, .1);
  }
  
    Front_left.Compute();
    analogWrite(F_L_drive_PWM1, PWM_val_F_L); 
}

void move_R_R(){
    //get encoder count
    getMotorData_R_R();
    if(Rear_right_gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    //PID Front_left(&speed_act_F_L, &PWM_val_F_L, &Wheel_S_F_L, 2.1,1.7,0, DIRECT);
    Rear_right.SetTunings(2.1, 1.7, 0);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     Rear_right.SetTunings(4, 6, .1);
  }
    Rear_right.Compute();
    analogWrite(R_R_PWM2, PWM_val_R_R); 
    
}

void move_R_L(){
   getMotorData_R_L(); 
  if(Rear_left_gap<10)
  {  //we're close to setpoint, use conservative tuning parameters
    //PID Front_left(&speed_act_F_L, &PWM_val_F_L, &Wheel_S_F_L, 2.1,1.7,0, DIRECT);
    Rear_left.SetTunings(2.1, 1.7, 0);
  }
  else
  {
     //we're far from setpoint, use aggressive tuning parameters
     Rear_left.SetTunings(5, 7, .1);
  }
    Rear_left.Compute();
    analogWrite(R_L_PWM1, PWM_val_R_L); 
}

