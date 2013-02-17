
/** Helper functions **/

inline int Check_Mirror(int a, int b){
  return mirror?a:b;
}

inline void Move_Forward(uint8_t pwm1, uint8_t pwm2){
  motor1.Control(FWD, pwm1);
  motor2.Control(FWD, pwm2);
}

inline void Move_Back(uint8_t pwm1, uint8_t pwm2){
  motor1.Control(BCK, pwm1);
  motor2.Control(BCK, pwm2);
}

inline void Motors_Brake(uint8_t pwm1, uint8_t pwm2){\
  motor1.Brake(pwm1);
  motor2.Brake(pwm2);
}

inline void Parallelogram_Up_I(){
  LAPTOP.println("UP");
  if(!digitalRead(PARALLELOGRAM_TRIP_SWITCH_TOP)){
    attachInterrupt(PARALLELOGRAM_TRIP_SWITCH_TOP, Parallelogram_Tripped_ISR, RISING);
    digitalWrite(PARALLELOGRAM_PIN1, HIGH);
    digitalWrite(PARALLELOGRAM_PIN2, LOW);
  }
}

inline void Parallelogram_Down_I(){
  LAPTOP.println("DOWN");
  if(!digitalRead(PARALLELOGRAM_TRIP_SWITCH_BOTTOM)){
    attachInterrupt(3, Parallelogram_Tripped_ISR, RISING);
    digitalWrite(PARALLELOGRAM_PIN1, LOW);
    digitalWrite(PARALLELOGRAM_PIN2, HIGH);
  }
}

inline void Parallelogram_Up(){
  digitalWrite(PARALLELOGRAM_PIN1, HIGH);
  digitalWrite(PARALLELOGRAM_PIN2, LOW);
}

inline void Parallelogram_Down(){
  digitalWrite(PARALLELOGRAM_PIN1, LOW);
  digitalWrite(PARALLELOGRAM_PIN2, HIGH);
}

inline void Parallelogram_Stop(){
  digitalWrite(PARALLELOGRAM_PIN1, LOW);
  digitalWrite(PARALLELOGRAM_PIN2, LOW);
}

inline void Check_Abort(){
  if( LAPTOP.available() )
    Abort();
}

char Serial_Wait(){
  LAPTOP.println("Waiting for Serial Input ");
  while(!LAPTOP.available());
//    LAPTOP.println(analogRead(A2)); // for sharp
  return LAPTOP.read();
}

void Toggle_Wait(){
  int static pin_state = digitalRead(15);
  if(pin_state == HIGH){
    while(digitalRead(15));
    delay(200);
    while(!digitalRead(15));
    delay(200);
    while(digitalRead(15));    
  }else{
    while(!digitalRead(15));
    delay(200);
    while(digitalRead(15));
  }
  delay(1000);
}

void Abort(){
  Motors_Brake(255, 255);
  turret_motor.Brake(0);
  Parallelogram_Stop();
  LAPTOP.println("ABORTED!");
  while(1);
}

void Serial_Print(){
  LAPTOP.print("encl: ");
  LAPTOP.print(encoder_motor1);
  LAPTOP.print("\tencr: ");
  LAPTOP.print(encoder_motor2);  
  LAPTOP.print("\tenct: ");
  LAPTOP.print(encoder_turret);
  LAPTOP.print("\tcsl: ");
  LAPTOP.print(servo1.GetAngle());
  LAPTOP.print("\tcsr: ");
  LAPTOP.print(servo2.GetAngle());
  LAPTOP.print("\tsharp: ");
  LAPTOP.print(analogRead(SHARP_SENSOR_PIN));
  if(pid_enable){
    LAPTOP.print("\t  Current Diff: ");
    LAPTOP.print(input - setpoint);
    LAPTOP.print("\t PID Output: ");
    LAPTOP.print(output);
    if( pid_type == STRAIGHT_LINE_PID ){
      LAPTOP.print("\t Left PWM: ");
      LAPTOP.print(base_pwm + output);
      LAPTOP.print("\t Right PWM: ");
      LAPTOP.print(base_pwm - output);  
      LAPTOP.print("\tx: ");
      LAPTOP.println(mx);
    }else{
      LAPTOP.print("\t Left PWM: ");
      LAPTOP.println(output);
    }
  }else{
    LAPTOP.print("\n");
  }
}

void Serial_Print_Sensors(){
  if(S1.High()||S2.High()||S3.High()||S4.High()){
  LAPTOP.print(S1.High()); 
  LAPTOP.print(S2.High()); 
  LAPTOP.print(S3.High());
  LAPTOP.println(S4.High());
  }
}

void Handshake_Launchpad(){
  char msg = '@';
  while(msg != '#'){
    msg = '@';
    LAUNCHPAD.write(msg);
    while(!LAUNCHPAD.available());
    msg = LAUNCHPAD.read();
    delay(500);    
  }
  LAPTOP.println("Handshake completed");
}

// Function for stopping parallelogram at top position. Parallelogram will stop only if trip is switched else will continue to try to rise up.
int Parallelogram_Tripped(){
   if(digitalRead(PARALLELOGRAM_TRIP_SWITCH_TOP)){
     Parallelogram_Stop();
     return 1;
   }else{
     Parallelogram_Up(); // Case added to prevent / counter sagging
     return 0;
   }
}
/*
int Parallelogram_Reached(int count){
  LAPTOP.print("PARALLELOGRAM COUNT:");
  LAPTOP.println(parallelogram_count);
  if( parallelogram_count >= count ){
    if(  parallelogram_sensor.Low() ){
      Parallelogram_Stop();
      return 1;
    }   
  }
  return 0;
}
*/
int Turret_Reached(int count){
  LAPTOP.print("TURRET COUNT:");
  LAPTOP.println(turret_count);
  if( turret_count >= count ){
    turret_motor.Brake(255);
    return 1;
  }
  return 0;
}

void Check_Motors(int pwm){
  while(1){
    motor1.Control(FWD, pwm);
    motor2.Brake(255);
//    motor2.Control(FWD, pwm);
    Check_Abort();
    Query_Launchpad();
    Serial_Print();
  }
}
