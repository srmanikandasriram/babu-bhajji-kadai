
/** Helper functions **/
int inline Check_Mirror(int a, int b){
  return mirror?a:b;
}

char Serial_Wait(){
  LAPTOP.println("Waiting for Serial Input ");
  while(!LAPTOP.available());
  return LAPTOP.read();
}

int Toggle_Wait(){
  int static pin_state = SW.High();
  while(pin_state == SW.High());
  pin_state = SW.High();
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
  LAPTOP.print(S1.High()); 
  LAPTOP.print(S2.High()); 
  LAPTOP.print(S3.High());
  LAPTOP.println(S4.High());
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

int Parallelogram_Reached(int count){
  Serial.print(" PARALLELOGRAM COUNT:");
  Serial.println(parallelogram_count);
  if( parallelogram_count >= count ){
    Parallelogram_Stop();
    return 1;
  }
  return 0;
}

void Check_Motors(){
  while(1){
    motor1.Control(FWD, 100);
    motor2.Control(FWD, 100);
    Check_Abort();
    Query_Launchpad();
    Serial_Print();
  }
}
