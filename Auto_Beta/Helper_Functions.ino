
/** Helper functions **/
int inline Check_Mirror(int a, int b){
  return mirror?a:b;
}

char Serial_Wait()
{
  LAPTOP.println("Waiting for Serial Input ");
  while(!LAPTOP.available());
  return LAPTOP.read();
}

void Check_Abort(){
  if (LAPTOP.available()) {
    left_motor.Brake(255);
    right_motor.Brake(255);
    turret_motor.Brake(0);
    while(1);
  }
}

void Serial_Print(){
  LAPTOP.print("encl: ");
  LAPTOP.print(encoder_left);
  LAPTOP.print("\tencr: ");
  LAPTOP.print(encoder_right);  
  LAPTOP.print("\tenct: ");
  LAPTOP.print(encoder_turret);
  LAPTOP.print("\tcsl: ");
  LAPTOP.print(servo_left.GetAngle());
  LAPTOP.print("\tcsr: ");
  LAPTOP.print(servo_right.GetAngle());
  if(pid_enable){
    LAPTOP.print("\t  Current Diff: ");
    LAPTOP.print(input - setpoint);
    LAPTOP.print("\t PID Output: ");
    LAPTOP.print(output);
    LAPTOP.print("\t Left PWM: ");
    LAPTOP.print(base_pwm + output);
    LAPTOP.print("\t Right PWM: ");
    LAPTOP.print(base_pwm - output);  
    LAPTOP.print("\tx: ");
    LAPTOP.println(mx);
  }else{
    LAPTOP.print("\n");
  }
}

void Serial_Print_Sensors(){
  LAPTOP.print(L2.High()); 
  LAPTOP.print(L1.High()); 
  LAPTOP.print(R1.High());
  LAPTOP.println(R2.High());
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

void Check_Motors(){
  while(1){
    left_motor.Control(FWD);
    left_motor.pwm(100);
    right_motor.Control(FWD);
    right_motor.pwm(100);
    Check_Abort();
    Query_Launchpad();
    Serial_Print();
  }
}


