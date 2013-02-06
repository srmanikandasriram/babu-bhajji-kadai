
/** Reset Functions **/

void Turret_Reset(){
  LAPTOP.println("Resetting Turret");
  turret_motor.pwm(150);
  if(Serial_Wait() == 'c') {
    turret_motor.Control(FWD);
  }else{
    turret_motor.Control(BCK);
  }
  delay(TURRET_DELAY);

  int sensor_reading, detected_count = 0, reading_count = 0;

  while(!LAPTOP.available()){
    sensor_reading = 0;
    reading_count = 0;
    while(reading_count<5){
      sensor_reading += analogRead(TURRET_SENSOR_PIN);
      delay(5);
      reading_count++;
    }
    sensor_reading /= 5;
    if(sensor_reading <= 300)
      detected_count++;
    else
      detected_count = 0;
    if(detected_count>5){
      turret_motor.Brake(0);
      break;
    }
    LAPTOP.println(sensor_reading);
  }
  if(LAPTOP.available()){
    LAPTOP.read();
    LAPTOP.println("Aborted Turret reset!");
    turret_motor.Brake(0);
    while(1);
  }
  LAPTOP.println("Turret reset complete");  
}

void Move_Parallelogram(int dir, int num){
  LAPTOP.println("Resetting Parallelogram");
  attachInterrupt(1, Parallelogram_ISR, FALLING);
  int k;
  for( k = 0; k<num; k++){
    if(dir){
      digitalWrite(48,HIGH);
      digitalWrite(49,LOW);
    }else{
      digitalWrite(48,LOW);
      digitalWrite(49,HIGH);
    }
    delay(500);
    parallelogram_reset = false;
    while(!parallelogram_reset){
      if(Serial.available()){
        Serial.read();
        Serial.println("ABORTED!");
        digitalWrite(48,LOW);
        digitalWrite(49,LOW);
        while(1);
      }
    }
    digitalWrite(48,LOW);
    digitalWrite(49,LOW);
    LAPTOP.println("one tape done");
    delay(500);
  }
  detachInterrupt(1);
}

void Turret_ISR(){
  encoder_turret++;
}

void Parallelogram_ISR(){
 parallelogram_reset = true;
}

void Parameters_Reset()
{
  Encoders.Read_Data();
  Encoders.Reset();
  Query_Launchpad();
  encoder_motor1 = encoder_motor2 = 0; 
  encoder_turret = 0;
  mx=0;
  my=0;
  mheading=1.57; //Increase accuracy
}
