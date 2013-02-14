
/** Reset Functions **/
void Turret_Reset(){
  char temp = Serial_Wait();
  Turret_Reset_char(temp);
}

void Turret_Reset_char(char sense){
  LAPTOP.println("Resetting Turret");
  turret_motor.pwm(150);
  if(sense == 'c') {
    turret_motor.Control(FWD);
  }else{
    turret_motor.Control(BCK);
  }
  delay(200);

  int sensor_reading, detected_count = 0, reading_count = 0;

  while(detected_count<3){
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
    LAPTOP.println(sensor_reading);
    Check_Abort();
  }
  turret_motor.Brake(255);
  LAPTOP.println("Turret reset complete");  
}

void Move_Parallelogram(int dir, int num){
  LAPTOP.println("Resetting Parallelogram");
  if(dir){
    Parallelogram_Up();
  }else{
    Parallelogram_Down();
  }
  parallelogram_count = 0;
  while(parallelogram_count != num){
    Check_Abort();
  }
  Parallelogram_Stop();
  LAPTOP.println("Parallelogram Reset done.");
}

void Turret_ISR(){
  encoder_turret++;
}

void Parallelogram_ISR(){
 parallelogram_count++;
}

void Parameters_Reset()
{
  Query_Launchpad();
  encoder_motor1 = encoder_motor2 = 0; 
  mx = 0;
  my = 0;
  motor_heading = 1.57; //Increase accuracy
}
