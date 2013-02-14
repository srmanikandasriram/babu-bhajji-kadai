
/** Auto Fallback **/

void Auto_Fallback(){
  LAPTOP.println("\n Fallback! ");
  //Move_Parallelogram(FWD,1);
//Parallelogram_Up();
//delay(300);
//Parallelogram_Stop();
  while(1){
    //  prevmillis = millis();
    if( (encoder_motor1 < distances_fallback[path_phase]) && (encoder_motor2 < distances_fallback[path_phase]) ){
      Move_Turret();
      Move_Servo();
      Query_Launchpad();
      if (pid_enable){
        PID_Adjust();
      }
      Serial_Print();
      Check_Abort();
    } 
    else {
      LCD.setCursor(0,1);
      LAPTOP.print("Path phase:");
      LAPTOP.println(path_phase);
      path_phase++;
      Transform_Fallback[path_phase]();
      if(fallback_begin)
        break;
    }
    //  LAPTOP.println(millis() - prevmillis);
  }
}

void Pick_Leaves1(){
  LAPTOP.println("Going to pick up the leaves.");
  pid_enable = false;
  Move_Forward(30, 30);
  encoder_turret = 0;
  turret_motor.Control(Check_Mirror(BCK,FWD),255);
  actuation_phase++;
  servo1.SetTargetAngle(2);
  servo2.SetTargetAngle(2);
}

void Accelerate_Bot1(){
  LAPTOP.println("Stopping to pick up leaves");
    Serial_Print();
  while(analogRead(SHARP_SENSOR_PIN)<400){
    Move_Servo();
    Serial_Print();
    Move_Turret();
  }
  Motors_Brake(HARDBRAKE, HARDBRAKE);
  while(encoder_turret<TURRET_ANG1){
    Move_Servo();
    Serial_Print();
  }
  turret_motor.Brake(0);

  Actuate_Low(LEFT_VG);
  Actuate_Low(RIGHT_VG);
  Actuate_High(V_PISTON);
  delay(1000);
  Actuate_Low(V_PISTON);
  LAPTOP.println("Picked up leaves");
  Toggle_Wait();
  Launchpad_Reset();
  encoder_motor1 = encoder_motor2 = 0;
  Serial_Print();
  pid_enable = true;
  base_pwm = minimum_pwm;
  Move_Forward(minimum_pwm, minimum_pwm);
  encoder_turret = 0;
  turret_motor.Control(Check_Mirror(FWD,BCK), 220);
  actuation_phase++;
  LAPTOP.println("Acceleration begun");
  servo1.SetTargetAngle(3);
  servo2.SetTargetAngle(3);
  while(base_pwm<maximum_pwm){
    base_pwm += acceleration;
    if(base_pwm == 36){
      Serial.println("\n PID begun \n");    /// indicate start of PID
      encoder_motor1 = encoder_motor2;
      pid.SetOutputLimits(-35,35);
    }
    if(base_pwm>35){
      PID_Adjust();
    }
    Query_Launchpad();
    Move_Turret();
    Move_Servo();
    delay(acceleration_delay);
    Serial_Print();
    Check_Abort();
  }

  LAPTOP.println("Acceleration completed.");
  cons_kp = 1.5;
  pid.SetTunings(cons_kp,cons_ki,cons_kd);
  pid.SetOutputLimits(-45,45);
}

void Decelerate_Bot1(){

  LAPTOP.println("Deceleration begun!");

  while(base_pwm>slowdown_pwm){
    base_pwm -= deceleration;
    Query_Launchpad();
    PID_Adjust();
    delay(deceleration_delay);
    Move_Turret();
    Move_Servo();
    Serial_Print();
    Check_Abort();
  }

  LAPTOP.println("Deceleration done");
  base_pwm = slowdown_pwm;
  pid.SetOutputLimits(-35,35);
}

void Detect_Line(){
  Motors_Brake(HARDBRAKE, HARDBRAKE);
  LAPTOP.println("Waiting for turret to turn");
  while(encoder_turret<TURRET_ANG2);
  turret_motor.Brake(0);
  //  Toggle_Wait();
  delay(400);
  LAPTOP.println("Moving forward slightly");
  Move_Forward(20,20);
  while(S2.Low()&&S3.Low());
  LAPTOP.println("Line detected");
  pid_enable = false;
  Launchpad_Reset();
  encoder_motor1 = 0; 
  encoder_motor2 = 0;
}

void Auto_Fallback_Begin(){
  LAPTOP.println("Going into soft turn");
  motor1.pwm(50);
  motor2.Brake(HARDBRAKE);
  while(S1.Low()&&S2.Low()&&S3.Low()&&S4.Low()){
    LAPTOP.println("\n DETECTED LINE");
  }
  LAPTOP.println("Aligned onto the line");
  Motors_Brake(255,255);
  fallback_begin = true;
}

void LineFollow_Fallback(){
  int strip_count = 0;
  LAPTOP.println("Commencing Auto Fallback Stratergy. Left turn complete.");
  Parameters_Reset();

  // Till Junction
  // NOTE: In first two while(1) loops, need to add turret angle change

  encoder_turret = 0;
  turret_motor.Control(Check_Mirror(FWD,BCK),255);
  while(1){
    LineFollow34();
    if(TURRET_SENSOR.Low())
      if(strip_count)
        turret_motor.Brake(255);
      else
        strip_count++;
    if(( S4.High() && S2.High() )||( S1.High() && S3.High() ))
      break;
  }
  Motors_Brake(255,255);
  // encoder_turret = 0;
  /*turret_motor.Control(Check_Mirror(FWD,BCK),255);
   while(encoder_turret<400);*/
  while(TURRET_SENSOR.High());
  turret_motor.Brake(255);
  servo1.Extend();
  servo2.Extend();
  delay(500);
  Actuate_High(LEFT_VG);    //first two leaves dropped
  Actuate_High(RIGHT_VG);
  delay(1000); 
  Toggle_Wait(); 
  // Linefollow till fourth ring drop site. Drop third leaf. 
  Parameters_Reset();
  motor2.Control(FWD,100);
  delay(600);
  servo1.Middle();
  servo2.Middle();
  while(S4.Low()&&S3.Low()&&S1.Low()&&S2.Low());
  Motors_Brake(255,255);

  // Till Junction
  // NOTE: In first two while(1) loops, need to add turret angle change


  encoder_turret = 0;

  while(1)
  {
    LineFollow_Straight();
    if(encoder_turret<TURRET_ANG2+400)
      turret_motor.Control(Check_Mirror(BCK,FWD),255);
    else 
      turret_motor.Brake(255);

    if(( S4.High() && S2.High() && S3.High() )||( S1.High() && S3.High() &&S2.High()  ))
      break;
  }
  Motors_Brake(255,255);
  while(encoder_turret<TURRET_ANG2+550);
  turret_motor.Brake(255);
  delay(50);
  motor1.Control(FWD,50);
  delay(600);
  while(S1.Low()&&S2.Low() &&S3.Low());
  Motors_Brake(255,255);
  delay(500);
  Parameters_Reset();
  while(1)
  {
    Serial.print("Moving to third leaf");
    LineFollow12();
    if(analogRead(SHARP_SENSOR_PIN)>450)
      break;
  }
  Motors_Brake(255,255);
  delay(500);
  //Toggle_Wait();
  Actuate_High(MIDDLE_VG);
  delay(400);
  LAPTOP.println("Dropped Third Leaf");
  Toggle_Wait();
  servo1.Home();
  encoder_turret = 0;  
  turret_motor.Control(FWD,255);
  while(TURRET_SENSOR.High());
  while(TURRET_SENSOR.Low());
  while(TURRET_SENSOR.High());  
  turret_motor.Brake(255);
  //Parallelogram_Down();
  //while(!Parallelogram_Reached(1));
  
  Serial.println("Parallelogram should be down by now");
  
  //Linefollowing centred on S2 and S1. Linefollowing with brake till bud junction
  Parameters_Reset();
  while(1)
  {
    if(!LineFollow12_Encoders(2000)) //&& Parallelogram_Reached(2) && encoder_turret>TURRET_ANG5) 
      break;    
  }
  while(LineFollow_Curve_Precision());
  Motors_Brake(255,255);  
  //Pick up, reverse and Go to Tokyo

  //  while(!Parallelogram_Reached(1));
  ///May have to move forward slightly here. Parallelogram can alternatively be lowerd early in the previous while(1) loop
 delay(1000);
  Actuate_Low(GRIPPER);                               // to pick up bud 1

  ///Take Parallelogram to lowest position
  //Toggle_Wait();

  parallelogram_count = 0;
  Parallelogram_Down(); 

 // while(!Parallelogram_Reached(1));
  delay(500);
  Parallelogram_Stop();
  Toggle_Wait();
  LAPTOP.println("Reverse");
  //Toggle_Wait();

  ///Take Parallelogram to mid position before reverse
  //  parallelogram_count = 0;
  //  Parallelogram_Up(); 
  //  while(!Parallelogram_Reached(1));
  Move_Parallelogram(FWD,1);
  detachInterrupt(PARALLELOGRAM_SENSOR_PIN);
  Parallelogram_Up();                                        //to raise para after black tape, to avoid count while shaking
  delay(500);
  Parallelogram_Stop();
  //Toggle_Wait();
  Parameters_Reset();
  Move_Back(150,100);
  Run_For_Encoder_Count(8400);

  Motors_Brake(255,0);
  Parameters_Reset();
  motor2.Control(BCK,30);
  while(encoder_motor2<9000){ 
    Query_Launchpad();
    if((S2.High() || S3.High() )&&encoder_motor2>2000){
      LAPTOP.println("This is Tokyo");
      break;
    }
  }  
  Motors_Brake(255,255);
  LAPTOP.println("Left Turn Done");
  delay(150);

  motor2.Control(FWD,30); //To get back on track
  while(!S3.High());
  Motors_Brake(255,255);
  delay(500);

  ///Take Parallelogram to top position  
  attachInterrupt(PARALLELOGRAM_SENSOR_PIN, Parallelogram_ISR, FALLING);
  parallelogram_count = 0;
  Parallelogram_Up();

  //Line Follow to Manual Bot
  Parameters_Reset();  
  LAPTOP.println("Ready to linefollow");  
  while(1){
    LAPTOP.print("Linefollow ONE");
    LineFollow_Straight_Precision();
    if((S3.High() && S1.High()) || (S4.High() && S2.High()) || (S3.High() && S2.High()))
      break;
    Parallelogram_Reached(1);
  }
  Move_Forward(40,40);  //To bypass junction
  delay(300);

  while(1){
    LAPTOP.print("Linefollow Precision");
    LineFollow_Straight_Precision();
    if(( S3.High() && S1.High() )||( S4.High() && S2.High())||(S3.High() && S2.High() ))
      break;
    Parallelogram_Reached(1);
  }
  Motors_Brake(255,255);
  while(!Parallelogram_Reached(1));
  LAPTOP.println("Meet the Manual Bot");


  //COMMS CODE COMES HERE DA!
  Toggle_Wait();  
  Actuate_High(GRIPPER);

  LAPTOP.println("Ready to go for two and three");
  delay(2000);

  //Continue to pickup bud two and three
  for(int i = 0; i<2; i++){
    Parameters_Reset();                  
    Move_Back(100,240);
    Run_For_Encoder_Count(4000); 
    LAPTOP.println("Part One done"); 

    ///Take Parallelogram to lowest position

      //parallelogram_count = 0;
    //Parallelogram_Down();


    Motors_Brake(0,255);
    motor1.Control(BCK,30);
    delay(600);
    while(S4.Low()&&S3.Low()&&S1.Low()&&S2.Low());

    Motors_Brake(255,255);
    Move_Parallelogram(BCK,1);
    Move_Forward(20,20);
    Motors_Brake(255,255);
    delay(500);

    Parameters_Reset();
    while(1){
      if(!LineFollow_Encoders(9000,2))
        break;
      //Parallelogram_Reached(2);
    }      

    while(LineFollow_Curve_Precision());

    if(i==1){
      Parameters_Reset();
      while(1){
        if(!LineFollow_Encoders(2000,1)) //Have to change 500 da!! 
          break;
      }
      Motors_Brake(255,255);
    }
    //Parallelogram_Stop();
    Motors_Brake(255,255);
    LAPTOP.println("Reached bud 2");
    //Toggle_Wait();
    Actuate_Low(GRIPPER);
    
    //delay(2000);
    ///Take Parallelogram to lowest position
    parallelogram_count = 0;
    Parallelogram_Down(); 

//    while(!Parallelogram_Reached(1));
    delay(300);
    Parallelogram_Stop();
    Toggle_Wait();
    LAPTOP.println("Going to reverse");
    Parameters_Reset();


    Move_Parallelogram(FWD,1);
    Parallelogram_Stop();
    Parallelogram_Up();
    delay(400);
    Parallelogram_Stop();

    Move_Back(120,200);
    if(i==1){
      Run_For_Encoder_Count(9500);
    }
    else{
      Run_For_Encoder_Count(9000);
    }
    Motors_Brake(255,0);
    Parameters_Reset();
    motor2.Control(BCK,20);
    while(encoder_motor2<7000){ 
      Query_Launchpad();
      LAPTOP.println(encoder_motor2);
      if(encoder_motor2>4500){// ths is the place where i have made changes
        LAPTOP.println("Line Detected");
        break;
      }
    }  
    Motors_Brake(255,255);
    LAPTOP.println("Right turn completed");
    delay(200);

    motor2.Control(FWD,60);
    while(S2.Low()&&S3.Low());

    Motors_Brake(255,255);
    ///Distance to manual bot is very short, so lift parallelogram fully and then continue to linefollow.
    ///Take Parallelogram to top position   
    //    parallelogram_count = 0;
    //    Parallelogram_Up();
    //    while(!Parallelogram_Reached(1));
    Move_Parallelogram(FWD,1);
    Move_Forward(30,30);
    Parameters_Reset();
    while(1){
      LAPTOP.print("Linefollow Precision");
      LineFollow_Straight_Precision();
      if(( S4.High() && S2.High() )||( S3.High() && S1.High() )||(S3.High() && S2.High()))
        break;
    }
    Motors_Brake(255,255);
    LAPTOP.println("Meet the Manual Bot");
    Toggle_Wait();
    //COMMS AGAIN!! 
    Actuate_High(GRIPPER);

    Serial.println("Reached da");
    delay(2000);
    
  }

  Move_Back(255,255);

  delay(700);

  Motors_Brake(100,100);
  Move_Parallelogram(BCK,1);
  LAPTOP.println("Stage two completed");
  Toggle_Wait();
  Turret_Reset_char('a');
  while(1);
}

