
/** Auto Fallback **/

void Auto_Fallback(){
  LAPTOP.println("\n Falllback! ");
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
    } else {
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
  Motors_Brake(VSOFTBRAKE, VSOFTBRAKE);
  while(encoder_turret<TURRET_ANG1){
    Move_Servo();
  }
  turret_motor.Brake(0);
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
  turret_motor.Control(Check_Mirror(FWD,BCK), 220);
  actuation_phase++;
  LAPTOP.println("Acceleration begun");

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

  LAPTOP.println("Moving forward slightly");
  Move_Forward(20,20);
  while(S2.Low()&&S3.Low());
  LAPTOP.println("Line detected");
  pid_enable = false;
  Launchpad_Reset();
  encoder_motor1 = 0; encoder_motor2 = 0;
}

void Auto_Fallback_Begin(){
  Motors_Brake(HARDBRAKE, HARDBRAKE);
  LAPTOP.println("Aligned onto the line");
  fallback_begin = true;
}

void Soft_Turn1(){
  LAPTOP.println("Going into soft turn");
  motor1.pwm(150);
  motor2.Brake(HARDBRAKE);
  pid_type = SOFT_TURN_PID;
  pid_enable = true;
  Set_Turn1(distances_fallback[path_phase]);
  Launchpad_Reset();
  encoder_motor1 = 0; encoder_motor2 = 0;
}

void Set_Turn1(float ang){
  setpoint = ang/360.0*encoder_count;
  cons_kp = 255.0/setpoint;
  distances_fallback[path_phase] = setpoint;
  pid.SetTunings(cons_kp, cons_ki, cons_kd);
  pid.SetOutputLimits(0,255);  
}


void LineFollow_Fallback(){
  
  LAPTOP.println("Commencing Auto Fallback Stratergy. Left turn complete.");
  Toggle_Wait();
  Parameters_Reset();

  // Till Junction
  // NOTE: In first two while(1) loops, need to add turret angle change
  while(1){
    LineFollow_Straight();
    if(( S4.High() && S2.High() )||( S1.High() && S3.High() ))
      break;
  }
  Motors_Brake(255,255);
  delay(50);
  Toggle_Wait();
  Actuate_High(LEFT_VG);
  Actuate_High(RIGHT_VG);
  
  // Linefollow till fourth ring drop site. Drop third leaf. 
  Parameters_Reset();
  motor2.Control(FWD,100);
  delay(600);
  while(S4.Low()&&S3.Low()&&S1.Low()&&S2.Low());
  Motors_Brake(255,255);
  // Till Junction
  // NOTE: In first two while(1) loops, need to add turret angle change
  while(1){
    LineFollow_Straight();
    if(( S4.High() && S2.High() )||( S1.High() && S3.High() ))
      break;
  }
  Motors_Brake(255,255);
  delay(50);
  Toggle_Wait();
  motor1.Control(FWD,100);
  delay(600);
  while(S4.Low()&&S3.Low()&&S1.Low()&&S2.Low());
  Motors_Brake(255,255);
  encoder_turret = 0;
  turret_motor.Control(Check_Mirror(BCK,FWD),255);
  while(encoder_turret<TURRET_ANG2);
  turret_motor.Brake(255);

  while(1){
    Serial.print("Encoders");
    if(!LineFollow_Encoders(1250,1)) 
      break;
  }
  Motors_Brake(255,255);
  delay(500);
  Actuate_High(MIDDLE_VG);
  delay(400);
  LAPTOP.println("Dropped Third Leaf");
  servo1.Home();
  
  turret_motor.Control(FWD,255);
  while(encoder_turret<TURRET_ANG2);
  turret_motor.Brake(255);
  
  Toggle_Wait();  

  ///Take Parallelogram to lowest position
  parallelogram_count = 0;
  Parallelogram_Down(); 
  
  while(!Parallelogram_Reached(2));

  Serial.println("Parallelogram should be down by now");
  
  //Linefollowing centred on S2 and S1. Linefollowing with brake till bud junction
  Parameters_Reset();
  while(1){
    //Parallelogram_Reached(2);
    if(!LineFollow12_Encoders(5000)) //&& Parallelogram_Reached(2) && encoder_turret>TURRET_ANG5) 
      break;    
  }
  while(LineFollow_Curve_Precision());
  Motors_Brake(255,255);  
  //Pick up, reverse and Go to Tokyo
  
//  while(!Parallelogram_Reached(1));
  ///May have to move forward slightly here. Parallelogram can alternatively be lowerd early in the previous while(1) loop
  
  Actuate_High(GRIPPER);
  LAPTOP.println("Reverse");
  Toggle_Wait();
  
  ///Take Parallelogram to mid position before reverse
//  parallelogram_count = 0;
//  Parallelogram_Up(); 
//  while(!Parallelogram_Reached(1));
  Move_Parallelogram(FWD,1);
  
  Parameters_Reset();
  Move_Back(255,200);
  Run_For_Encoder_Count(8000);
  
  Motors_Brake(255,0);
  Parameters_Reset();
  motor2.Control(BCK,30);
  while(encoder_motor2<9000){ 
    Query_Launchpad();
    if(S2.High()&&encoder_motor2>2000){
       LAPTOP.println("This is Tokyo");
       break;
    }
  }  
  Motors_Brake(255,255);
  LAPTOP.println("Left Turn Done");
  delay(150);
  
  motor2.Control(FWD,40); //To get back on track
  while(!S2.High());
  
  ///Take Parallelogram to top position  
  parallelogram_count = 0;
  Parallelogram_Up();
  
  //Line Follow to Manual Bot
  Parameters_Reset();  
  LAPTOP.println("Ready to linefollow");  
  while(1){
    LAPTOP.print("Linefollow ONE");
    LineFollow_Straight();
    if((S3.High() && S1.High()) || (S4.High() && S2.High()) || (S3.High() && S2.High()))
      break;
    Parallelogram_Reached(1);
  }
  Move_Forward(40,40);  //To bypass junction
  delay(200);
  
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
  Toggle_Wait();
  
  //COMMS CODE COMES HERE DA!
    
  Actuate_Low(GRIPPER);
  LAPTOP.println("Ready to go for two and three");
 
  //Continue to pickup bud two and three
  for(int i = 0; i<2; i++){
    Parameters_Reset();                  
    Move_Back(125,225);
    Run_For_Encoder_Count(3500); 
    LAPTOP.println("Part One done"); 
    
    ///Take Parallelogram to lowest position

    //parallelogram_count = 0;
    //Parallelogram_Down();

      
    Motors_Brake(0,255);
    motor1.Control(BCK,40);
    delay(600);
    while(S4.Low()&&S3.Low()&&S1.Low()&&S2.Low());
    
    Motors_Brake(255,255);
    Move_Parallelogram(BCK,2);
    Move_Forward(30,30);
    
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
        if(!LineFollow_Encoders(500,1)) //Have to change 500 da!! 
          break;
      }
      Motors_Brake(255,255);
    }
    //Parallelogram_Stop();
    Motors_Brake(255,255);
    LAPTOP.println("Reached bud 2");
    Toggle_Wait();
    Actuate_High(GRIPPER);

    LAPTOP.println("Going to reverse");
    Parameters_Reset();
    

    Move_Parallelogram(FWD,1);
    Move_Back(170,255);
    if(i==1){
      Run_For_Encoder_Count(9000);
    }else{
      Run_For_Encoder_Count(8500);
    }
    Motors_Brake(255,0);
    Parameters_Reset();
    motor2.Control(BCK,40);
    while(encoder_motor2<9000){ 
      Query_Launchpad();
      LAPTOP.println(encoder_motor2);
      if(S4.High()&&encoder_motor2>2000){
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
    Actuate_Low(GRIPPER);
    Serial.println("Reached da");
  }
  
  Move_Back(255,255);

  delay(700);

  Motors_Brake(100,100);
  LAPTOP.println("Stage two completed");
  while(1);
}
