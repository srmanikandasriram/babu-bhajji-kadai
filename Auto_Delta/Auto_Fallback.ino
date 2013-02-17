
/** Auto Fallback **/

void Auto_Fallback(){
  while(!ps_complete){
    Transform_Fallback[path_phase]();
    path_phase++;
    Check_Abort();
  }
}

void Pick_LeavesF(){
  LAPTOP.println("Going to pick up the leaves.");
  Move_Forward(15, 15);
  encoder_turret = 0;
  turret_motor.Control(Check_Mirror(BCK,FWD),255);
  encoder_turret_target = TURRET_ANG1F;
  servo1.SetTargetAngle(2);
  servo2.SetTargetAngle(2);
  
  while(encoder_motor1 < distances_fallback[path_phase] && encoder_motor2 < distances_fallback[path_phase]){
    Query_Launchpad();
    Move_TurretF();
    Move_Servo();
    Serial_Print();
    Check_Abort();
  }
}

void Accelerate_BotF(){
  LAPTOP.println("Stopping to pick up leaves");
  while(analogRead(SHARP_SENSOR_PIN)<350){
    LAPTOP.print("SHARP Check running"); LAPTOP.println(analogRead(SHARP_SENSOR_PIN));
    
    Query_Launchpad();
    Move_Servo();
    Move_TurretF();
    Serial_Print();
  }
  Motors_Brake(HARDBRAKE, HARDBRAKE);
  while(encoder_turret < encoder_turret_target){
    Move_Servo();
    Serial_Print();
  }
  turret_motor.Brake(255);

  Actuate_Low(LEFT_VG);
  Actuate_Low(RIGHT_VG);
  Actuate_High(V_PISTON);
  delay(1000);
  Actuate_Low(V_PISTON);
  LAPTOP.println("Picked up leaves");
  Toggle_Wait();
  //Toggle_Wait(); //Double toggle_wait because it was generally being bypassed. 
  Parameters_Reset();
  pid_enable = true;
  base_pwm = minimum_pwm;
  Move_Forward(minimum_pwm, minimum_pwm);
  encoder_turret = 0;
  turret_motor.Control(Check_Mirror(FWD,BCK), 220);
  encoder_turret_target = TURRET_ANG2F;
  LAPTOP.println("Acceleration begun");
  servo1.SetTargetAngle(3);
  servo2.SetTargetAngle(3);
  while(base_pwm < maximum_pwm){
    base_pwm += acceleration;
    if(base_pwm == 36){
      LAPTOP.println("\n PID begun \n");    /// indicate start of PID
      encoder_motor1 = encoder_motor2;
      pid.SetOutputLimits(-35,35);
    }
    if(base_pwm>35){
      PID_Adjust();
    }
    Query_Launchpad();
    Move_TurretF();
    Move_Servo();
    delay(acceleration_delay);
    Serial_Print();
    Check_Abort();
  }

  LAPTOP.println("Acceleration completed.");
  cons_kp = 1.5;
  pid.SetTunings(cons_kp,cons_ki,cons_kd);
  pid.SetOutputLimits(-45,45);

  while(encoder_motor1 < distances_fallback[path_phase] && encoder_motor2 < distances_fallback[path_phase]){
    Query_Launchpad();
    Move_TurretF();
    Move_Servo();
    Serial_Print();
    Check_Abort();
    PID_Adjust();
  }
}

void Decelerate_BotF(){

  LAPTOP.println("Deceleration begun!");

  while(base_pwm > slowdown_pwm){
    base_pwm -= deceleration;
    Query_Launchpad();
    PID_Adjust();
    delay(deceleration_delay);
    Move_TurretF();
    Move_Servo();
    Serial_Print();
    Check_Abort();
  }

  LAPTOP.println("Deceleration done");
  base_pwm = slowdown_pwm;
  pid.SetOutputLimits(-35,35);

  while(encoder_motor1 < distances_fallback[path_phase] && encoder_motor2 < distances_fallback[path_phase]){
    Query_Launchpad();
    Move_TurretF();
    Move_Servo();
    Serial_Print();
    Check_Abort();
    PID_Adjust();
  }
}

void Detect_Line(){
  Motors_Brake(HARDBRAKE, HARDBRAKE);
  LAPTOP.println("Waiting for turret to turn");
  while(encoder_turret < encoder_turret_target);
  turret_motor.Brake(0);
  delay(400);
  LAPTOP.println("Moving forward slightly");
  Move_Forward(20,20);
  while(S2.Low()&&S3.Low());
  LAPTOP.println("Line detected");
  delay(100);
}

void Turn_and_Align(){
  LAPTOP.println("Going into soft turn");
  motor1.pwm(30); //Changed from 40 to 30 because of overshoot on correction
  motor2.Brake(HARDBRAKE);
  delay(500);
  while(S1.Low()&&S2.Low()&&S3.Low()&&S4.Low());
  Motors_Brake(255,255);
  delay(500);//delay added to increase precision
  // code for correcting and aligning back on to the line
  LAPTOP.println("Aligned onto the line");
}

void First_LineFollow(){
  LAPTOP.println("LineFollow to 1st Latitude");
  Parameters_Reset();
  
  turret_motor.Control(Check_Mirror(FWD,BCK),255);
  while(1){
    LineFollow34();
    Move_Servo();
    if(turret_sensor.Low())
      turret_motor.Brake(255);
    if(( S4.High() && S2.High() )||( S1.High() && S3.High() ))
      break;
  }
}

void Drop_Two_Leaves(){
  LAPTOP.println("Dropping first two leaves");
  Motors_Brake(255,255);
  while(turret_sensor.High());
  turret_motor.Brake(255);
  Toggle_Wait();
  delay(500);
  
  Actuate_High(LEFT_VG);
  Actuate_High(RIGHT_VG);
  delay(1000);
}

void To_Last_Leaf(){
  Parameters_Reset();
  motor1.Brake(255);
  motor2.Control(FWD,50);
  delay(600);
  servo1.Middle();
  servo2.Middle();
  while(S4.Low()&&S3.Low()&&S1.Low()&&S2.Low());
  LAPTOP.println("Aligned to straight line");
  encoder_turret_target = TURRET_ANG3F;
  turret_motor.Control(Check_Mirror(BCK,FWD),255);
  LAPTOP.println("Line following to next junction");
  while(1){
    LineFollow_Straight();
    Move_TurretF();
    if(( S4.High() && S3.High() && S2.High() )||( S1.High() && S2.High() && S3.High()  ))
      break;
  }
  Serial_Print_Sensors();
  motor1.Control(FWD,30);
  motor2.Brake(255);
  delay(600);
  while(S1.Low()&&S2.Low()&&S3.Low()){
    Move_TurretF();
  }
  Motors_Brake(255,255);
  delay(1000);
  LAPTOP.println("Moving to third leaf");
  Parameters_Reset();
  while(analogRead(SHARP_SENSOR_PIN)<300){
    LAPTOP.print("SHARP VALUE : "); LAPTOP.println(analogRead(SHARP_SENSOR_PIN));
    Query_Launchpad();
    LineFollow12();
    Move_TurretF();
  }
  Motors_Brake(255,255);
  Toggle_Wait();
}
void Drop_Last_Leaf(){
  LAPTOP.println("Dropping Third Leaf");

  delay(500);
  Actuate_High(MIDDLE_VG);
  delay(400);
 
}

void To_First_Bud(){
  turret_motor.Control(FWD,255);
  while(turret_sensor.High());
  while(turret_sensor.Low());
  while(turret_sensor.High());  
  turret_motor.Brake(255);
  //Linefollowing centred on S2 and S1. Linefollowing with brake till bud junction
  Parameters_Reset();
  while(1){
    if(LineFollow12_Encoders(2000))
      break;    
  }
  while(!LineFollow_Curve_Precision());
  Motors_Brake(255,255);  
  //Pick up, reverse and Go to Tokyo
  ///May have to move forward slightly here. Parallelogram can alternatively be lowerd early in the previous while(1) loop

  Actuate_Low(GRIPPER);                               // to pick up bud 1
  ///Take Parallelogram to lowest position
  Toggle_Wait();

  parallelogram_count = 0;
  Parallelogram_Down(); 
  delay(300);
  Parallelogram_Stop();
  
  //Toggle_Wait();
  LAPTOP.println("Reverse");
}

void To_Junction(){
  ///Take Parallelogram to mid position before reverse
  Parallelogram_Up();                                        //to raise para after black tape, to avoid count while shaking
  delay(500);
  Parallelogram_Stop();
  parallelogram_count = 0;
  Parameters_Reset();
  Move_Back(150,100);
  Run_For_Encoder_Count(8400);

  Motors_Brake(255,0);
  Parameters_Reset();
  motor2.Control(BCK,30);
  LAPTOP.println("This is Tokyo");
  while(encoder_motor2<9000){ 
    Query_Launchpad();
    if((S2.High() || S3.High() )&&encoder_motor2>2000)
      break;
  }  
  Motors_Brake(255,255);
  LAPTOP.println("Left Turn Done");
  delay(150);

  motor2.Control(FWD,30); //To get back on track
  while(!S2.High());

  ///Take Parallelogram to top position  
  parallelogram_count = 0;
  Parallelogram_Up();

  //Line Follow to Manual Bot
  Parameters_Reset();  
  LAPTOP.println("Ready to linefollow");  
  while(1){
    LAPTOP.print("Linefollow ONE");
    LineFollow_Straight_Precision();
    Parallelogram_Tripped();
    if((S3.High() && S1.High()) || (S4.High() && S2.High()) || (S3.High() && S2.High()))
      break;
  }
  Move_Forward(40,40);  //To bypass junction
  delay(200);
}

void To_Bud_Transfer(){
  /*
  // Old LineFollow
  while(1){
    LAPTOP.print("Linefollow Precision");
    LineFollow_Straight_Precision();
    Parallelogram_Reached(1);
    if(( S3.High() && S1.High() )||( S4.High() && S2.High())||(S3.High() && S2.High() ))
      break;
  }*/
  Parameters_Reset();
  while(1){
    Parallelogram_Tripped();
    
    if(bud_count==0){
      LAPTOP.println("BUD ONE");
      if(LineFollow_Encoders(5000,3))
       break;
    }else{
      LAPTOP.println("BUD TWO OR THREE");
      if(LineFollow_Encoders(3500,3))
       break;
    }
  }
  Motors_Brake(255,255);
  while(!Parallelogram_Tripped());
  LAPTOP.println("Meet the Manual Bot");
}

void Transfer_Bud(){
  //COMMS CODE COMES HERE DA!

  Toggle_Wait();  
  Actuate_High(GRIPPER);

  LAPTOP.println("Ready to go for two and three");
  delay(1000);
  bud_count++;
}

void To_Curve2(){
  LAPTOP.println("Goint to Curve2"); 
  Parameters_Reset();                  
  Move_Back(80,240);
  if(bud_count == 1){
    Run_For_Encoder_Count(3500); 
  }else{
    Run_For_Encoder_Count(2500); 
  }

  Motors_Brake(0,255);
  motor1.Control(BCK,20);
  delay(600);
  while(S4.Low()&&S3.Low()&&S1.Low()&&S2.Low());

  Motors_Brake(255,255);

}

void To_Next_Bud(){
  int local_flag = 0;
  Motors_Brake(255,255);
  Parallelogram_Down();
  /*
  delay(500);
  Parallelogram_Stop();
  Toggle_Wait();
  parallelogram_count = 0;
  Parallelogram_Down();
  */

  Parameters_Reset();
  prevmillis = millis();
  while(1){
    if(digitalRead(PARALLELOGRAM_TRIP_SWITCH_BOTTOM)){
      Parallelogram_Stop();   
      Parallelogram_Up();
      local_flag = 1;
      prevmillis = millis();
    }
    if(millis() - prevmillis >= 150*bud_count && local_flag)
      Parallelogram_Stop(); 
    if(LineFollow_Encoders(9000,2))
      break;
  }


  while(!LineFollow_Curve_Precision()){
    if(digitalRead(PARALLELOGRAM_TRIP_SWITCH_BOTTOM) && !local_flag){
      Parallelogram_Stop();   
      Parallelogram_Up();
      prevmillis = millis();
      local_flag = 1;
    }
    if(millis() - prevmillis >= 150*bud_count && local_flag)
      Parallelogram_Stop();      
  }
  Motors_Brake(255,255);
   while(1){
    if(digitalRead(PARALLELOGRAM_TRIP_SWITCH_BOTTOM) && !local_flag){
      Parallelogram_Stop();   
      Parallelogram_Up();
      prevmillis = millis();
      local_flag = 1;
    }
    if(millis() - prevmillis >= 150*bud_count && local_flag) {
      Parallelogram_Stop();      
      break;
    }
  }
  Toggle_Wait();
   
  if(bud_count == 2){
    Parameters_Reset();
    while(1){
      if(LineFollow_Encoders(3000,1))  
        break;
    }
  }
  Motors_Brake(255,255);
  
  LAPTOP.println("Reached next bud");

  Toggle_Wait();
  Actuate_Low(GRIPPER);
  
  Move_Parallelogram(BCK,1);
  Parallelogram_Down(); 
  delay(300);
  Parallelogram_Stop();
}

void Tokyo2(){
  Toggle_Wait();
  LAPTOP.println("Going to reverse");
  Parameters_Reset();

  Parallelogram_Up();
  delay(100);
  Parallelogram_Stop();
  Move_Back(30,30);
  Run_For_Encoder_Count(1000);
  Parallelogram_Up();
  delay(500);
  Parallelogram_Stop();  

  Move_Back(80,160);
  if(bud_count == 2){
    Run_For_Encoder_Count(13500);
  }else{
    Run_For_Encoder_Count(10500);
  }
  Motors_Brake(255,0);
  motor2.Control(BCK,20);
  Parameters_Reset();
  while(encoder_motor2<9000){ 
    Query_Launchpad();
    LAPTOP.println(encoder_motor2);
    if((S4.High()||S3.High())&&encoder_motor2>2000){
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
  Toggle_Wait();
  parallelogram_count = 0;
  Parallelogram_Up();
}

void The_End(){
  Move_Back(100,255);
  delay(700);
  Motors_Brake(100,100);
  Move_Parallelogram(BCK,1);
  LAPTOP.println("Stage two completed");
  Toggle_Wait();
  Move_Turret_Dir('a');
  Parallelogram_Reset();
}

void Move_TurretF(){
  if(encoder_turret > encoder_turret_target)
    turret_motor.Brake(255);
}
