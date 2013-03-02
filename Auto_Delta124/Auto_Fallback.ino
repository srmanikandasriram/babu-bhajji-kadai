
/** Auto Fallback **/

void Auto_Fallback(){
  while(!ps_complete){
    if(omit_leaf1&&omit_leaf2){
      if(!omit_leaf3){
        Transform_3L_B1[path_phase]();
        LAPTOP.println("Tansform 3L B1");
      }else if(bud_count == 0){
        Transform_NL_B1[path_phase]();
        LAPTOP.println("Tansform NL B1");
      }else{
        Transform_NL_B23[path_phase]();
        LAPTOP.println("Tansform NL B23");
      }
    }else{
      Transform_Fallback[path_phase]();
        LAPTOP.println("Tansform");
    }
    path_phase++;    
    Check_Abort();
  }
}

void To_Pick_LeavesF(){
  LCD.clear();
  LCD.print("To Pick Leaves");
  LAPTOP.println("Going to pick up the leaves.");
  Move_Forward(25, 25);
  
  encoder_turret = 0;
  turret_motor.Control(Check_Mirror(BCK,FWD),255);
  encoder_turret_target = Check_Mirror(TURRET_ANG1MF, TURRET_ANG1F);

  servo1.SetTargetAngle(2);
  servo2.SetTargetAngle(2);

  Parallelogram_Up();
  while(encoder_motor1 < distance_leaves_pickup && encoder_motor2 < distance_leaves_pickup){
    Query_Launchpad();
    Move_TurretF();
    Move_Servo();
    Serial_Print();
    Check_Abort();
    if( parallelogram_sensor.High())
      Parallelogram_Stop();
  }
}

void Pick_LeavesF(){

  LCD.clear();
  LCD.print("Pick Leaves");
  LAPTOP.println("Stopping to pick up leaves");
  int threshold = Check_Mirror(350, 150);
  while(analogRead(SHARP_SENSOR_PIN) < threshold){
    LAPTOP.print("SHARP Check running"); LAPTOP.println(analogRead(SHARP_SENSOR_PIN));
  
    Query_Launchpad();
    Move_Servo();
    Move_TurretF();
    Serial_Print();
    if( parallelogram_sensor.High())
      Parallelogram_Stop();
  }
  Motors_Brake(HARDBRAKE, HARDBRAKE);
  while(!parallelogram_sensor.High());
  Parallelogram_Stop();
  while(!Move_TurretF()){
    Move_Servo();
    Serial_Print();
  }

  if(omit_leaf1) 
    Actuate_High(LEFT_VG);
  if(omit_leaf3)
    Actuate_High(MIDDLE_VG);
  if(omit_leaf2)
    Actuate_High(RIGHT_VG);
  
  Actuate_High(V_PISTON);
  Parallelogram_Up();
  delay(50 + (omit_leaf1&&omit_leaf2&&!bud_count)*250);
  Parallelogram_Stop();
  delay(450 - (omit_leaf1&&omit_leaf2&&!bud_count)*250);
  Actuate_Low(V_PISTON);
  LAPTOP.println("Picked up leaves");
  Toggle_Wait(); // TMP
  delay(100);
}

void Move_Straight_FastF(){
  prevmillis = millis();
  LCD.clear();
  LCD.print("Move Straight");
  Move_Forward(190,255);

  if(!omit_leaf1 || !omit_leaf2 || omit_leaf3){
    encoder_turret = 0;
    turret_motor.Control(Check_Mirror(FWD,BCK), 255);
    encoder_turret_target = Check_Mirror(TURRET_ANG2MF, TURRET_ANG2F);
  }
    
  LAPTOP.println("Moving forward fast");
  LAPTOP.println(distance_straight_line);
  LAPTOP.println("Moving forward fast");

  servo1.SetTargetAngle(3);
  servo2.SetTargetAngle(3);

  base_pwm = 20;
  Move_Forward(base_pwm, base_pwm);
  while(base_pwm < 150){
    if(base_pwm<70)
      base_pwm +=2;
    else
      base_pwm += 5;
    Query_Launchpad();
    Move_TurretF();
    Move_Servo();

    Serial_Print();
    Check_Abort();
  }
  Move_Forward(255,255);

  while(encoder_motor1 < distance_straight_line && encoder_motor2 < distance_straight_line){
    Query_Launchpad();
    Move_TurretF();
    Move_Servo();
    Serial_Print();
  }  
}
/*
void Accelerate_BotF(){
  //Toggle_Wait(); //Double toggle_wait because it was generally being bypassed. 
  Parameters_Reset();
  pid_enable = true;
  base_pwm = minimum_pwm;
  Move_Forward(minimum_pwm, minimum_pwm);
  encoder_turret = 0;
  turret_motor.Control(Check_Mirror(FWD,BCK), 220);
  encoder_turret_target = Check_Mirror(TURRET_ANG2MF, TURRET_ANG2F);
  LAPTOP.println("Acceleration begun");
  servo1.SetTargetAngle(3);
  servo2.SetTargetAngle(3);
  while(base_pwm < maximum_pwm){
    LAPTOP.println(encoder_motor1);
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

  while(encoder_motor1 < (distances_fallback[path_phase]) && encoder_motor2 < (distances_fallback[path_phase])){
 //   while(encoder_motor1 < 3880 && encoder_motor2 < 3880){
   LAPTOP.println(encoder_motor1);
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
    LAPTOP.println(encoder_motor1);
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

  while(encoder_motor1 < (distances_fallback[path_phase]) && encoder_motor2 < (distances_fallback[path_phase])){
  

  //while(encoder_motor1 < 5800 && encoder_motor2 < 5800){
    LAPTOP.println(encoder_motor1);
    Query_Launchpad();
    Move_TurretF();
    Move_Servo();
    Serial_Print();
    Check_Abort();
    PID_Adjust();
  }
}

void Detect_Line(){
  turret_motor.Brake(0);
  LCD.clear();
  LCD.print("Detect Line");
  Motors_Brake(100, 100);
  delay(350);
  LAPTOP.println("Moving forward slightly");
  Move_Forward(20,20);
  while(S2.Low()&&S3.Low());
  LAPTOP.println("Line detected");
  
  Parameters_Reset();
  while(encoder_motor1 < distances_fallback[path_phase] && encoder_motor2 < distances_fallback[path_phase]){
    Query_Launchpad();
    Check_Abort();
    Move_TurretF();
  }
//  Motors_Brake(255,255);
//  delay(400);
}
*/
void Set_TurnF(float ang){
  setpoint = ang/360.0*encoder_count;
  cons_kp = 400.0/setpoint;
  distance_turn = setpoint;
  pid.SetTunings(cons_kp, cons_ki, cons_kd);
  pid.SetOutputLimits(0,255);  
}

void Turn_and_Align(int dir, int angle){
  LCD.clear();
  LCD.print("Soft Turn");
  LAPTOP.println("Going into soft turn");
//  Motors_Brake(255,255);
//  Toggle_Wait();
  Set_TurnF(angle);
  Parameters_Reset();
  if( dir == 1 || dir == 4 || dir == 3){
    pid_type = SOFT_TURN_PIDL;
  }else{
    pid_type = SOFT_TURN_PIDR;
  }

  while(encoder_motor1 < distance_turn && encoder_motor2 < distance_turn){
    line_detected = false;
    Query_Launchpad();
    Check_Abort();
    Move_TurretF();
    PID_Adjust();
    Serial_Print();
    if( line_detected)
      if(dir == 1 && distance_turn-encoder_motor1<1500)//check and decide
        break;
      else if(dir == 4 && encoder_motor1>500)//check and decide
        break;
      else if(dir == 3 && distance_turn-encoder_motor1<400)//check and decide
        break;
      else if(dir == 2 && distance_turn-encoder_motor2<1500)
        break;
    /*if(dir == 4 && encoder_motor1>distances_fallback[path_phase]-1500) { //check and decide
      line_detected = true;
      break;
    } */
     
  }
//  Motors_Brake(255,255);
//  delay(1000);
//  delay(100);//delay added to increase precision
  // code for correcting and aligning back on to the line
  LAPTOP.println("Aligned onto the line");
}

void First_LineFollow(){
  turret_motor.Brake(0);
  Motors_Brake(255,255);
  delay(800);
  motor1.Control(FWD,0);  
  Turn_and_Align(1, Check_Mirror(140,100));//changed in the morning
  Motors_Brake(255,255);
  delay(100);
  
  while(S1.Low() && S2.Low() && S3.Low() && S4.Low() ){
    if(line_detected){
      motor1.Brake(255);
      motor2.Control(FWD,20);
    }else{
      motor2.Brake(255);
      motor1.Control(FWD,20);
    }
  }  
  LCD.clear();
  LCD.print("LF to Line1");
  LAPTOP.println("LineFollow to 1st Latitude");
  Parameters_Reset();
/*  while(analogRead(SHARP_SENSOR_PIN)<400){
    LineFollow34();
    Move_Servo();
    Move_TurretF();
    /*
    if(flag_turret == 1 || turret_sensor.Low()) {
      turret_motor.Brake(255);
      flag_turret = 1;
    }
    if(( S4.High() && S2.High() )||( S1.High() && S3.High() ))
      break;
  }*/
  turret_motor.Control(Check_Mirror(FWD,BCK),255);
  while(1){
    LineFollow34_Slow();
    Move_Servo();
    Move_TurretF();
    /*if(flag_turret == 1 || turret_sensor.Low()) {
      turret_motor.Brake(255);
      flag_turret = 1;
    }*/
    if(( S4.High() && S2.High() )||( S1.High() && S3.High() ))
      break;
  }  
/*  while(!( S4.High() && S2.High() )||( S1.High() && S3.High() )){
    LineFollow34_Slow();
    Move_Servo();
    Move_TurretF();
    /*if(flag_turret == 1 || turret_sensor.Low()) {
      turret_motor.Brake(255);
      flag_turret = 1;
    }
    if(( S4.High() && S2.High() )||( S1.High() && S3.High() ))
      break;
  }*/  
}

void Drop_Two_Leaves(){
  LCD.clear();
  LCD.print("Drop 2 leaves");
  LAPTOP.println("Dropping first two leaves");
  Motors_Brake(255,255);
  //while(turret_sensor.High());
  //turret_motor.Brake(255);
/*  int temp = millis() - prevmillis;
  LCD.begin(16,2);
  LCD.clear();*/
//  delay(600);
//  LCD.write(temp);
//  Toggle_Wait();
  delay(250);
  Actuate_High(LEFT_VG);
  Actuate_High(RIGHT_VG);
  

}

void Latitude_To_Last_Leaf(){
  
  LCD.clear();
  LCD.print("To last leaf");
//  Turn_and_Align(2);
  Parameters_Reset();
  motor1.Brake(255);
  motor2.Control(FWD,125);
  delay(500);
  encoder_turret = 0;
  encoder_turret_target = Check_Mirror(TURRET_ANG3MF, TURRET_ANG3F);
  turret_motor.Control(Check_Mirror(BCK,FWD),255);
  servo1.Middle();
  servo2.Middle();
  while(S4.Low()&&S3.Low()&&S1.Low()&&S2.Low()){
    Serial_Print();
    Move_TurretF();
  }
  LAPTOP.println("Aligned to straight line");
  LAPTOP.println("Line following to next junction");
  while(1){
    LineFollow_Straight_Precision_slow(); //for latitude
    Serial_Print();
    Move_TurretF();
    if(( S4.High() && S3.High() && S2.High() )||( S1.High() && S2.High() && S3.High()  ))
      break;
  }

  Motors_Brake(255,255);
  turret_motor.Brake(0);
  delay(200);
  turret_motor.Control(Check_Mirror(BCK,FWD),255);
  motor1.Control(FWD,100);
  Turn_and_Align(1,110);
  while(S2.Low() && S3.Low() && S1.Low() && S4.Low()){
    Serial_Print();
    /*    if(encoder_turret_flag == 0 && Move_TurretF() == 1) {
      encoder_turret = 0; // reset turret
      encoder_turret_flag = 1;
      encoder_turret_target = 200;
    }
    if(encoder_turret_flag == 1 && encoder_turret>encoder_turret_target){      
      turret_motor.Brake(255);  
    }*/
    Move_TurretF();
    if(line_detected)
      motor2.Control(FWD,20);
    else
      motor1.Control(FWD,20);
  }

  LAPTOP.println("Moving to third leaf");
}
void To_Last_Leaf(){
  Parameters_Reset();

  int threshold_edge = Check_Mirror(490,350), threshold_mid = Check_Mirror(425, 430);
  while(analogRead(SHARP_SENSOR_PIN)<threshold_edge){
    Query_Launchpad();
    LineFollow12();
    Move_TurretF();
  }
  encoder_turret = 0;
  encoder_turret_target = 450;
  turret_motor.Control(Check_Mirror(FWD,BCK),255);
  if(mirror){
    while(analogRead(SHARP_SENSOR_PIN)>threshold_mid){
      Query_Launchpad();
      LineFollow12_Slow();
      Move_Turret_EncoderF();
    }
  }else{
    while(analogRead(SHARP_SENSOR_PIN)<threshold_mid){
      Query_Launchpad();
      LineFollow12_Slow();
      Move_Turret_EncoderF();
    }    
  }
  while(analogRead(SHARP_SENSOR_PIN)>threshold_edge){
    Query_Launchpad();
    LineFollow12_Slow();
    Move_Turret_EncoderF();  
  }
  Motors_Brake(255,255);
  Parallelogram_Down()
  delay((omit_leaf1&&omit_leaf2)*250);
  Parallelogram_Stop();
  delay(300-(omit_leaf1&&omit_leaf2)*250);
}

void Drop_Last_Leaf(){
  LCD.clear();
  LCD.print("Drop last leaf");
  LAPTOP.println("Dropping Third Leaf");
  Actuate_High(MIDDLE_VG);
//  delay(200); //latest change on 23rd feb delay was there before 400
 
}

void To_First_Bud(){
  servo1.Middle();
  servo2.Middle();
  if(omit_leaf1&&omit_leaf2){
    Parameters_Reset();
    while(!LineFollow12_Encoders(3000));
    Motors_Brake(255,255);
  }
  //Linefollowing centred on S2 and S1. Linefollowing with brake till bud junction
  
  encoder_turret_target = Check_Mirror(TURRET_ANG4MF, TURRET_ANG4F);
  turret_motor.Control(Check_Mirror(FWD,BCK),255);
 
  while(!LineFollow12()){
    Move_TurretF();
  }
  Motors_Brake(255,255);
  while(!Move_TurretF());
  
  /*
  int flag_turret = 1;
  while(1){
    if(flag_turret == 0 && turret_sensor.Low())
      flag_turret += 1;
    else if(flag_turret == 1 && turret_sensor.High())
      flag_turret += 1;
    else if(flag_turret == 2 && turret_sensor.Low())
      flag_turret += 1;
    else if(flag_turret == 3 && turret_sensor.High())
      flag_turret += 1;
    else if(flag_turret == 4 && turret_sensor.Low()) {
      flag_turret += 1;
      if(!mirror)
        turret_motor.Brake(255);
    } else if(flag_turret == 5 && turret_sensor.High() && mirror)
      flag_turret += 1;
    else if(flag_turret == 6 && turret_sensor.Low() && mirror) {
      flag_turret += 1;
      turret_motor.Brake(255);
    }
      
    if(LineFollow12())
      break;    
  }
  while(!LineFollow12_Slow()) {
    if(flag_turret == 0 && turret_sensor.Low())
      flag_turret += 1;
    else if(flag_turret == 1 && turret_sensor.High())
      flag_turret += 1;
    else if(flag_turret == 2 && turret_sensor.Low())
      flag_turret += 1;
    else if(flag_turret == 3 && turret_sensor.High())
      flag_turret += 1;
    else if(flag_turret == 4 && turret_sensor.Low()) {
      flag_turret += 1;
      if(!mirror)
        turret_motor.Brake(255);
    } else if(flag_turret == 5 && turret_sensor.High() && mirror)
      flag_turret += 1;
    else if(flag_turret == 6 && turret_sensor.Low() && mirror) {
      flag_turret += 1;
      turret_motor.Brake(255);
    }
  }
  while ((flag_turret <=4 && !mirror) || (flag_turret<=6 && mirror))
{
    
    if(flag_turret == 0 && turret_sensor.Low())
      flag_turret += 1;
    else if(flag_turret == 1 && turret_sensor.High())
      flag_turret += 1;
    else if(flag_turret == 2 && turret_sensor.Low())
      flag_turret += 1;
    else if(flag_turret == 3 && turret_sensor.High())
      flag_turret += 1;
    else if(flag_turret == 4 && turret_sensor.Low()) {
      flag_turret += 1;
      if(!mirror)
        turret_motor.Brake(255);
    } else if(flag_turret == 5 && turret_sensor.High() && mirror)
      flag_turret += 1;
    else if(flag_turret == 6 && turret_sensor.Low() && mirror) {
      flag_turret += 1;
      turret_motor.Brake(255);
      
    }
      
}*/

  //Pick up, reverse and Go to Tokyo

 while(parallelogram_sensor.Low()){
  Parallelogram_Up(); 
 }
 Parallelogram_Stop();
  Toggle_Wait();
  Actuate_Low(GRIPPER);                               // to pick up bud 1
  ///Take Parallelogram to lowest position
  
  Parallelogram_Down(); 
  delay(600+(omit_leaf1&&omit_leaf2)*250);
  Parallelogram_Stop();
  delay(200);
  
//  Toggle_Wait();
  LAPTOP.println("Reverse");
}

void Tokyo(){
  LCD.clear();
  LCD.print("To junction");
  ///Take Parallelogram to mid position before reverse
  
  Parallelogram_Up();                                        //to raise para after black tape, to avoid count while shaking
  while(!parallelogram_sensor.High());
  delay(500);
  Parallelogram_Stop();
  Parameters_Reset();
  Move_Back(80,50);
  Run_For_Encoder_Count(500);
  Move_Back(170,120);
  Run_For_Encoder_Count(700);
  Move_Back(255,255);
  Run_For_Encoder_Count(Check_Mirror(9350,6300));
  Motors_Brake(255,0);
  Parameters_Reset();
  motor2.Control(BCK,30);
  Turn_and_Align(2,95);
  Motors_Brake(255,255);
  delay(200);
  if(line_detected){
    while(S2.Low() && S3.Low()){
      motor2.Control(FWD,25);
    }
  }else{
    while(S3.Low() && S2.Low()){
      motor1.Control(FWD,20);
    }
  }
  Motors_Brake(255,255);
  delay(100);
  LAPTOP.println("This is Tokyo");
/*  while(encoder_motor2<9000){ 
    Query_Launchpad();
    if((S2.High() || S3.High() )&&encoder_motor2>2000)
      break;
  }*/  
//  Motors_Brake(255,255);
  LAPTOP.println("Left Turn Done");
//  delay(100);

//  motor2.Control(FWD,30); //To get back on track
//  while(S2.Low());
//  while(S3.Low());
//  Motors_Brake(255,255);
//  delay(100);
  ///Take Parallelogram to top position  
//  parallelogram_count = 0;
  Parallelogram_Up();

  //Line Follow to Manual Bot
  Parameters_Reset();  
  LAPTOP.println("Ready to linefollow");  
  while(LineFollow_Encoders(500,3)){
    Parallelogram_Tripped();
  }
      
    
  while(1){
    LAPTOP.print("Linefollow ONE");
    LineFollow_Straight_Fast();
    Parallelogram_Tripped();
    if((S3.High() && S1.High()) || (S4.High() && S2.High()) || (S3.High() && S2.High()))
      break;
  }
//  Move_Forward(40,40);  //To bypass junction
//  delay(200);
}
    
void To_Bud_Transfer(){
  LCD.clear();
  LCD.print("to Bud transfer");
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
      if(LineFollow_Encoders(5850,4))
        break;
    }else{
      LAPTOP.println("BUD TWO OR THREE");
      if(LineFollow_Encoders(1500,3))
        break;
    }
  }
  Motors_Brake(255,255);
//  while(!Parallelogram_Tripped());
  Parallelogram_Up();
  delay(75);
  Parallelogram_Stop();
  LAPTOP.println("Meet the Manual Bot");
}

void Wait_For_TSOP(){
  // Communication Code
//    Parallelogram_Stop();
  long int temp_millis = millis(), wait_time = 20;
  if(mirror) {
    while ( 1 ) {
      if( digitalRead(COMM_TSOP_M1) == LOW || digitalRead(COMM_TSOP_M2) == LOW ) { // if Comm is on
        if( millis() - temp_millis > wait_time) { // AND for a long time
          break; /// lets go !
        }
      }
      else
        temp_millis = millis();
    }
  } else {
    while ( 1 ) {
      if( digitalRead(COMM_TSOP_1) == LOW || digitalRead(COMM_TSOP_2) == LOW ) { // if Comm is on
        if( millis() - temp_millis > wait_time) { // AND for a long time
          break; /// lets go !
        }
      }
      else
        temp_millis = millis();
    }
  }
}

void Transfer_Bud(){
  LCD.clear();
  LCD.print("Transfering bud");
 
 Wait_For_TSOP();
//  while( !(digitalRead(COMM_TSOP_1) == LOW || digitalRead(COMM_TSOP_2) == LOW) );
 // Toggle_Wait();
  Actuate_High(GRIPPER);
  LAPTOP.println("Ready to go for two and three");
  delay(1000);
  bud_count++;
 // if( bud_count == 2)
//    path_phase -= 5;
}

void To_Curve2(){
  LCD.clear();
  LCD.print("To curve2");
  LAPTOP.println("Goint to Curve2"); 
  Parameters_Reset();                  
  Move_Back(90,150);
  if(bud_count == 2){
    Run_For_Encoder_Count(Check_Mirror(3800,2000));// to check
  }else{
    Run_For_Encoder_Count(Check_Mirror(3200,3200));
  }
  Parallelogram_Down();
  Motors_Brake(0,255);
  motor1.Control(BCK,255);
  Turn_and_Align(1,Check_Mirror(100,115));
  Motors_Brake(255,255);
  delay(200);
  if(line_detected){
    while(S2.Low() && S1.Low()){ 
      motor1.Control(FWD,25);
    }
  }else{
    while(S3.Low() && S4.Low()){
      motor2.Control(FWD,25);
    }
  }
  
  /*delay(600);
  while(S1.Low());
  while(S2.Low());
  //while(S3.Low());
  
  if (!mirror)
  {
  while(S3.Low());
    while(S4.Low());
  }
//  while(S4.Low()&&S3.Low()&&S1.Low()&&S2.Low());
  */
//  Motors_Brake(255,255);
//  delay(200);

}

void To_Next_Bud(){
  LCD.clear();
  LCD.print("To next bud");

  int local_flag = 0;
  Motors_Brake(255,255);
 // Parallelogram_Down();
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
    if(Trip_Switch(PARALLELOGRAM_TRIP_SWITCH_BOTTOM)){
      Parallelogram_Stop();   
      Parallelogram_Up();
      local_flag = 1;
      prevmillis = millis();
    }
    if( parallelogram_sensor.High() && local_flag)
      Parallelogram_Stop();      
    if( parallelogram_sensor.Low() && local_flag)
      Parallelogram_Up();  
    if(LineFollow_Encoders(9000,2))
      break;
  }


  while(!LineFollow_Curve_Precision()){
    if(Trip_Switch(PARALLELOGRAM_TRIP_SWITCH_BOTTOM) && !local_flag){
      Parallelogram_Stop();   
      Parallelogram_Up();
      prevmillis = millis();
      local_flag = 1;
    }
    if( parallelogram_sensor.High() && local_flag)
      Parallelogram_Stop();      
    if( parallelogram_sensor.Low() && local_flag)
      Parallelogram_Up(); 
  }
  Motors_Brake(255,255);
  Parallelogram_Up();
  while(!parallelogram_sensor.High());
  delay(50);
  Parallelogram_Stop();
  
  if(bud_count == 2){
    Parameters_Reset();
    while(1){
      if(LineFollow_Encoders(3000,1))  
        break;
    }
  }
  Motors_Brake(255,255);
  delay(200);
  if (mirror) {
  motor1.Control(FWD,20);
  while(S1.Low()||S2.Low());
  Motors_Brake(255,255);
  }
  LAPTOP.println("Reached next bud");

  Toggle_Wait();
  
  Actuate_Low(GRIPPER);
  
  Move_Parallelogram(BCK,1);
  Parallelogram_Down(); 
  delay(850);
  Parallelogram_Stop();
  //delay(800);
}

void Tokyo2(){
  LCD.clear();
  LCD.print("Tokyo2");

//  Toggle_Wait();
  LAPTOP.println("Going to reverse");
  Parameters_Reset();

  Parallelogram_Up();
  while(!parallelogram_sensor.High());  //to take the parallelogram up till the tape
  delay(300);
  Parallelogram_Stop();
  Move_Back(40,40);
  Run_For_Encoder_Count(1000);
  Parallelogram_Up();
  Move_Back(Check_Mirror(80,120), Check_Mirror(250,160));
  if(bud_count == 2){
    Run_For_Encoder_Count(Check_Mirror(15000,11900));
  }else{
    Run_For_Encoder_Count(Check_Mirror(13500,8500));
  }
  Motors_Brake(255,0);
  Parameters_Reset();
  motor2.Control(BCK,30);
  Turn_and_Align(2,95);
  Motors_Brake(255,255);
  delay(200);
  if(line_detected){
    while(S3.Low() && S4.Low()){
      motor2.Control(FWD,20);
      Parallelogram_Tripped();
    }
  }else{
    while(S2.Low() && S1.Low()){
      motor1.Control(FWD,20);
      Parallelogram_Tripped();
    }
  }
  Motors_Brake(255,255);
  delay(120);
/*
  //delay(400);
  int local_flag = 0;
  Parameters_Reset();
  while(encoder_motor2<9000){ 
    Query_Launchpad();
    LAPTOP.println(encoder_motor2);
    Parallelogram_Tripped();
    if(encoder_motor2>2000){//latest change on 23rd fwb earlier it was s3 or s4 now only s3
      if(S4.High())
        local_flag = 1;
      if(S3.High() && local_flag){ 
        LAPTOP.println("Line Detected");
        break;
     }
    }
  }  
  Motors_Brake(255,255);*/
 // while(!Parallelogram_Tripped);
Parallelogram_Stop();
  Wait_For_TSOP();
  Parallelogram_Up();

  LAPTOP.println("Right turn completed");


  motor2.Control(FWD,60);
  while(S3.Low()&&S4.Low()){
    Parallelogram_Tripped();
  }

  Motors_Brake(255,255);
  
//  Toggle_Wait();
  //parallelogram_count = 0;
  //Parallelogram_Up();
}

void The_End(){
  Move_Back(100,255);
  delay(700);
  Motors_Brake(100,100);
  Move_Parallelogram(BCK,1);
  LAPTOP.println("Stage two completed");
  
  Toggle_Wait();
  Move_Turret_Dir('a');
  Move_Turret_Dir('a');
  Move_Turret_Dir('a');
  Move_Turret_Dir('a');
  Move_Turret_Dir('c');
  Parallelogram_Reset();
}

int Move_TurretF(){
  if(encoder_turret>encoder_turret_target){
   if(turret_sensor.Low()){
     turret_motor.Brake(255);
     return 1;
   }
  }
  return 0;
}

int Move_Turret_EncoderF(){
  if(encoder_turret>encoder_turret_target){
    turret_motor.Brake(255);
    return 1;
  }
  return 0;
}

