
/** Auto Fallback **/

void Auto_Fallback(){
  while(!ps_complete){
    Transform_Fallback[path_phase]();
    path_phase++;
    Check_Abort();
  }
}

void To_Pick_LeavesF(){
  LCD.clear();
  LCD.print("To Pick Leaves");
  LAPTOP.println("Going to pick up the leaves.");
  Move_Forward(15, 15);
  encoder_turret = 0;
  turret_motor.Control(Check_Mirror(BCK,FWD),255);
  encoder_turret_target = Check_Mirror(TURRET_ANG1MF, TURRET_ANG1F);
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

void Pick_LeavesF(){
  LCD.clear();
  LCD.print("Pick Leaves");
  LAPTOP.println("Stopping to pick up leaves");
  int threshold = Check_Mirror(350, 250);
  if(mirror) { // right sharp is being used // NEEDS TO BE CHANGED
  
    while(analogRead(SHARP_SENSOR_PIN) < threshold){
      LAPTOP.print("SHARP Check running"); LAPTOP.println(analogRead(SHARP_SENSOR_PIN));
    
      Query_Launchpad();
      Move_Servo();
      Move_TurretF();
      Serial_Print();
    }
  } else {
    
    while(analogRead(SHARP_SENSOR_PIN) < threshold){
      LAPTOP.print("SHARP Check running"); LAPTOP.println(analogRead(SHARP_SENSOR_PIN));
    
      Query_Launchpad();
      Move_Servo();
      Move_TurretF();
      Serial_Print();
    }
  }
  Motors_Brake(HARDBRAKE, HARDBRAKE);
  while(encoder_turret < encoder_turret_target){
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
  delay(1000);
  Actuate_Low(V_PISTON);
  LAPTOP.println("Picked up leaves");
  Toggle_Wait(); // TMP
  //delay(1000);
}

void Move_Straight_FastF(){
  LCD.clear();
  LCD.print("Move Straight");
  Parameters_Reset();
  Move_Forward(255,255);
  encoder_turret = 0;
  turret_motor.Control(Check_Mirror(FWD,BCK), 220);
  encoder_turret_target = Check_Mirror(TURRET_ANG2MF, TURRET_ANG2F);
  LAPTOP.println("Moving forward fast");
  servo1.SetTargetAngle(3);
  servo2.SetTargetAngle(3);

  while(encoder_motor1 < distances_fallback[path_phase] && encoder_motor2 < distances_fallback[path_phase]){
    Query_Launchpad();
    Move_TurretF();
    Move_Servo();
    Serial_Print();
  }  
}

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
  LCD.clear();
  LCD.print("Detect Line");
  Motors_Brake(100, 100);
  delay(350);
  LAPTOP.println("Moving forward slightly");
  Move_Forward(35,35);
  while(S2.Low()&&S3.Low()){
    Move_TurretF();
  }
  LAPTOP.println("Line detected");
  
  Parameters_Reset();
  while(encoder_motor1 < distances_fallback[path_phase] && encoder_motor2 < distances_fallback[path_phase]){
    Query_Launchpad();
    Check_Abort();
    Move_TurretF();
  }
  Motors_Brake(255,255);
  delay(400);
}

void Turn_and_Align(){
  LCD.clear();
  LCD.print("Soft Turn");
  LAPTOP.println("Going into soft turn");
  motor1.Control(FWD,100);
  motor2.Control(BCK,75);
  delay(200);
  while(S1.Low()&&S2.Low()&&S3.Low()&&S4.Low());
//  Motors_Brake(255,255);
//  delay(100);//delay added to increase precision
  // code for correcting and aligning back on to the line
  LAPTOP.println("Aligned onto the line");
}

void First_LineFollow(){
  LCD.clear();
  LCD.print("LF to Line1");
  LAPTOP.println("LineFollow to 1st Latitude");
  Parameters_Reset();
  int flag_turret = Check_Mirror(1, 0);
  turret_motor.Control(Check_Mirror(FWD,BCK),255);
  while(1){
    LineFollow34();
    Move_Servo();
    if(flag_turret == 1 || turret_sensor.Low()) {
      turret_motor.Brake(255);
      flag_turret = 1;
    }
    if(( S4.High() && S2.High() )||( S1.High() && S3.High() ))
      break;
  }
  
}

void Drop_Two_Leaves(){
  LCD.clear();
  LCD.print("Drop 2 leaves");
  LAPTOP.println("Dropping first two leaves");
  Motors_Brake(255,255);
  //while(turret_sensor.High());
  //turret_motor.Brake(255);
  
  Toggle_Wait();
  delay(200);
  
  Actuate_High(LEFT_VG);
  Actuate_High(RIGHT_VG);
  delay(300);
}

void To_Last_Leaf(){
  LCD.clear();
  LCD.print("To last leaf");

  Parameters_Reset();
  motor1.Brake(255);
  motor2.Control(FWD,50);
  delay(600);
  servo1.Middle();
  servo2.Middle();
  while(S4.Low()&&S3.Low()&&S1.Low()&&S2.Low());
  LAPTOP.println("Aligned to straight line");
  encoder_turret_target = Check_Mirror(TURRET_ANG3MF, TURRET_ANG3F);
  turret_motor.Control(Check_Mirror(BCK,FWD),255);
  LAPTOP.println("Line following to next junction");
  while(1){
    LineFollow_Straight();
    Move_TurretF();
    if(( S4.High() && S3.High() && S2.High() )||( S1.High() && S2.High() && S3.High()  ))
      break;
  }
  Serial_Print_Sensors();
  motor1.Control(FWD,30);//latest change on 23rd fwb before it was 30
  motor2.Brake(255);
  delay(600);
  while(S1.Low()&&S2.Low()&&S3.Low()){
    Move_TurretF();
  }
  Motors_Brake(255,255);
  delay(200);
  LAPTOP.println("Moving to third leaf");
  Parameters_Reset();
  int threshold = Check_Mirror(200, 350); // NEED TO CHANGE
  if(mirror) { // right sensor is being used
    while(analogRead(SHARP_SENSOR_PIN) < threshold){
      LAPTOP.print("SHARP VALUE : "); LAPTOP.println(analogRead(SHARP_SENSOR_PIN));
      Query_Launchpad();
      LineFollow12();
      Move_TurretF();
    }
  } else {
    while(analogRead(SHARP_SENSOR_PIN)< threshold){
      LAPTOP.print("SHARP VALUE : "); LAPTOP.println(analogRead(SHARP_SENSOR_PIN));
      Query_Launchpad();
      LineFollow12();
      Move_TurretF();
    }
  }
  Motors_Brake(255,255);
  //delay(200); // latest change on 23rd feb it was 500  
  Toggle_Wait();
}
void Drop_Last_Leaf(){
  LCD.clear();
  LCD.print("Drop last leaf");
  LAPTOP.println("Dropping Third Leaf");

  delay(200);
  Actuate_High(MIDDLE_VG);
  delay(200); //latest change on 23rd feb delay was there before 400
 
}

void To_First_Bud(){
  LCD.clear();
  LCD.print("to first bud");
  /*turret_motor.Control(Check_Mirror(FWD,BCK),255);
  while(turret_sensor.High());
  while(turret_sensor.Low());
  while(turret_sensor.High());
  while(turret_sensor.Low());
  while(turret_sensor.High());
  if(mirror) {
    while(turret_sensor.Low());
    while(turret_sensor.High());
  }  
  
  turret_motor.Brake(255);
  */
  //Linefollowing centred on S2 and S1. Linefollowing with brake till bud junction
  Parameters_Reset();
  turret_motor.Control(Check_Mirror(FWD,BCK),255); 
  int flag_turret = 0;
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
      
    if(LineFollow12_Encoders(2000))
      break;    
  }
  while(!LineFollow_Curve_Precision()) {
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
      
}
  Motors_Brake(255,255);  
  //Pick up, reverse and Go to Tokyo
  ///May have to move forward slightly here. Parallelogram can alternatively be lowerd early in the previous while(1) loop

  Actuate_Low(GRIPPER);                               // to pick up bud 1
  ///Take Parallelogram to lowest position
  
  Toggle_Wait();

  parallelogram_count = 0;
  Parallelogram_Down(); 
  delay(800);
  Parallelogram_Stop();
  //delay(800);
  
  //Toggle_Wait();
  LAPTOP.println("Reverse");
}

void To_Junction(){
  LCD.clear();
  LCD.print("To junction");
  ///Take Parallelogram to mid position before reverse
  Parallelogram_Up();                                        //to raise para after black tape, to avoid count while shaking
  while(parallelogram_sensor.Low());
  delay(200);
  Parallelogram_Stop();
  parallelogram_count = 0;
  Parameters_Reset();
  Move_Back(150,100);
  Run_For_Encoder_Count(Check_Mirror(8400,7100));
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
  delay(100);

  motor2.Control(FWD,30); //To get back on track
  while(S2.Low());
  while(S3.Low());
  Motors_Brake(255,255);
  delay(100);
  ///Take Parallelogram to top position  
  parallelogram_count = 0;
  Parallelogram_Up();

  //Line Follow to Manual Bot
  Parameters_Reset();  
  LAPTOP.println("Ready to linefollow");  
  while(1){
    LAPTOP.print("Linefollow ONE");
    LineFollow_Straight_fast1();
    Parallelogram_Tripped();
    if((S3.High() && S1.High()) || (S4.High() && S2.High()) || (S3.High() && S2.High()))
      break;
  }
  Move_Forward(40,40);  //To bypass junction
  delay(200);
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
      if(LineFollow_Encoders(2500,4))
       break;
    }else{
      LAPTOP.println("BUD TWO OR THREE");
      if(LineFollow_Encoders(3200,3))
       break;
    }
  }
  Motors_Brake(255,255);
  while(!Parallelogram_Tripped());
  LAPTOP.println("Meet the Manual Bot");
}

void Wait_For_TSOP(){
  // Communication Code
  long int temp_millis = millis();
  while ( 1 ) {
    if( (digitalRead(COMM_TSOP_1) == LOW || digitalRead(COMM_TSOP_2) == LOW) ) { // if Comm is on
      if( millis() - temp_millis > 0) { // AND for a long time
        break; /// lets go !
      }
    }
    else
      temp_millis = millis();
  }
}

void Transfer_Bud(){
  LCD.clear();
  LCD.print("Transfering bud");
  Wait_For_TSOP();
//  while( !(digitalRead(COMM_TSOP_1) == LOW || digitalRead(COMM_TSOP_2) == LOW) );
  //Toggle_Wait();
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
  Move_Back(40,180);
  if(bud_count == 2)
  {
    Run_For_Encoder_Count(Check_Mirror(3800,2000));// to check
  }
  else
  {
    Run_For_Encoder_Count(Check_Mirror(3800,2800));
  }
  Motors_Brake(0,255);
  if (!mirror)
    {
      motor1.Control(BCK,25);
    }
   else
     {
       motor1.Control(BCK,25);
     }
  
  delay(600);
  while(S1.Low());
  while(S2.Low());
  //while(S3.Low());
  
  if (!mirror)
  {
  while(S3.Low());
    while(S4.Low());
  }
//  while(S4.Low()&&S3.Low()&&S1.Low()&&S2.Low());

  Motors_Brake(255,255);
  delay(200);

}

void To_Next_Bud(){
  LCD.clear();
  LCD.print("To next bud");

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
    if( parallelogram_sensor.High() && local_flag)
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
    if( parallelogram_sensor.High() && local_flag)
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
    if( parallelogram_sensor.High() && local_flag) {
      Parallelogram_Stop();      
      break;
    }
  }
  
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
  delay(800);
  Parallelogram_Stop();
  //delay(800);
}

void Tokyo2(){
  LCD.clear();
  LCD.print("Tokyo2");

  //Toggle_Wait();
  LAPTOP.println("Going to reverse");
  Parameters_Reset();

  Parallelogram_Up();
  while(parallelogram_sensor.Low());  //to take the parallelogram up till the tape
  delay(100);
  Parallelogram_Stop();
  Move_Back(30,30);
  Run_For_Encoder_Count(1000);
  Parallelogram_Up();
  //delay(500);
  //Parallelogram_Stop();  
  if(bud_count == 2){
    Move_Back(Check_Mirror(80,60), Check_Mirror(250,180));
    Run_For_Encoder_Count(Check_Mirror(13000,10700));
  }else{
    Move_Back(Check_Mirror(80,60), Check_Mirror(210,180));
    Run_For_Encoder_Count(Check_Mirror(9500,8500));
  }
  Motors_Brake(255,0);
  motor2.Control(BCK,20);// latest change on 23rd feb before it was 20
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
  Motors_Brake(255,255);
  Wait_For_TSOP();
  LAPTOP.println("Right turn completed");


  motor2.Control(FWD,60);
  while(S3.Low()&&S4.Low()){
    Parallelogram_Tripped();
  }

  Motors_Brake(255,255);
  
  //Toggle_Wait();
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
  Parallelogram_Reset();
}

void Move_TurretF(){
  if( encoder_turret>encoder_turret_target )
    turret_motor.Brake(255);
}
