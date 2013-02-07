
/** Auto Stage Two **/

void Auto_Stage_Two_Universal(){

  LAPTOP.println("Commencing Auto Stage Two");
  Serial_Wait();
  Parameters_Reset();
  
  // Till Junction
  // NOTE: In first two while(1) loops, need to add turret angle change
  while(1){
    LineFollow();
    if(( S4.High() && S2.High() )||( S1.High() && S3.High() ))
      break;
  }
  Motors_Brake(255,255);
  delay(50);
  
  // Linefollow till fourth ring drop site. Drop third leaf. 
  Parameters_Reset();  
  while(1){
    if(!LineFollow_Encoders(2000)) //Need to change 2000!! IMPORTANT NUMBER!!!
      break;
  }
  Motors_Brake(255,255);
  delay(100);
  Actuate_High(Check_Mirror(RIGHT_VG,LEFT_VG));
  delay(200);
  LAPTOP.println("Dropped Third Leaf");
  Serial_Wait();
  
  motor2.Control(BCK,30);
  while(!S1.High());
  
  //Linefollowing centred on S2 and S1. Linefollowing with brake till bud junction
  Parameters_Reset();
  while(1){
    if(!LineFollow12_Encoders(6000)) 
      break;
  }
  LineFollow12_Brake();
      
  //Reverse and Go to Tokyo
  Actuate_High(GRIPPER);
  LAPTOP.println("Reverse");
  Serial_Wait();
  
  Parameters_Reset();
  Move_Back(255,200);
  Run_For_Encoder_Count(6000);
  
  Motors_Brake(255,0);
  Parameters_Reset();
  motor2.Control(BCK,60);
  while(encoder_motor2<6000){ 
    Query_Launchpad();
    if(S4.High()&&encoder_motor2>2000){
       LAPTOP.println("This is Tokyo");
       break;
    }
  }  
  Motors_Brake(255,255);
  LAPTOP.println("Left Turn Done");
  delay(200);
  
  motor2.Control(FWD,60); //To get back on track
  while(!S2.High());
  
  //Line Follow to Manual Bot
  Parameters_Reset();  
  LAPTOP.println("Ready to linefollow");  
  while(1){
    LAPTOP.print("Linefollow ONE");
    LineFollow();
    if(( S3.High() && S2.High() )||( S4.High() && S3.High() )||( S1.High()&&S2.High() ))
      break;
  }
  Move_Forward(40,40);  //To bypass junction
  delay(200);
  while(1){
    LAPTOP.print("Linefollow Slow");
    LineFollow_Slow();
    if(( S4.High() && S2.High() )||( S3.High() && S1.High() ))
      break;
  }
  Motors_Brake(255,255);
  LAPTOP.println("Meet the Manual Bot");
  Serial_Wait();
  
  //COMMS CODE COMES HERE DA!
    
  Actuate_Low(GRIPPER);
  LAPTOP.println("Ready to go for two and three");
  Serial_Wait();      
 
  //Continue to pickup bud two and three
  for(int i = 0; i<2; i++){
    Parameters_Reset();                  
    Move_Back(125,225);
    delay(600); 
    LAPTOP.println("Part One done"); 
    //Move_Back(255,255);
      
    //Should Encoder enabled linefollow be added? 
      
    Motors_Brake(0,255);
    motor1.Control(BCK,60);
    delay(600);
    while(S4.Low()&&S3.Low()&&S1.Low()&&S2.Low());

    Parameters_Reset();
    while(1){
      if(!LineFollow_Encoders(9000))
        break;
    }
      
    while(1){
      if(LineFollow_Brake())
        break;
    }
        
    if(i==1){
      Parameters_Reset();
      while(1){
        if(!LineFollow_Encoders(500)) //Have to change 500 da!! 
          break;
      }
      Motors_Brake(255,255);
    }

    LAPTOP.println("Reached bud 2");
    Serial_Wait();
    Actuate_High(GRIPPER);
    //Motor_Up();
    LAPTOP.println("Going to reverse");
    Serial_Wait();
    Parameters_Reset();

    Move_Back(170,255);
    Run_For_Encoder_Count(7500);
    Motors_Brake(255,0);
    Parameters_Reset();
    motor2.Control(BCK,60);
    while(encoder_motor2<7000){ 
      Query_Launchpad();
      LAPTOP.println(encoder_motor2);
      if(S4.High()||S3.High()){//&&encoder_motor2>2000){
        LAPTOP.println("Line Detected");
        break;
      }
    }  
    Motors_Brake(255,255);
    LAPTOP.println("Right turn completed");
    delay(500);
    motor2.Control(FWD,60);
    while(!S2.High());

    LAPTOP.println("Linefollow");
    Parameters_Reset();
    while(1){
      LAPTOP.print("Linefollow Slow");
      LineFollow_Slow();
      if(( S4.High() && S2.High() )||( S3.High() && S1.High() ))
        break;
    }
    Motors_Brake(255,255);
    LAPTOP.println("Meet the Manual Bot");
    Serial_Wait();
         //COMMS AGAIN!! 
    Actuate_Low(GRIPPER);
    Serial.println("Reached da");
    Serial_Wait();                  
  }
}
