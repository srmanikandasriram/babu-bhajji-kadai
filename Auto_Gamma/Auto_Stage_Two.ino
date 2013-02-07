/*
float mheading=1.57;

int n=0,j;

void Auto_Stage_Two(){


  LAPTOP.println("Commencing Auto Stage Two");
  Serial_Wait();
  Parameters_Reset();
  
  //Till Junction
  //NOTE: In first two while(1) loops, need to add turret angle change
  while(1)
  {
    LineFollow();
    if( ( R2.High() && L1.High() )||( L2.High() && R1.High() )  ) {
        break;
      } 
  }
  Motors_Brake(255,255);
  delay(50);
  
  //Linefollow till fourth ring drop site. Drop third leaf. 
  Parameters_Reset();  
  while(1){
     if(!LineFollow_Encoders(2000)) //Need to change 2000!! IMPORTANT NUMBER!!!
         break;
     }
  Motors_Brake(255,255);
  delay(100);
  Actuate_High(LEFT_VG);
  delay(200);
  Serial.println("Dropped Third Leaf");
  Serial_Wait();
  
  //Linefollowing centred on L1 and L2. Linefollowing with brake till bud junction
   Parameters_Reset();
   while(1){
     if(!LineFollowL1L2_Encoders(6000)) 
         break;
     }
   LineFollowL1L2_Brake();
  
  
  
  //Reverse and Go to Tokyo
  
  Actuate_High(GRIPPER);
  Serial.println("Reverse");
  Serial_Wait();
  
  Parameters_Reset();
  Move_Back(255,200);
  Run_For_Encoder_Count(6000);
  
  Motors_Brake(255,0);
  Parameters_Reset();
  right_motor.Control(BCK,60);
  while(Encoders.right<6000) 
  { 
    Encoders.Read_Data();
    if(R2.High()&&Encoders.right>2000){
       Serial.println("This is Tokyo");
       break;
     }
  }  
  Motors_Brake(255,255);
  Serial.println("Left Turn Done");
  delay(200);
  
  right_motor.Control(FWD,60); //To get back on track
  while(!L1.High());
  
  
  //Line Follow to Manual Bot
  Parameters_Reset();  
  Serial.println("Ready to linefollow");  
  while(1){
    LAPTOP.print("Linefollow ONE");
    LineFollow();
    if( ( R1.High() && L1.High() )  || (R2.High() && R1.High()) || (L2.High()&&L1.High()) )
        break;
  }
  Move_Forward(40,40);  //To bypass junction
  delay(200);
  while(1){
    LAPTOP.print("Linefollow Slow");
    LineFollow_Slow();
    if( ( R2.High() && L1.High()) || (R1.High() && L2.High() )  )
       break;
  }
  Motors_Brake(255,255);
  Serial.println("Meet the Manual Bot");
  Serial_Wait();
  
                          //COMMS CODE COMES HERE DA! 
    
  Actuate_Low(GRIPPER);
  Serial.println("Ready to go for two and three");
  Serial_Wait();      
 
 
 //Continue to pickup bud two and three
   for(int i=0;i<2;i++){
    
      Parameters_Reset();                  
      Move_Back(125,225);
      delay(600); 
      Serial.println("Part One done"); 
      //Move_Back(255,255);
      
      //Should Encoder enabled linefollow be added? 
      
      Motors_Brake(0,255);
      left_motor.Control(BCK,60);
      delay(600);
      while(R2.Low()&&R1.Low()&&L2.Low()&&L1.Low());

      
      Parameters_Reset();
      while(1){
        if(!LineFollow_Encoders(9000))
          break;
      }
      
      while(1){
        if(LineFollow_Brake())
          break;
      }
        

      if(i==1)
      {
           Parameters_Reset();
           while(1){
            if(!LineFollow_Encoders(500)) //Have to change 500 da!! 
              break;
      }
      }

      Serial.println("Reached bud 2");
      Serial_Wait();
      Actuate_High(GRIPPER);
      //Motor_Up();
      Serial.println("Going to reverse");
      Serial_Wait();
      Parameters_Reset();

        Move_Back(170,255);
        
   Run_For_Encoder_Count(7500);
    
  Motors_Brake(255,0);
  Parameters_Reset();
  right_motor.Control(BCK,60);
  while(Encoders.right<6000)  
  { 
    Encoders.Read_Data();
    if(R2.High()&&Encoders.right>2000){
       Serial.println("Line Detected");
       break;
     }
  }  
  Motors_Brake(255,255);
  Serial.println("Right turn completed");
  delay(500);
  right_motor.Control(FWD,60);
  while(!L1.High());
   
      
  Serial.println("Linefollow");
  Parameters_Reset();
  while(1){
    LAPTOP.print("Linefollow Slow");
    LineFollow_Slow();
    if( ( R2.High() && L1.High()) || (R1.High() && L2.High() )  )
       break;
  }
  Motors_Brake(255,255);
  Serial.println("Meet the Manual Bot");
  Serial_Wait();
         //COMMS AGAIN!! 
     Actuate_Low(GRIPPER);
     Serial.println("Reached da");
     Serial_Wait();                  
  }
 
  
  
  //Phase 10: Finish
  Move_Back(255,255);
  delay(1000);
  Motors_Brake(255,255);
  Motor_Down();
  while(1);
}*/
