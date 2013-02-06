//Autobot Stage Two Mirror Arena
//Sequential Code Structure

void Auto_Stage_Two_Mirror(){


  LAPTOP.println("Commencing Auto Stage Two Mirror");
  Serial_Wait();
  Parameters_Reset();

  //Till Junction
  //NOTE: In first two while(1) loops, need to add turret angle change
  while(1)
  {
    LAPTOP.print("LinefollowFirst");
    LineFollow();
    if( ( R2.High() && L1.High() )||( L2.High() && R1.High() )  )
      {
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
  Actuate_High(RIGHT_VG);
  delay(200);
  Serial.println("Dropped Third Leaf");
  Serial_Wait();
  
  //Line Follow till Bud One site
  Parameters_Reset();
  while(1){
     if(!LineFollowR1R2_Encoders(6000)) 
         break;
     }
  LineFollowR1R2_Brake();

  //Reverse and go to Tokyo

  Actuate_High(GRIPPER);
  Serial.println("Reverse");
  Serial_Wait();
  
  Parameters_Reset();
  Move_Back(200,255);
  Run_For_Encoder_Count(6000);
  
  Motors_Brake(0,255);
  Parameters_Reset();
  left_motor.Control(BCK,60);
  while(Encoders.left<6000) 
  { 
    Encoders.Read_Data();
    if(L2.High()&&Encoders.left>2000){
       Serial.println("This is Tokyo");
       break;
     }
  }  
  Motors_Brake(255,255);
  Serial.println("Left Turn Done");
  delay(200);
  
  left_motor.Control(FWD,60); //To get back on track
  while(!R1.High());
   
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
      Move_Back(255,125);
      delay(600); 
      Serial.println("Part One done"); 
      //Move_Back(255,255);
      
      //Should Encoder enabled linefollow be added? 
      
      Motors_Brake(255,0);
      right_motor.Control(BCK,60);
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
             if(!LineFollow_Encoders(500)) //Have to change 500 da! 
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

        Move_Back(255,170);
        
   Run_For_Encoder_Count(7500);
    
  Motors_Brake(0,255);
  Parameters_Reset();
  left_motor.Control(BCK,60);
  while(Encoders.left<6000)  //!R2.High()||
  { 
    Encoders.Read_Data();
    if(L2.High()&&Encoders.left>2000){
       Serial.println("line detected,,,,,");
       break;
     }
  }  
  Motors_Brake(255,255);
  Serial.println("left turn completed......");
  delay(500);
  left_motor.Control(FWD,60);
  while(!R1.High());
   
   
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
}



