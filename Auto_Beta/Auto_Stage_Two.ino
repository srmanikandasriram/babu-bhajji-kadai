
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
}

void LineFollow(){
  Serial_Print_Sensors();
  if( ( R2.High() && R1.High() && L1.High() )||( L2.High() && R1.High() && L1.High() )  ){  Move_Forward(SLOW,SLOW);}
  else if(  R2.Low() && R1.Low() && L2.High()  ){  Move_Forward(SLOWEST,NORMAL);motor1.Brake(255);}
  else if(  R2.Low() && R1.Low() && L1.High()  ){  Move_Forward(SLOW,NORMAL); }
  else if(  R2.High() && L1.Low() && L2.Low()  ){  Move_Forward(NORMAL,SLOWEST); motor2.Brake(255);}
  else if(  R1.High() && L1.Low() && L2.Low()  ){  Move_Forward(NORMAL,SLOW); } 
  delay(2);
}



int LineFollow_Brake(){
  Serial.print("LineFollowBrake");
  Serial_Print_Sensors();
  
  if(L1.High()){Move_Forward(0,60);motor1.Brake(150);}
  if(R1.High()){Move_Forward(60,0);motor2.Brake(150);}
  if(L2.High()){Move_Forward(0,30);motor1.Brake(255);}
  if(R2.High()){Move_Forward(30,0);motor2.Brake(255);}
  
  
    if( ( R2.High() && R1.High() )||( R1.High() && L1.High() )||(L1.High() && L2.High())){
      Motors_Brake(255,255);
      return 1;
    }
   return 0;
   
}

void LineFollow_Slow(){
  Serial_Print_Sensors();
  if( ( R2.High() && R1.High() && L1.High() )||( L2.High() && R1.High() && L1.High() )  ){  Move_Forward(SLOW,SLOW);}
  else if(  R2.Low() && R1.Low() && L2.High()  ){  Move_Forward(0,25);motor1.Brake(255);}
  else if(  R2.Low() && R1.Low() && L1.High()  ){  Move_Forward(10,25); }
  else if(  R2.High() && L1.Low() && L2.Low()  ){  Move_Forward(25,0); motor2.Brake(255);}
  else if(  R1.High() && L1.Low() && L2.Low()  ){  Move_Forward(25,10); } 
  delay(2);
}

int LineFollow_Encoders(long int encoder_value){
  Serial.print(Encoders.left); Serial.print("\t"); Serial.println(Encoders.right);
  Encoders.Read_Data();
  if(Encoders.left<encoder_value){
     LineFollow();
     return 1;
  }
  return 0;
}

int LineFollowR1R2_Encoders(long int encoder_value){
  Serial.print(Encoders.left); Serial.print("\t"); Serial.println(Encoders.right);
  Encoders.Read_Data();
  if(Encoders.left<encoder_value){
     LineFollowR1R2();
     return 1;
  }
  return 0;
}

int LineFollowL1L2_Encoders(long int encoder_value){
  Serial.print(Encoders.left); Serial.print("\t"); Serial.println(Encoders.right);
  Encoders.Read_Data();
  if(Encoders.left<encoder_value){
     LineFollowL1L2();
     return 1;
  }
  return 0;
}

int LineFollow12_Encoders(long int encoder_value){
   Serial.print(Encoders.left); Serial.print("\t"); Serial.println(Encoders.right);
   Encoders.Read_Data();
   if(Encoders.left<encoder_value){
     LineFollow12();
     return 1;
  }
  return 0;
}

//BECAUSE OF ABOVE CONSTRUCT, LINEFOLLOW12 IS NOT REQUIRED!!!
void LineFollow_Distance(float distance, int count, int tpwm, char p)
{
  int j=0;
  while(my<distance)
  {
    Serial_Print_Sensors();
    j++;
    if(j%10==0)
      Locationfinder(12,50,2688,10); 
    LineFollow(); 
    //Turret_Rotate(count,tpwm,p);
  }
  Motors_Brake(255,255);
  /*while(Turret.Get_Count()<count)
   {
     Motors_Brake(255,255);
     Turret_Rotate(count,tpwm,p);
     Serial.println(Turret.Get_Count());
   }*/
   //Turret_Stop();
}

void LineFollow12(){
      Serial_Print_Sensors();
    if( R1.High() || R2.High() ){Move_Forward(60,0);motor2.Brake(255);}
    if( L1.High() && L2.Low() ){Move_Forward(60,0);}
    else if(R1.Low() && R2.High()){Move_Forward(0,80);}
  }


void LineFollowL1L2(){
  Serial_Print_Sensors();
  
  if( R1.High() || R2.High() ){Move_Forward(60,0);right_motor.Brake(255);}
  if( L1.High() && L2.Low() ){Move_Forward(60,0);}
  else if(R1.Low() && R2.High()){Move_Forward(0,80);}
}

void LineFollowR1R2(){
  Serial_Print_Sensors();
  
  if(L1.High()||L2.High()){Move_Forward(0,60);left_motor.Brake(255);}
  if( R1.High() && R2.Low() ){Move_Forward(0,60);}
  else if( R1.Low() && R2.High() ){Move_Forward(80,0);}
}
      
void LineFollowL1L2_Distance(int distance,int count, int tpwm, char p )
{
  my=0;
  int j=0;
  while(my<distance)
  { 
    Serial_Print_Sensors();
    j++;
    if(j%10==0)
      Locationfinder(12,50,2688,10);
    LineFollowL1L2();
    //Turret_Rotate(count,tpwm,p);
  }
  Motors_Brake(255,255);
  /*while(Turret.Get_Count()<count)
  {
   Turret_Rotate(count,tpwm,p); 
  }*/
  //Turret_Stop();
}

void LineFollowR1R2_Distance(int distance,int count, int tpwm, char p )
{
  my=0;
  int j=0;
  while(my<distance)
  { 
    Serial.println("R1R2Distance!!");
    Serial_Print_Sensors();
    j++;
    if(j%10==0)
      Locationfinder(12,50,2688,10);
    LineFollowR1R2();
    //Turret_Rotate(count,tpwm,p);
  }
  Motors_Brake(255,255);
  /*while(Turret.Get_Count()<count)
  {
   Turret_Rotate(count,tpwm,p); 
  }*/
  //Turret_Stop();
}

void LineFollowL1L2_Brake()
{  
   while(1)
  {  
    Serial.println("L1L2Brake");
    Serial_Print_Sensors();
    if( R1.High()||R2.High()){Move_Forward(60,0);right_motor.Brake(255);}
    if( L1.High() && L2.Low() ){Move_Forward(80,0);right_motor.Brake(255);}
    else if( L1.Low() && L2.High() ){Move_Forward(0,80);left_motor.Brake(255);}  
     if( ( R2.High() && R1.High() && L1.High() )||( L2.High() && R1.High() && L1.High() )  )
        break;
    if(!Serial.available()){
      Serial.read();
      break;
    }
  }
  Motors_Brake(255,255);
}

void LineFollow12_Brake(){
   
   while(1)
  {  
    Serial.println("12Brake");
    Serial_Print_Sensors();
    if( R1.High()||R2.High()){Move_Forward(60,0);motor2.Brake(255);}
    if( L1.High() && L2.Low() ){Move_Forward(80,0);motor2.Brake(255);}
    else if( L1.Low() && L2.High() ){Move_Forward(0,80);motor1.Brake(255);}  
     if( ( R2.High() && R1.High() && L1.High() )||( L2.High() && R1.High() && L1.High() )  )
        break;
    if(!Serial.available()){
      Serial.read();
      break;
    }
  }
  Motors_Brake(255,255);
}

void LineFollowR1R2_Brake()
{  
   while(1)
  {  
    Serial.println("R1R2Brake");
    Serial_Print_Sensors();

    if(L1.High()||L2.High()){Move_Forward(0,60);left_motor.Brake(255);}
    if( R1.High() && R2.Low() ){Move_Forward(0,60);left_motor.Brake(255);}
    else if( R1.Low() && R2.High() ){Move_Forward(60,0);right_motor.Brake(255);}  
     if( ( R2.High() && L1.High())||( L2.High() && R1.High())|| ( R1.High() && L1.High() )  || (R2.High() && R1.High()) || (L2.High()&&L1.High()) )
        break;
  }
  Motors_Brake(255,255);
}
void Motor_Up()
{
  Parallelogram.Reset();
  riseup();
   int j= Parallelogram.Get_Count();
  while(1)
  {
    Serial.println(Parallelogram.Get_Count());
    if(j-Parallelogram.Get_Count()!=0)
      break;
  }
  stoprise();
  Parallelogram.Reset();
 
}
void Motor_Down()
{
   Parallelogram.Reset();
  risedown();
  int j= Parallelogram.Get_Count();
  while(1)
  {
    Serial.println(Parallelogram.Get_Count());
    if(j-Parallelogram.Get_Count()!=0)
      break;
  }
  stoprise();
  Parallelogram.Reset();
  
}

void Run_For_Encoder_Count(int encoder_count){
     while(1){
      Query_Launchpad();
      Serial.println(encoder_motor1); 
      if(encoder_motor1>encoder_count){
        break;
      }
     }
}

void Locationfinder(float wheelDiameter, float Width,int countsperrevolution,int period)
{
  float distancepercount;
  float radianspercount;
  int mperiod;
  int mpreviousleftcounts;
  int mpreviousrightcounts;  
  distancepercount = (PI * wheelDiameter)/ (float)countsperrevolution; // encoder resolution
  radianspercount = PI * (wheelDiameter / Width)/ countsperrevolution;
  Encoders.Read_Data();
  int leftcounts= Encoders.left;
  int rightcounts = Encoders.right;
  Serial.print(Encoders.left); Serial.print("\t");Serial.println(Encoders.right);   
  int deltaleft = (leftcounts-mpreviousleftcounts);
  int deltaright = (rightcounts-mpreviousrightcounts);
  float deltaDistance = (float)(deltaleft + deltaright) *0.5f * distancepercount;
  float deltaHeading = (float)(deltaright - deltaleft) * radianspercount;
  float deltaX = deltaDistance * (float )cos(mheading);
  float deltaY = deltaDistance * (float )sin(mheading);
  mx+=deltaX;
  my+=deltaY;
  mheading+=deltaHeading;
  if(mheading>PI)
    mheading-=TWO_PI;
  else if(mheading <= -PI)
    mheading+=TWO_PI;
  mpreviousleftcounts= leftcounts;
  
  mpreviousrightcounts = rightcounts;


  Serial.print("mx=    ");
  Serial.print(mx,6);
  Serial.print("      my=    ");
  Serial.println(my,6);
			
}
