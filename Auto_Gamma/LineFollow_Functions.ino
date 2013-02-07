
/** Line Follow Functions **/

void LineFollow(){
  Serial_Print_Sensors();
  if(( S4.High() && S3.High() && S2.High() )||( S1.High() && S3.High() && S2.High() )){  Move_Forward(SLOW,SLOW);}
  else if(  S4.Low() && S3.Low() && S1.High()  ){  Move_Forward(SLOWEST,NORMAL);  motor1.Brake(255);}
  else if(  S4.Low() && S3.Low() && S2.High()  ){  Move_Forward(SLOW,NORMAL);}
  else if(  S4.High() && S2.Low() && S1.Low()  ){  Move_Forward(NORMAL,SLOWEST);  motor2.Brake(255);}
  else if(  S3.High() && S2.Low() && S1.Low()  ){  Move_Forward(NORMAL,SLOW);} 
  delay(2);
}

int LineFollow_Brake(){
  LAPTOP.print("LineFollowBrake");
  Serial_Print_Sensors();
  
  if( S2.High() ){  Move_Forward(0,60);  motor1.Brake(150);}
  if( S3.High() ){  Move_Forward(60,0);  motor2.Brake(150);}
  if( S1.High() ){  Move_Forward(0,30);  motor1.Brake(255);}
  if( S4.High() ){  Move_Forward(30,0);  motor2.Brake(255);}
  
  if(( S4.High() && S3.High() )||( S2.High() && S1.High() )){
      Motors_Brake(255,255);
      return 1;
  }
  return 0;
}

void LineFollow_Slow(){
  Serial_Print_Sensors();
  if(( S4.High() && S3.High() && S2.High() )||( S1.High() && S3.High() && S2.High() )){  Move_Forward(SLOW,SLOW);}
  else if(  S4.Low() && S3.Low() && S1.High()  ){  Move_Forward(0,25);  motor1.Brake(255);}
  else if(  S4.Low() && S3.Low() && S2.High()  ){  Move_Forward(10,25);}
  else if(  S4.High() && S2.Low() && S1.Low()  ){  Move_Forward(25,0);  motor2.Brake(255);}
  else if(  S3.High() && S2.Low() && S1.Low()  ){  Move_Forward(25,10);} 
  delay(2);
}

int LineFollow_Encoders(long int encoder_value){
  LAPTOP.print(encoder_motor1);
  LAPTOP.print("\t");
  LAPTOP.println(encoder_motor2);
  
  Query_Launchpad();
  if( encoder_motor1<encoder_value ){
     LineFollow();
     return 1;
  }
  return 0;
}

int LineFollow12_Encoders(long int encoder_value){
  LAPTOP.print(encoder_motor1);
  LAPTOP.print("\t");
  LAPTOP.println(encoder_motor2);
  Query_Launchpad();
  if( encoder_motor1<encoder_value ){
    LineFollow12();
    return 1;
  }
  return 0;
}

void LineFollow12(){
  Serial_Print_Sensors();
  if( S3.High() || S4.High() ){  Move_Forward(60,0);  motor2.Brake(255);}
  if( S2.High() && S1.Low() ){  Move_Forward(60,0);}
  else if( S2.Low() && S1.High() ){  Move_Forward(0,60);}
}

void LineFollow12_Brake(){
  while(1){  
    LAPTOP.println("12Brake");
    Serial_Print_Sensors();
    if( S3.High() || S4.High() ){  Move_Forward(60,0);  motor2.Brake(255);}
    if( S2.High() && S1.Low() ){  Move_Forward(80,0);  motor2.Brake(255);}
    else if( S2.Low() && S1.High() ){  Move_Forward(0,80);  motor1.Brake(255);}  
    if( ( S4.High() && S3.High())||( S2.High() && S3.High()))
      break;
  }
  Motors_Brake(255,255);
}

void Run_For_Encoder_Count(int encoder_count){
  int i = 0; 
  while(1){
    Query_Launchpad();
    LAPTOP.println(encoder_motor1); 
    if( encoder_motor1>encoder_count )
      break;
  }
}

/*
void LineFollowR1R2_Brake()
{  
   while(1)
  {  
    LAPTOP.println("R1R2Brake");
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
    LAPTOP.println(Parallelogram.Get_Count());
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
    LAPTOP.println(Parallelogram.Get_Count());
    if(j-Parallelogram.Get_Count()!=0)
      break;
  }
  stoprise();
  Parallelogram.Reset();
  
}
*/
/*
int LineFollowR1R2_Encoders(long int encoder_value){
  LAPTOP.print(Encoders.left); LAPTOP.print("\t"); LAPTOP.println(Encoders.right);
  Encoders.Read_Data();
  if(Encoders.left<encoder_value){
     LineFollowR1R2();
     return 1;
  }
  return 0;
}

int LineFollowL1L2_Encoders(long int encoder_value){
  LAPTOP.print(Encoders.left); LAPTOP.print("\t"); LAPTOP.println(Encoders.right);
  Encoders.Read_Data();
  if(Encoders.left<encoder_value){
     LineFollowL1L2();
     return 1;
  }
  return 0;
}
*/
/*
//BECAUSE OF ABOVE CONSTRUCT, LINEFOLLOW12 IS NOT REQUIRED!!!

void LineFollow_Distance(float distance, int count, int tpwm, char p){
  int j = 0;
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
  //while(Turret.Get_Count()<count)
  // {
   //  Motors_Brake(255,255);
   //  Turret_Rotate(count,tpwm,p);
   //  LAPTOP.println(Turret.Get_Count());
   //}
   //Turret_Stop();
}
*/
/*
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
  //while(Turret.Get_Count()<count)
  //{
  // Turret_Rotate(count,tpwm,p); 
  //}
  //Turret_Stop();
}

void LineFollowR1R2_Distance(int distance,int count, int tpwm, char p )
{
  my=0;
  int j=0;
  while(my<distance)
  { 
    LAPTOP.println("R1R2Distance!!");
    Serial_Print_Sensors();
    j++;
    if(j%10==0)
      Locationfinder(12,50,2688,10);
    LineFollowR1R2();
    //Turret_Rotate(count,tpwm,p);
  }
  Motors_Brake(255,255);
  //while(Turret.Get_Count()<count)
  //{
  // Turret_Rotate(count,tpwm,p); 
  //}
  //Turret_Stop();
}

void LineFollowL1L2_Brake()
{  
   while(1)
  {  
    LAPTOP.println("L1L2Brake");
    Serial_Print_Sensors();
    if( R1.High()||R2.High()){Move_Forward(60,0);right_motor.Brake(255);}
    if( L1.High() && L2.Low() ){Move_Forward(80,0);right_motor.Brake(255);}
    else if( L1.Low() && L2.High() ){Move_Forward(0,80);left_motor.Brake(255);}  
     if( ( R2.High() && R1.High() && L1.High() )||( L2.High() && R1.High() && L1.High() )  )
        break;
    if(!LAPTOP.available()){
      LAPTOP.read();
      break;
    }
  }
  Motors_Brake(255,255);
}
*/

