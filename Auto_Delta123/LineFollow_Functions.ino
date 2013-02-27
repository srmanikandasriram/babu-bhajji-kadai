
/** Line Follow Functions **/

void LineFollow_Straight(){
  LAPTOP.println("LineFollow Straight ");
  Serial_Print_Sensors();
  if(S2.High()){
    if(S1.High()){
      Move_Forward(0,60); 
    }else{
      Move_Forward(10,50); 
    }
  }else if(S1.High()){
    Move_Forward(0,50);
    motor1.Brake(255);
  }else if(S3.High()){
    if(S4.High()){
      Move_Forward(50,0);
    }else{
      Move_Forward(50,10);
    }  
  }else if(S4.High()){
    Move_Forward(50,0);
    motor2.Brake(255);
  }else{
    Move_Forward(50,50);
  }
}

void LineFollow_Straight_slow(){
  LAPTOP.println("LineFollow Straight ");
  Serial_Print_Sensors();
  if(S2.High()){
    if(S1.High()){
      Move_Forward(0,30); 
    }else{
      Move_Forward(5,20); 
    }
  }else if(S1.High()){
    Move_Forward(0,20);
    motor1.Brake(255);
  }else if(S3.High()){
    if(S4.High()){
      Move_Forward(20,0);
    }else{
      Move_Forward(20,5);
    }  
  }else if(S4.High()){
    Move_Forward(20,0);
    motor2.Brake(255);
  }else{
    Move_Forward(20,20);
  }
}

void LineFollow_Curve(){
  LAPTOP.println("LineFollow Curve");
  Serial_Print_Sensors();
  if(S2.High()){
    if(S1.High()){
      Move_Forward(0,50); 
    }else{
      Move_Forward(10,50); 
    }
  }else if(S1.High()){
    Move_Forward(0,50);
    motor1.Brake(255);
  }else if(S3.High()){
    if(S4.High()){
      Move_Forward(40,0);
    }else{
      Move_Forward(40,10);
    }  
  }else if(S4.High()){
    Move_Forward(40,0);
    motor2.Brake(255);
  }else{
    Move_Forward(40,50);
  }
}

void LineFollow_Curve2(){
  LAPTOP.println("LineFollow Curve2");
  Serial_Print_Sensors();
  if(S2.High()){
    if(S1.High()){
      Move_Forward(0,70); 
    }else{
      Move_Forward(10,70); 
    }
  }else if(S1.High()){
    Move_Forward(0,70);
    motor1.Brake(255);
  }else if(S3.High()){
    if(S4.High()){
      Move_Forward(40,0);
    }else{
      Move_Forward(40,10);
    }  
  }else if(S4.High()){
    Move_Forward(40,0);
    motor2.Brake(255);
  }else{
   Move_Forward(40,70);
  }
}

int LineFollow_Encoders(long int encoder_value, int curve_id){
  Query_Launchpad();
  LAPTOP.print(encoder_motor1);   LAPTOP.print("\t");   LAPTOP.println(encoder_motor2);

  if((encoder_motor1+encoder_motor2)/2 < encoder_value){
    if(curve_id == 1){
      LineFollow_Curve();
    }else if(curve_id == 2){
      LineFollow_Curve2();
    }else if(curve_id == 3){
      LineFollow_Straight_Precision();      
    }else{
      LineFollow_Straight();
    }
    return 0;
  }
  return 1;
}

void LineFollow12(){
  LAPTOP.println("LineFollow S1S2 ");
  static int local_flag = 0; 
  Serial_Print_Sensors();
  if(S2.High()){
    if(S1.High()){
      Move_Forward(25,35);
    }else{
      Move_Forward(25,0); 
      local_flag = 2;
    }
  }else if(S1.High()){
    Move_Forward(0,35); 
    local_flag = 1; 
  }else{
    if(local_flag == 2){
      Move_Forward(25,0);
      motor2.Brake(255);
      local_flag = 0;
    }else if(local_flag == 1){
      Move_Forward(0,35);
      motor1.Brake(255); 
      local_flag = 0; 
    }else{
      Move_Forward(25,0); 
      motor2.Brake(255);
    }
  } 
}

void LineFollow34(){
  LAPTOP.println("LineFollow S3S4 ");
  static int local_flag = 0; 
  Serial_Print_Sensors();
  if(S1.High()||S2.High()){
    Move_Forward(0,15);  
  }
  if(S3.High()){
    if(S4.High()){
      Move_Forward(30,20);
    }else{
      Move_Forward(0,20); 
      local_flag = 2;
    }
  }else if(S4.High()){
    Move_Forward(30,0); 
    local_flag = 1; 
  }else{
    if(local_flag == 2){
      Move_Forward(0,15);
      motor1.Brake(255);
      local_flag = 0;
    }else if(local_flag == 1){
      Move_Forward(25,0);
      motor2.Brake(255); 
      local_flag = 0; 
    }else{
      Move_Forward(0,20); 
      motor1.Brake(255);
    }
  } 
}

int LineFollow12_Encoders(long int encoder_value){
  Query_Launchpad();
  LAPTOP.print(encoder_motor1);   LAPTOP.print("\t");   LAPTOP.println(encoder_motor2);
  
  if((encoder_motor1+encoder_motor2)/2 < encoder_value){
    LineFollow12();
    return 0;
  }
  Motors_Brake(255,255);
  return 1;
}

int LineFollow34_Encoders(long int encoder_value){
  Query_Launchpad();
  LAPTOP.print(encoder_motor1);   LAPTOP.print("\t");   LAPTOP.println(encoder_motor2);
  if((encoder_motor1+encoder_motor2)/2 < encoder_value){
    LineFollow34();
    return 0;
  }
  Motors_Brake(255,255);
  return 1;
}

int LineFollow_Straight_Precision(){
  LAPTOP.println("LineFollow Straight Precision ");
  Serial_Print_Sensors();
    
  if(S2.High()){
    if(S1.High()){
      Move_Forward(0,55);
      motor1.Brake(255);   
    }else{
      Move_Forward(0,35); 
    }
  }else if(S1.High()){
    Move_Forward(0,35);
    motor1.Brake(255);
  }else if(S3.High()){
    if(S4.High()){
      Move_Forward(55,0);
      motor2.Brake(255);
    }else{
      Move_Forward(35,0);
    }  
  }else if(S4.High()){
    Move_Forward(35,0);
    motor2.Brake(255);
  }else{
    Move_Forward(35,35);
  }

  if(S2.High() && S3.High()){
    Motors_Brake(255,255);
    return 1;
  }
  return 0;
}

int LineFollow_Curve_Precision(){
  LAPTOP.println("LineFollow Curve Precision ");
  Serial_Print_Sensors();
  if(S2.High()){
    if(S1.High()){
      Move_Forward(0,30);
      motor1.Brake(255);
    }else{
      Move_Forward(0,30); 
    }
  }else if(S1.High()){
    Move_Forward(0,30);
    motor1.Brake(255);
  }else if(S3.High()){
    if(S4.High()){
      Move_Forward(20,0);
      motor2.Brake(255);
    }else{
      Move_Forward(20,0);
    }  
  }else if(S4.High()){
    Move_Forward(20,0);
    motor2.Brake(255);
  }else{
    Move_Forward(20,30);
  }
  if(S2.High() && S3.High()){
    Motors_Brake(255,255);
    return 1;
  }
  return 0;
}

void Run_For_Encoder_Count(int encoder_count){
  int i = 0; 
  while(1){
    if(bud_count)
       Parallelogram_Tripped();
    Query_Launchpad();
    LAPTOP.print(encoder_motor1);   LAPTOP.print("\t");   LAPTOP.println(encoder_motor2);
    if((encoder_motor1+encoder_motor2)/2 > encoder_count)
      break;
  }
  Motors_Brake(255,255);
}
