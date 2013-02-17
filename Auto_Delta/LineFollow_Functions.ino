
/** Line Follow Functions **/

void LineFollow_Straight(){
  LAPTOP.println("LineFollow Straight ");
  Serial_Print_Sensors();
  if(S2.High()){
    if(S1.High()){
      Move_Forward(0,LINEFOLLOW_STRAIGHT_HIGH); 
    }else{
      Move_Forward(LINEFOLLOW_STRAIGHT_LOW,LINEFOLLOW_STRAIGHT_MODERATE); 
    }
  }else if(S1.High()){
    Move_Forward(0,LINEFOLLOW_STRAIGHT_MODERATE);
    motor1.Brake(255);
  }else if(S3.High()){
    if(S4.High()){
      Move_Forward(LINEFOLLOW_STRAIGHT_MODERATE,0);
    }else{
      Move_Forward(LINEFOLLOW_STRAIGHT_MODERATE,LINEFOLLOW_STRAIGHT_LOW);
    }  
  }else if(S4.High()){
    Move_Forward(LINEFOLLOW_STRAIGHT_MODERATE,0);
    motor2.Brake(255);
  }else{
    Move_Forward(LINEFOLLOW_STRAIGHT_MODERATE,LINEFOLLOW_STRAIGHT_MODERATE);
  }
}

void LineFollow_Curve(){
  LAPTOP.println("LineFollow Curve");
  Serial_Print_Sensors();
  if(S2.High()){
    if(S1.High()){
      Move_Forward(0,LINEFOLLOW_CURVE_HIGH); 
    }else{
      Move_Forward(LINEFOLLOW_CURVE_LOW,LINEFOLLOW_CURVE_HIGH); 
    }
  }else if(S1.High()){
    Move_Forward(0,LINEFOLLOW_CURVE_HIGH);
    motor1.Brake(255);
  }else if(S3.High()){
    if(S4.High()){
      Move_Forward(LINEFOLLOW_CURVE_MODERATE,0);
    }else{
      Move_Forward(LINEFOLLOW_CURVE_MODERATE,LINEFOLLOW_CURVE_LOW);
    }  
  }else if(S4.High()){
    Move_Forward(LINEFOLLOW_CURVE_MODERATE,0);
    motor2.Brake(255);
  }else{
    Move_Forward(LINEFOLLOW_CURVE_MODERATE,LINEFOLLOW_CURVE_HIGH);
  }
}

void LineFollow_Curve2(){
  LAPTOP.println("LineFollow Curve2");
  Serial_Print_Sensors();
  if(S2.High()){
    if(S1.High()){
      Move_Forward(0,LINEFOLLOW_CURVE2_HIGH); 
    }else{
      Move_Forward(LINEFOLLOW_CURVE2_LOW,LINEFOLLOW_CURVE2_HIGH); 
    }
  }else if(S1.High()){
    Move_Forward(0,LINEFOLLOW_CURVE2_HIGH);
    motor1.Brake(255);
  }else if(S3.High()){
    if(S4.High()){
      Move_Forward(LINEFOLLOW_CURVE2_MODERATE,0);
    }else{
      Move_Forward(LINEFOLLOW_CURVE2_MODERATE,LINEFOLLOW_CURVE2_LOW);
    }  
  }else if(S4.High()){
    Move_Forward(LINEFOLLOW_CURVE2_MODERATE,0);
    motor2.Brake(255);
  }else{
   Move_Forward(LINEFOLLOW_CURVE2_MODERATE,LINEFOLLOW_CURVE2_HIGH);
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
  if(S3.High()||S4.High()){
    Move_Forward(LINEFOLLOW12_34CORRECTION,0);  
  }

  if(S2.High()){
    if(S1.High()){
      Move_Forward(LINEFOLLOW12_INNER_MOTOR,LINEFOLLOW12_OUTER_MOTOR);
    }else{
      Move_Forward(LINEFOLLOW12_INNER_MOTOR,0); 
      local_flag = 2;
    }
  }else if(S1.High()){
    Move_Forward(0,LINEFOLLOW12_OUTER_MOTOR); 
    local_flag = 1; 
  }else{
    if(local_flag == 2){
      Move_Forward(LINEFOLLOW12_INNER_MOTOR,0);
      motor2.Brake(255);
      local_flag = 0;
    }else if(local_flag == 1){
      Move_Forward(0,LINEFOLLOW12_OUTER_MOTOR);
      motor1.Brake(255); 
      local_flag = 0; 
    }else{
      Move_Forward(LINEFOLLOW12_INNER_MOTOR,0); 
      motor2.Brake(255);
    }
  } 
}

void LineFollow34(){
  LAPTOP.println("LineFollow S3S4 ");
  static int local_flag = 0; 
  Serial_Print_Sensors();
  if(S1.High()||S2.High()){
    Move_Forward(0,LINEFOLLOW34_12CORRECTION);  
  }
  if(S3.High()){
    if(S4.High()){
      Move_Forward(LINEFOLLOW34_INNER_MOTOR,LINEFOLLOW34_OUTER_MOTOR);
    }else{
      Move_Forward(0,LINEFOLLOW34_OUTER_MOTOR); 
      local_flag = 2;
    }
  }else if(S4.High()){
    Move_Forward(LINEFOLLOW34_INNER_MOTOR,0); 
    local_flag = 1; 
  }else{
    if(local_flag == 2){
      Move_Forward(0,LINEFOLLOW34_OUTER_MOTOR - 5);
      motor1.Brake(255);
      local_flag = 0;
    }else if(local_flag == 1){
      Move_Forward(LINEFOLLOW34_INNER_MOTOR -5 ,0);
      motor2.Brake(255); 
      local_flag = 0; 
    }else{
      Move_Forward(0,LINEFOLLOW34_OUTER_MOTOR); 
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
      Move_Forward(0,LINEFOLLOW_STRAIGHT_PRECISION_HIGH);
      motor1.Brake(255);   
    }else{
      Move_Forward(0,LINEFOLLOW_STRAIGHT_PRECISION_LOW); 
    }
  }else if(S1.High()){
    Move_Forward(0,LINEFOLLOW_STRAIGHT_PRECISION_LOW);
    motor1.Brake(255);
  }else if(S3.High()){
    if(S4.High()){
      Move_Forward(LINEFOLLOW_STRAIGHT_PRECISION_HIGH,0);
      motor2.Brake(255);
    }else{
      Move_Forward(LINEFOLLOW_STRAIGHT_PRECISION_LOW,0);
    }  
  }else if(S4.High()){
    Move_Forward(LINEFOLLOW_STRAIGHT_PRECISION_LOW,0);
    motor2.Brake(255);
  }else{
    Move_Forward(LINEFOLLOW_STRAIGHT_PRECISION_LOW,LINEFOLLOW_STRAIGHT_PRECISION_LOW);
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
      Move_Forward(0,LINEFOLLOW_CURVE_PRECISION_HIGH);
      motor1.Brake(255);
    }else{
      Move_Forward(0,LINEFOLLOW_CURVE_PRECISION_HIGH); 
    }
  }else if(S1.High()){
    Move_Forward(0,LINEFOLLOW_CURVE_PRECISION_HIGH);
    motor1.Brake(255);
  }else if(S3.High()){
    if(S4.High()){
      Move_Forward(LINEFOLLOW_CURVE_PRECISION_LOW,0);
      motor2.Brake(255);
    }else{
      Move_Forward(LINEFOLLOW_CURVE_PRECISION_LOW,0);
    }  
  }else if(S4.High()){
    Move_Forward(LINEFOLLOW_CURVE_PRECISION_LOW,0);
    motor2.Brake(255);
  }else{
    Move_Forward(LINEFOLLOW_CURVE_PRECISION_LOW,LINEFOLLOW_CURVE_PRECISION_HIGH);
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
    Query_Launchpad();
    LAPTOP.print(encoder_motor1);   LAPTOP.print("\t");   LAPTOP.println(encoder_motor2);
    if((encoder_motor1+encoder_motor2)/2 > encoder_count)
      break;
  }
  Motors_Brake(255,255);
}
