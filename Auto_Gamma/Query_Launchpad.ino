
/** Function to obtain left and right encoder values **/

int Query_Launchpad(){  
  // Frame spec:
  //	Left encoder values LSB
  //	Left encoder values MSB
  //	Right encoder values LSB
  //	Right encoder values MSB
  LAUNCHPAD.write('S');

  if(!Launchpad_Wait())
    return 0;
  diff_encoder_left = LAUNCHPAD.read();
  if(!Launchpad_Wait())
    return 0;
  diff_encoder_left = diff_encoder_left + (uint16_t)(LAUNCHPAD.read()<<8);
  
  if(!Launchpad_Wait())
    return 0;
  diff_encoder_right = LAUNCHPAD.read();
  if(!Launchpad_Wait())
    return 0;
  diff_encoder_right = diff_encoder_right + (uint16_t)(LAUNCHPAD.read()<<8);

  if(mirror){
    encoder_motor2 += diff_encoder_left;
    encoder_motor1 += diff_encoder_right;
  }else{   
    encoder_motor1 += diff_encoder_left;
    encoder_motor2 += diff_encoder_right;
  }
  if(diff_encoder_left == 0 || diff_encoder_right == 0)
    LAPTOP.println("\n ZERO \n");
}

int Launchpad_Wait(){
  long int starttime = millis();
  while(!LAUNCHPAD.available()){
    if( millis()-starttime > 5 ){
      LAPTOP.println("Launchpad didn't respond! :( ");
      //Abort();
      encoder_motor1 += 100;
      encoder_motor2 += 100;
      return 0;
    }
  }
  return 1;
}
