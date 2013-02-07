
/** Function to obtain left and right encoder values **/

void Query_Launchpad(){  
  // Frame spec:
  //	Left encoder values LSB
  //	Left encoder values MSB
  //	Right encoder values LSB
  //	Right encoder values MSB
  
  LAUNCHPAD.write('S');
  
  while(!LAUNCHPAD.available());
  diff_encoder_left = LAUNCHPAD.read();
  while(!LAUNCHPAD.available());
  diff_encoder_left = diff_encoder_left + (uint16_t)(LAUNCHPAD.read()<<8);
  
  while(!LAUNCHPAD.available());
  diff_encoder_right = LAUNCHPAD.read();
  while(!LAUNCHPAD.available());
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
