
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

  
  encoder_left += diff_encoder_left;
  encoder_right += diff_encoder_right;
  
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

/** Function to obtain left and right encoder values *

void Query_Launchpad(){  
  // Frame spec:
  //    Left encoder values LSB
  //    Left encoder values MSB
  //    Right encoder values LSB
  //    Right encoder values MSB
  
  LAUNCHPAD.write('S'); // tell it to freeze values
//  LAPTOP.println("sent S");
  uint16_t diff_encoder_left_2 = 0, diff_encoder_right_2 = 0;
  
  do {
    LAUNCHPAD.write('l'); // tell it to give L
//    LAPTOP.print("sent: l\treceived:\t");
    while(!LAUNCHPAD.available());
    diff_encoder_left_2 = LAUNCHPAD.read();
//    LAPTOP.println(diff_encoder_left_2,BIN);
  } while( (diff_encoder_left_2 & 0b11100000) != 0b10100000 );
  diff_encoder_left = diff_encoder_left_2 & 0b00011111;
//  delay(100);
  do {
    LAUNCHPAD.write('L'); // tell it to give L2
//    LAPTOP.print("sent: L\treceived:\t");
    while(!LAUNCHPAD.available());
    diff_encoder_left_2 = LAUNCHPAD.read();
//    LAPTOP.println(diff_encoder_left_2,BIN);
  } while( (diff_encoder_left_2 & 0b11100000) != 0b01000000 );
  diff_encoder_left_2 &= 0b00011111;
  diff_encoder_left = diff_encoder_left + (uint16_t)(diff_encoder_left_2<<5);
//  delay(100);
  do {
    LAUNCHPAD.write('r'); // tell it to give L
//    LAPTOP.print("sent: r\treceived:\t");
    while(!LAUNCHPAD.available());
    diff_encoder_right_2 = LAUNCHPAD.read();
//    LAPTOP.println(diff_encoder_right_2,BIN);
  } while( (diff_encoder_right_2 & 0b11100000) != 0b11000000 );
  diff_encoder_right = diff_encoder_right_2 & 0b00011111;
//  delay(100);
  do {
    LAUNCHPAD.write('R'); // tell it to give L2
//    LAPTOP.print("sent: R\treceived:\t");
    while(!LAUNCHPAD.available());
    diff_encoder_right_2 = LAUNCHPAD.read();
//    LAPTOP.println(diff_encoder_right_2,BIN);
  } while( (diff_encoder_right_2 & 0b11100000) != 0b01100000 );
  diff_encoder_right_2 &= 0b00011111;
  diff_encoder_right = diff_encoder_right + (uint16_t)(diff_encoder_right_2<<5);

  
  encoder_left += diff_encoder_left;
  encoder_right += diff_encoder_right;
  if(diff_encoder_left == 0 || diff_encoder_right == 0)
    LAPTOP.println("\n ZERO \n"); 

}*/
