// Core AutoBot Code for the entire path
// 27-01-2013 07:01PM
/**********************************************************************
  Bot Data:
    wheel to wheel                                -
    wheel dia                                     - 125mm
    no.of interrupts per wheel rotation           - 1422 w/ ext encoders
    no.of interrupts per one full turret rotation - 
***********************************************************************/

/** Necessary Header Files **/
#include <PID_v1.h>
#include <Servo.h>
#include <inttypes.h>

/** Class definition for motor**/
class Motor{
    unsigned int terminal_pin1, terminal_pin2, pwm_pin;
  public:
    Motor();
    Motor(unsigned int pin1, unsigned int pin2, unsigned int pin3){
      terminal_pin1 = pin1;
      terminal_pin2 = pin2;
      pwm_pin = pin3;
      pinMode(terminal_pin1, OUTPUT);
      pinMode(terminal_pin2, OUTPUT);
      pinMode(pwm_pin, OUTPUT);
      pwm(0);
    }

    void Control(boolean motor_direction){
      switch(motor_direction){
        case 1:
          digitalWrite(terminal_pin1, LOW);
          digitalWrite(terminal_pin2, HIGH);
          break;
        case 0:
          digitalWrite(terminal_pin1, HIGH);
          digitalWrite(terminal_pin2, LOW);
          break;
      }
    }

    void pwm(uint8_t pwm){   
     analogWrite(pwm_pin, pwm);
    }

    void Brake(uint8_t strength){
      digitalWrite(terminal_pin1, LOW);
      digitalWrite(terminal_pin2, LOW);
      analogWrite(pwm_pin, strength);
    }
};

class Sensor{
    uint8_t input_pin; 
  public:
    Sensor(){}
    Sensor(uint8_t pin){
      input_pin = pin;
      pinMode(input_pin,INPUT);
    }
    
    boolean High(){
      return digitalRead(input_pin);
    }
    
    boolean Low(){
      return !digitalRead(input_pin);
    }
    
    ~Sensor(){}
};

/** Configuration Constants: Affect behaviour **/
uint16_t distances[25] = {0, 700, 7400, 12150, 16650, 300, 4000, 2400, 8030, 3600, 80, 8000, 9650, 2000, 5000, 5000, 3000, 1000, 4000, 2000, 5000, 5000, 3000, 1000, 4000, 1000};

/** Global declarations **/
Motor left_motor(22, 23, 9), right_motor(25, 24, 10), turret_motor(27, 26, 11); // the order of pin numbers determine the direction
Sensor L1(A13),L2(A14),R1(A12),R2(A11);
const int actuations[] = {0, 37, 40, 44, 39, 41, 48, 49};

int NORMAL = 40;
int SLOW = 20;
int SLOWEST = 0; 

Servo servo_left, servo_right;
float servo_angle_left, servo_angle_right;
float servo_speeds[] = {0, 0.5, 0.5, 0.5, 0.25, 0.25, 0.25, 0.375, 0.5, 0};

// for PWM
const int mask = 0b11111000;

// for behaviour
const int minimum_pwm = 20, maximum_pwm = 200;
const int acceleration_delay_short = 2, acceleration_delay_long = 5;
const int deceleration_delay_short = 2, deceleration_delay_long = 5;
const int path_profile = 1;
const int encoder_count = 11604;  // Encoder count for hard turn 5802

// For PID
double setpoint, input, output;
double cons_kp = 1.0, cons_ki = 0, cons_kd = 0;
double aggressive_kp = 3.0, aggressive_ki = 0, aggressive_kd = 0;
PID pid(&input, &output, &setpoint, cons_kp, cons_ki, cons_kd, DIRECT);
int base_pwm;
boolean pid_enable = 0;

// For encoder values
uint16_t diff_encoder_left = 0, diff_encoder_right = 0;
long int encoder_left = 0, encoder_right = 0;
long int volatile encoder_turret = 0;

// for line sensors
boolean linefollow_enable = false, line_detected = false;

// Core phase variables
int actuation_phase = 0, path_phase = 0;

long int prevmillis = 0;

/** Macros and constants**/
#define Actuate_High(pin) digitalWrite(pin,HIGH);
#define Actuate_Low(pin) digitalWrite(pin,LOW);
#define LAPTOP Serial
#define LAUNCHPAD Serial1
#define FWD 1
#define BCK 0
#define SERVO_LFT 7
#define SERVO_RGT 8
#define SERVO_ANG_L1 60
#define SERVO_ANG_L2 93
#define SERVO_ANG_L3 110
#define SERVO_ANG_R1 20
#define SERVO_ANG_R2 45
#define SERVO_ANG_R3 110
#define TURRET_ENCODER_PIN A10
#define TURRET_DELAY 200
#define LEFT_VG 41
#define MIDDLE_VG 39
#define RIGHT_VG 44
#define V_PISTON 40
#define GRIPPER 37
#define VSLOW 20
#define SLOW 50
#define FAST 100
#define VFAST 200
#define VSOFTBRAKE 100
#define HARDBRAKE 255
#define PICK_LEAVES 1
#define ACCELERATE 2
#define DECELERATE 3
#define DROP_RIGHT_LEAF 4
#define DROP_MIDDLE_LEAF 5
#define LINEFOLLOW 6
#define LINEFOLLOWL1L2 7
#define LINEFOLLOWL1L2BRAKE 8
#define REVERSE_TO_LATITUDE 9
#define TOKYODRIFT 10
#define LATITUDEFOLLOW1 11
#define LATITUDEFOLLOW2 12
#define TRANSFER_FIRST_BUD 13
#define BACKLATITUDE1 14
#define LONGITUDE1 15
#define BUD2 16
#define LATITUDE3 17
#define TURRET_ANG1 1080 //1150
#define TURRET_ANG2 1750 //1830
#define TURRET_ANG3 2000 //1800 //1830
#define TURRET_ANG4 1700 //1030 //1830
#define TURRET_ANG5 3400 //3660

float curservo_angle_left = SERVO_ANG_L1, curservo_angle_right = SERVO_ANG_R1;

#define moveforward(l_pwm,r_pwm){\
  left_motor.Control(FWD);\
  right_motor.Control(FWD);\
  left_motor.pwm(l_pwm);\
  right_motor.pwm(r_pwm);\
}

#define moveback(l_pwm,r_pwm){\
  left_motor.Control(BCK);\
  right_motor.Control(BCK);\
  left_motor.pwm(l_pwm);\
  right_motor.pwm(r_pwm);\
}

#define mbreak(l_pwm,r_pwm){\
   left_motor.Brake(l_pwm);\
  right_motor.Brake(r_pwm);\
}

void setup(){
  LAPTOP.begin(115200);
  LAUNCHPAD.begin(115200);
  attachInterrupt(0, Turret_ISR, RISING);

  servo_left.attach(SERVO_LFT, 564, 2800);
  servo_right.attach(SERVO_RGT, 564, 2800);
  servo_left.write(SERVO_ANG_L1);
  servo_right.write(SERVO_ANG_R1);

  for(int i = 1; i<8; i++){
    pinMode(actuations[i],OUTPUT);
  }

  // for encoders
  pinMode(TURRET_ENCODER_PIN, INPUT);  
  pinMode(A9, INPUT);

  // for sensors
  pinMode(sensors[1],INPUT);
  pinMode(sensors[2],INPUT);
  pinMode(sensors[3],INPUT);
  pinMode(sensors[4],INPUT);
  
  // for PWM
  TCCR1B = TCCR1B & mask | 0x02;

  // for PID
  input = 0;
  setpoint = 0;  
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(1);
  pid.SetOutputLimits(-15,15);

//  Handshake_Launchpad();
  LAPTOP.println("Initialised");
  
  char temp = Serial_Wait();
  if( temp == 't' ){
    Turret_Reset();
    //Check_Motors();
    Serial_Wait();
  }else if( temp == 'p'){  
    Parallelogram_Reset(0,1);
    Serial_Wait();
  }
//  Parallelogram_Reset(1,1);
  Query_Launchpad();
  Serial_Print();
  encoder_left = encoder_right = 0;
  LAPTOP.println("Encoder values reset.");
  //path_phase = DROP_RIGHT_LEAF;
  //actuation_phase = 3;
  Transform();
}

void loop(){
  prevmillis = millis();
  if( linefollow_enable ){
    Query_Launchpad();
    PID_Adjust();
    Serial_Print();
    Move_Turret();
    Move_Servo();
    Check_Abort();
    if( (encoder_left > distances[path_phase]) || (encoder_right > distances[path_phase]) || line_detected)
      Transform();
  }else if( (encoder_left < distances[path_phase]) && (encoder_right < distances[path_phase]) ){
    Move_Turret();
    Move_Servo();
    Query_Launchpad();
    if (pid_enable){
      PID_Adjust();
    }
    Serial_Print();
    delay(acceleration_delay_long);
    Check_Abort();
  } else {
    Transform();
  }
  LAPTOP.println(millis() - prevmillis);
}

void Handshake_Launchpad(){
  char msg = '@';
  while(msg != '#'){
    msg = '@';
    LAUNCHPAD.write(msg);
    while(!LAUNCHPAD.available());
    msg = LAUNCHPAD.read();
    delay(500);    
  }
  LAPTOP.println("Handshake completed");
}

void Check_Motors(){
  while(1){
    left_motor.Control(FWD);
    left_motor.pwm(100);
    right_motor.Control(FWD);
    right_motor.pwm(100);
    Check_Abort();
    Query_Launchpad();
    Serial_Print();
  }
}

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

  encoder_left+=diff_encoder_left;
  encoder_right+=diff_encoder_right;
  if(diff_encoder_left == 0 || diff_encoder_right == 0)
    LAPTOP.println("\n ZERO \n");
}

/** Helper functions **/
char Serial_Wait(){
  while(!LAPTOP.available());
  return LAPTOP.read();
}

void Check_Abort(){
  if (LAPTOP.available()) {
    left_motor.Brake(255);
    right_motor.Brake(255);
    turret_motor.Brake(0);
    while(1);
  }
}

void Serial_Print(){
  LAPTOP.print("encl: ");
  LAPTOP.print(encoder_left);
  LAPTOP.print("\tencr: ");
  LAPTOP.print(encoder_right);
  LAPTOP.print("\tenct: ");
  LAPTOP.print(encoder_turret);
  LAPTOP.print("\tcsl: ");
  LAPTOP.print(curservo_angle_left);
  LAPTOP.print("\tcsr: ");
  LAPTOP.print(curservo_angle_right);
  if(pid_enable){
    LAPTOP.print("\t  Current Diff: ");
    LAPTOP.print(input - setpoint);
    LAPTOP.print("\t PID Output: ");
    LAPTOP.print(output);
    LAPTOP.print("\t Left PWM: ");
    LAPTOP.print(base_pwm + output);
    LAPTOP.print("\t Right PWM: ");
    LAPTOP.println(base_pwm - output);  
  }else{
    LAPTOP.print("\n");
  }
}

/** Core Functions **/
void Turret_Reset(){
  LAPTOP.println("Resetting Turret");
  turret_motor.pwm(100);
  if(Serial_Wait() == 'c') {
    turret_motor.Control(FWD);
  }else{
    turret_motor.Control(BCK);
  }
  delay(TURRET_DELAY);

  int sensor_reading, detected_count = 0, reading_count = 0;

  while(!LAPTOP.available()){
    sensor_reading = 0;
    reading_count = 0;
    while(reading_count<5){
      sensor_reading += analogRead(TURRET_ENCODER_PIN);
      delay(5);
      reading_count++;
    }
    sensor_reading /= 5;
    if(sensor_reading <= 300)
      detected_count++;
    else
      detected_count = 0;
    if(detected_count>5){
      turret_motor.Brake(0);
      break;
    }
  }
  if(LAPTOP.available()){
    LAPTOP.read();
    LAPTOP.println("Aborted Turret reset!");
    turret_motor.Brake(0);
    while(1);
  }  
}

void Parallelogram_Reset(int dir, int num){
  LAPTOP.println("Resetting Parallelogram");
  int Reading, i = 0, j = 0, k;
  if(dir){
    digitalWrite(48,HIGH);
    digitalWrite(49,LOW);
  }else{
    digitalWrite(48,LOW);
    digitalWrite(49,HIGH);
  }
  for( k = 0; k<num; k++){
    delay(500);
    while(!Serial.available()){
      Reading = 0;
      j = 0;
      while(j<5){
        Reading += analogRead(A9);
        delay(5);
        j++;
      }
      Reading /= 5;
      if(Reading <= 300)
        i++;
      if(i>5){
        digitalWrite(48,LOW);
        digitalWrite(49,LOW);
        break;
      }
    }
    if(Serial.available()){
      Serial.read();
      Serial.println("ABORTED!");
      digitalWrite(48,LOW);
      digitalWrite(49,LOW);
    }
    LAPTOP.println("one tape done");
  }
}

/** To accelerate bot **/

void Accelerate_Bot(){
  // Accelerate from rest
  while(base_pwm<140){
    base_pwm++;
    if(base_pwm>35)
      pid.SetOutputLimits(-35,35);
//    turret_motor.pwm(base_pwm+75);
    Query_Launchpad();
    PID_Adjust();
    Move_Turret();
    Move_Servo();
    delay(acceleration_delay_short);
    Serial_Print();
    Check_Abort();
  }
}

void Decelerate_Bot(){
  // Decelerate to half
  while(base_pwm>50){
    base_pwm -= 5;
    Query_Launchpad();
    PID_Adjust();
    delay(deceleration_delay_short);
    Move_Turret();
    Move_Servo();
    Serial_Print();
    Check_Abort();
  }
}

void Move_Servo(){
//  static float curservo_angle_left = SERVO_ANG_L1, curservo_angle_right = SERVO_ANG_R1;
  float servo_speed = servo_speeds[path_phase];
  if(curservo_angle_left < servo_angle_left){
    curservo_angle_left += servo_speed;
    servo_left.write((int)curservo_angle_left);
  }else if(curservo_angle_left>servo_angle_left){
    curservo_angle_left -= servo_speed;
    servo_left.write((int)curservo_angle_left);
  }
  if(curservo_angle_right<servo_angle_right){
    curservo_angle_right += servo_speed;
    servo_right.write((int)curservo_angle_right);
  }else if(curservo_angle_right>servo_angle_right){
    curservo_angle_right -= servo_speed;
    servo_right.write((int)curservo_angle_right);
  }
}

void Move_Turret(){
  switch(actuation_phase){
  case 1:
    if(encoder_turret>TURRET_ANG1){
      turret_motor.Brake(0);
    }
    break;
  case 2:
    if(encoder_turret>TURRET_ANG2){
      turret_motor.Brake(255);
    }
    break;
  case 3:
    if(encoder_turret>TURRET_ANG3){
      turret_motor.Brake(0);
    }
    break;
  case 4:
    if(encoder_turret>TURRET_ANG4){
      turret_motor.Brake(0);
    }
    break;
  case 5:
    if(encoder_turret>TURRET_ANG5){
      turret_motor.Brake(0);
    }
    break;
  }
}

void PID_Adjust(){
  switch(path_phase){
  case 2:
  case 3:
  case 4:
  case 5:
  case 9:
   /** straight line PID **/

    input = encoder_left - encoder_right;
    if(input>-15&&input<15)
      pid.SetTunings(cons_kp, cons_ki, cons_kd);
    else
      pid.SetTunings(aggressive_kp, aggressive_ki, aggressive_kd);
    pid.Compute();
    left_motor.pwm(base_pwm + output);
    right_motor.pwm(base_pwm - output);
    break;
  case 6:
  case 11:
  case 14:
  case 16:
  case 17:
  case 20:
  case 22:
  case 23:
    LineFollow();
    break;
  case 7:
    LineFollowL1L2();   
    break;
  case 8:
  case 12:
  case 15:
  case 18:
  case 21:
  case 24:
    LineFollowL1L2break();
    if( ( R2.High() && R1.High() && L1.High() )||( L2.High() && R1.High() && L1.High() )  )
      line_detected = true;
    break;
  case 10:
    input = encoder_right;
    pid.Compute();
    if(output>150){
      left_motor.pwm(150);
      right_motor.pwm(150);
    }else if(output>15){
      left_motor.pwm(output);
      right_motor.pwm(output);
    }else{
      left_motor.pwm(15);
      right_motor.pwm(15);
    }
    if(R2.High()||R1.High()||L1.High()||L2.High()){
      LAPTOP.println("\n DETECTED LINE");
      line_detected = true;
    }
    break; 
  }
}

void Turret_ISR(){
  encoder_turret++;
}

void Transform(){
  path_phase++;
  if(path_profile == 1){
    switch(path_phase){
    case PICK_LEAVES:
      // 15cm to Leaves
      LAPTOP.println("Going to pick up the leaves.");
      pid_enable = false;
      left_motor.Control(FWD);
      right_motor.Control(FWD);
      left_motor.pwm(VSLOW);
      right_motor.pwm(VSLOW);
      encoder_turret = 0;
      turret_motor.Control(FWD);
      turret_motor.pwm(255);
      actuation_phase++;
      servo_angle_left = SERVO_ANG_L2;
      servo_angle_right = SERVO_ANG_R2;
      break;
      
    case ACCELERATE:
      // Brake the bot
      LAPTOP.println("Stopping to pick up leaves");
      left_motor.Brake(VSOFTBRAKE);
      right_motor.Brake(VSOFTBRAKE);
      servo_left.write(SERVO_ANG_L2);
      servo_right.write(SERVO_ANG_R2);
      while(encoder_turret<TURRET_ANG1);
      turret_motor.Brake(0);
      LAPTOP.println("Turret has aligned.");
      Actuate_High(V_PISTON);
      delay(1000);
      Actuate_Low(V_PISTON);
//      delay(1000);
      Serial_Wait();
      Query_Launchpad();
      encoder_left = encoder_right = 0;
      // Acceleration
      LAPTOP.println("Bot is accelerating");
      pid_enable = true;
      left_motor.Control(FWD);
      right_motor.Control(FWD);
      encoder_turret = 0;
      turret_motor.Control(FWD);
      turret_motor.pwm(150);
      actuation_phase++;
      base_pwm = minimum_pwm;
      servo_angle_right = SERVO_ANG_R3;
      Accelerate_Bot();
      LAPTOP.println("Acceleration completed.");
      cons_kp = 1.5;
      pid.SetTunings(cons_kp,cons_ki,cons_kd);
      pid.SetOutputLimits(-55,55);
      base_pwm = VFAST;
      break;
      
    case DECELERATE:
      // slow down
      LAPTOP.println("Deceleration begun!");
      base_pwm = 120;
      Decelerate_Bot();
      base_pwm = 40;
      cons_kp = 1.5;
      pid.SetTunings(cons_kp,cons_ki,cons_kd);
      pid.SetOutputLimits(-35,35);
      LAPTOP.println("Deceleration done");
      break;
      
    case DROP_RIGHT_LEAF:
      Actuate_High(RIGHT_VG);
      LAPTOP.println("Right leaf dropped!");
      if(encoder_turret<TURRET_ANG2)
        LAPTOP.println("didnt rotate enough!!");
      encoder_turret = 0;
      turret_motor.Control(FWD);
      turret_motor.pwm(180);
      actuation_phase++;
      break;
      
  case DROP_MIDDLE_LEAF:
      // brake the bot
      LAPTOP.println("Stopping at second leaf");
      left_motor.Brake(HARDBRAKE);
      right_motor.Brake(HARDBRAKE);
      while(encoder_turret<TURRET_ANG2);
      turret_motor.Brake(0);
      LAPTOP.println("Middle leaf dropped");
      Serial_Print();
      Actuate_High(MIDDLE_VG);
      Serial_Wait();
      Query_Launchpad();
      encoder_left = 0; encoder_right = 0;
      pid_enable = false;
      LAPTOP.println("Moving forward slightly");
      left_motor.Control(FWD);
      right_motor.Control(FWD);
      left_motor.pwm(50);
      right_motor.pwm(50);
      while(L1.Low()&&R1.Low());
      mbreak(255,255);
      delay(10);
      left_motor.Control(FWD);
      right_motor.Control(FWD);
      left_motor.pwm(50);
      right_motor.pwm(50);
      servo_angle_left = SERVO_ANG_L2;
      servo_right.write(SERVO_ANG_R1);
      encoder_turret = 0;
      turret_motor.Control(BCK);
      turret_motor.pwm(150);
      actuation_phase++;
      break;
      
   case LINEFOLLOW:
      // soft turn
      LAPTOP.println("Going into soft turn");
      right_motor.Brake(HARDBRAKE);
      left_motor.pwm(150);
      delay(100);
      while(!R2.High());      
      right_motor.Control(FWD);
      right_motor.pwm(30);
      left_motor.Brake(HARDBRAKE);
      while(!R1.High());
      delay(150);
      right_motor.Brake(HARDBRAKE);
      LAPTOP.println(" Beginning to follow line ");
      Query_Launchpad();
      encoder_left = encoder_right = 0;     
      linefollow_enable = true;
      break;

   case LINEFOLLOWL1L2:
      // brake the bot
      LAPTOP.println("LinegollowL1L2 entered");
      left_motor.Brake(HARDBRAKE);
      right_motor.Control(BCK);
      right_motor.pwm(30);
      while(!L2.High());
      right_motor.Brake(HARDBRAKE);
      Query_Launchpad();
      encoder_left = encoder_right = 0;
      while(encoder_turret<TURRET_ANG4);
      turret_motor.Brake(255);
      servo_left.write(SERVO_ANG_L3);
      delay(500);
      Actuate_High(LEFT_VG);
      delay(500);
      Serial_Wait();
      LAPTOP.println("Stopping near third leaf");
      encoder_turret = 0;
      turret_motor.Control(BCK);
      turret_motor.pwm(200);
      actuation_phase++;
      servo_angle_left = SERVO_ANG_L2;
      break;
      
    case LINEFOLLOWL1L2BRAKE:
      // brake the bot
      Query_Launchpad();
      encoder_left = encoder_right = 0;
      left_motor.Brake(HARDBRAKE);
      right_motor.Brake(HARDBRAKE);
      LAPTOP.println("Final alignment.");
      break;
      
    case REVERSE_TO_LATITUDE:
      // brake the bot
      left_motor.Brake(HARDBRAKE);
      right_motor.Brake(HARDBRAKE);
      while(encoder_turret<TURRET_ANG5);
      turret_motor.Brake(255);
      LAPTOP.println("Reached Bud");
//      Parallelogram_Reset(0,1);
      Serial_Wait();
      Actuate_High(GRIPPER);
      //Parallelogram_Reset(1,1);
      Serial_Wait();
      setpoint = 0;
      pid.SetMode(AUTOMATIC);
      pid.SetSampleTime(1);
      pid.SetOutputLimits(-35,35);  
      
      LAPTOP.println("Reverse to latitude");
      base_pwm = VFAST;
      left_motor.Control(BCK);
      right_motor.Control(BCK);
      left_motor.pwm(50);
      right_motor.pwm(0);
      delay(300);
      Query_Launchpad();
      encoder_left = encoder_right = 0;
      linefollow_enable = false;
      pid_enable = true;
      break;
    case TOKYODRIFT:
      LAPTOP.println("Aligning onto latitude");
      Set_Turn(distances[path_phase]);
      LAPTOP.println(distances[path_phase]);
      Query_Launchpad();
      encoder_left = encoder_right = 0;
      left_motor.Control(FWD);
      right_motor.Control(BCK);
      left_motor.pwm(255);
      right_motor.pwm(255);
      delay(400);
      base_pwm = 0;
      LAPTOP.println("delay over");
      linefollow_enable = true;
      line_detected = false; 
      break;
    case LATITUDEFOLLOW1:
      left_motor.Brake(HARDBRAKE);
      right_motor.Brake(HARDBRAKE);
      line_detected=false;
      LAPTOP.println("Line follow 1");
      Serial_Print();
      LAPTOP.println("Line follow 1");
      break;
    case LATITUDEFOLLOW2:
      left_motor.Brake(HARDBRAKE);
      right_motor.Brake(HARDBRAKE);
      LAPTOP.println("Line follow 2");
      break;
    case TRANSFER_FIRST_BUD:
      left_motor.Brake(HARDBRAKE);
      right_motor.Brake(HARDBRAKE);
      LAPTOP.println("Reached manual bot.");
      Query_Launchpad();
      encoder_left = encoder_right = 0;
      linefollow_enable = false;
      pid_enable = false;
      Serial_Wait();
//      moveback(155,75);
//      delay(300);
      moveback(155,155);
      break;
    case 14:
    case 20:
       mbreak(0,255);
       left_motor.Control(BCK);
       left_motor.pwm(60);
       while(!L2.High());
       mbreak(255,255);
       Serial_Wait();
       linefollow_enable = true;
       line_detected = false; 
       Query_Launchpad();
       encoder_left = encoder_right = 0;
       moveforward(50,50);
       //move motor down.........
       break;
    case 15:
    case 21:
      left_motor.Brake(HARDBRAKE);
      right_motor.Brake(HARDBRAKE);
      Serial_Wait();
      Query_Launchpad();
      encoder_left = encoder_right = 0;
      LAPTOP.println("near next bud.....");
      moveforward(50,50);
      break;
    case 16:
    case 22:
      left_motor.Brake(HARDBRAKE);
      right_motor.Brake(HARDBRAKE);
      Serial_Wait();
      Query_Launchpad();
      encoder_left = encoder_right = 0;
      LAPTOP.println("eexact bud");
      //move  motor up after clamp.......
      moveback(200,255);
      line_detected = false; 
      break;
    case 17:
    case 23:
       mbreak(255,0);
       Query_Launchpad();
       encoder_left = encoder_right = 0;
       right_motor.Control(BCK);
       right_motor.pwm(50);
       while(encoder_right<4000){  //!R2.High()||
         Query_Launchpad();
         if(R2.High()&&encoder_right>3000){
           Serial.println("line detected,,,,,");
           break;
         }
       }
       mbreak(255,255);
       Serial_Wait();
       Query_Launchpad();
       encoder_left = encoder_right = 0;
       moveforward(30,30);
       break;
     case 18:
     case 24:
       mbreak(255,255);
       Serial_Wait();
       Query_Launchpad();
       encoder_left = encoder_right = 0;
       Serial.println("going to transfer next bud");
       break;
     case 19:
     case 25:
       mbreak(255,255);
       Serial_Wait();
       Query_Launchpad();
       encoder_left = encoder_right = 0;
       Serial.println("transfering next bud near manual bot..");       
       moveback(155,155);
       linefollow_enable = false;
       break;
     case 26:
       mbreak(255,255);
       LAPTOP.println("completed");
       while(1);
    }
  }
}

void Set_Turn(float ang){
  setpoint = ang/360.0*encoder_count;
  cons_kp = 255.0/setpoint;
  distances[path_phase] = setpoint;
  pid.SetTunings(cons_kp, cons_ki, cons_kd);
  pid.SetOutputLimits(0,255);  
}

void LineFollow(){
  if( ( R2.High() && R1.High() && L1.High() )||( L2.High() && R1.High() && L1.High() )  ){  moveforward(SLOW,SLOW);}
  else if(  R2.Low() && R1.Low() && L2.High()  ){  moveforward(SLOWEST,NORMAL);left_motor.Brake(255);}
  else if(  R2.Low() && R1.Low() && L1.High()  ){  moveforward(SLOW,NORMAL); }
  else if(  R2.High() && L1.Low() && L2.Low()  ){  moveforward(NORMAL,SLOWEST); right_motor.Brake(255);}
  else if(  R1.High() && L1.Low() && L2.Low()  ){  moveforward(NORMAL,SLOW); } 
  delay(2);
}

void LineFollowL1L2(){
  if( L1.High() && L2.Low() ){moveforward(50,0);}
  else if( L1.Low() && L2.High() ){moveforward(0,100);}
  else if( ( R2.High() && R1.High() && L1.High() )||( L2.High() && R1.High() && L1.High() )  ){mbreak(255,255);Serial.println("node....");delay(1);}
}

void LineFollowL1L2break(){
  if( L1.Low() && L2.Low() ){moveforward(10,10);}
  else if( L1.High() && L2.Low() ){moveforward(30,0);right_motor.Brake(255);}
  else if( L1.Low() && L2.High() ){moveforward(0,30);left_motor.Brake(255);}  
}

void displaysensors(){
  if(L2.High()) Serial.println("L2");
  if(L1.High()) Serial.println("L1");
  if(R1.High()) Serial.println("R1");
  if(R2.High()) Serial.println("R2");
}
