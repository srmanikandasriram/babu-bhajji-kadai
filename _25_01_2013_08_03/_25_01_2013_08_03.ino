// Core AutoBot Code for the entire path
// 25-01-2013 08:03
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

class SENSOR
  {
   public:
        SENSOR(){}
        SENSOR(int pin){
           pinMode(pin,INPUT);
           INPIN = pin;
        }
        boolean High(){
          if(digitalRead(INPIN)==HIGH) return true;
          else  return false;
        }
        boolean Low(){
         if (digitalRead(INPIN) == LOW) return true;
         else return false;
        }
        ~SENSOR(){}
   private:
        int INPIN; 
  };

/** Configuration Constants: Affect behaviour **/
uint16_t distances[13] = {0, 700, 7400, 12150, 16900,950,4000,2000,6030,3800,6800,9800,0}; // 20500, 100, 4000, 3000, 0};

/** Global declarations **/
Motor left_motor(22, 23, 9), right_motor(25, 24, 10), turret_motor(27, 26, 11); // the order of pin numbers determine the direction
SENSOR L1(A13),L2(A14),R1(A12),R2(A11);
const int actuations[] = {0,37,40,44,39,41,48,49};

int NORMAL=40;
int SLOW = 20;
int SLOWEST = 0; 

Servo servo_left, servo_right;
float servo_angle_left, servo_angle_right;
float servo_speeds[] = {0, 0.25, 0.5, 0.5, 0.25, 0.25, 0.25, 0.375, 0.5, 0};

// for PWM
const int mask = 0b11111000;

// for behaviour
const int minimum_pwm = 20, maximum_pwm = 200;
const int acceleration_delay_short = 2, acceleration_delay_long = 5;
const int deceleration_delay_short = 2, deceleration_delay_long = 5;
const int path_profile = 1;
const int encoder_count = 5802;  // Encoder count for hard turn 5802
const int diff = 10;

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
byte sensor_state = 0x00;
const int sensors[] = {0,A11,A12,A13,A14};	// R2, R1, L1, L2
boolean linefollow_enable = false, line_detected = false;

// Core phase variables
int actuation_phase = 0, path_phase = 0;

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
#define SERVO_ANG_L2 95
#define SERVO_ANG_L3 135
#define SERVO_ANG_R1 20
#define SERVO_ANG_R2 40
#define SERVO_ANG_R3 130
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
#define TURRET_ANG1 1080 //1150
#define TURRET_ANG2 1800 //1830
#define TURRET_ANG3 1800 //1830
#define TURRET_ANG4 1030 //1830
#define TURRET_ANG5 4130 //3660
#define TURRET_ANG6 3660
#define RUN_STAGE_TWO 20

float curservo_angle_left = SERVO_ANG_L1, curservo_angle_right = SERVO_ANG_R1;
# define moveforward(l_pwm,r_pwm){\
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

int prevmillis = 0;
void setup(){
  LAPTOP.begin(115200);
  LAUNCHPAD.begin(115200);
  attachInterrupt(0, Turret_ISR, RISING);

  servo_left.attach(SERVO_LFT, 564, 2800);
  servo_right.attach(SERVO_RGT, 564, 2800);
  servo_left.write(SERVO_ANG_L1);
  servo_right.write(SERVO_ANG_R1);

  for(int i = 1; i<7; i++){
    pinMode(actuations[i],OUTPUT);
  }

  // for encoders
  pinMode(TURRET_ENCODER_PIN, INPUT);  

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
  pid.SetOutputLimits(-35,35);  
  LAPTOP.println("Initialised");

  if( Serial_Wait() == 't' ){
    Turret_Reset();
    //Check_Motors();
    Serial_Wait();
  }
  Query_Launchpad();
  Serial_Print();
  encoder_left = encoder_right = 0;
  LAPTOP.println("Encoder values reset.");
//  path_phase = LINEFOLLOWL1L2BRAKE;
  Transform();
}

void loop(){
  prevmillis = millis();
  if( linefollow_enable ){
    Read_Sensors();
    Query_Launchpad();
    PID_Adjust();
    Serial_Print();
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
  Serial.println(millis() - prevmillis);
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

void Read_Sensors(){
  sensor_state = 0x80;
  for(int i = 1; i<=4; i++){
    sensor_state |= digitalRead(sensors[i])<<(i-1);
  }
  //LAPTOP.print("Sensor Reading: ");
  //LAPTOP.println(sensor_state,BIN);
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

/** To accelerate bot **/

void Accelerate_Bot(){
  // Accelerate from rest
  while(base_pwm<140){
    base_pwm++;
//    turret_motor.pwm(base_pwm+75);
    Query_Launchpad();
    PID_Adjust();
    Move_Turret();
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
      turret_motor.Brake(0);
    }
    break;
  case 3:
    if(encoder_turret>TURRET_ANG3){
      turret_motor.Brake(0);
    }else if(encoder_turret>500)
      servo_angle_right = SERVO_ANG_R1;
    break;
  case 4:
    if(encoder_turret>TURRET_ANG4){
      turret_motor.Brake(0);
    }
    break;
  case 5:
    if(encoder_turret>TURRET_ANG5){
      turret_motor.Brake(0);
    }else if(encoder_turret>200)
      servo_angle_left = SERVO_ANG_L1;
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
   LineFollow();
    break;
  case 7:
    LineFollowL1L2();   
    break;
  case 8:
    LineFollowL1L2break();
    if( ( R2.High() && R1.High() && L1.High() )||( L2.High() && R1.High() && L1.High() )  )
      line_detected=true;
    break;
  case 10:
    input = encoder_left;
    pid.Compute();
    if(output>75){
      left_motor.pwm(75);
      right_motor.pwm(75);
    }else if(output>25){
      left_motor.pwm(output);
      right_motor.pwm(output);
    }else{
      left_motor.pwm(25);
      right_motor.pwm(25);
    }
    if(abs(input-setpoint<750)&&(sensor_state&0x0F)){
      LAPTOP.println("\n DETECTED LINE");
      line_detected = true;
    }
    break;
  case 11:
   LineFollow();
    break;
   case 12:
    LineFollowL1L2break();
    if( ( R2.High() && R1.High() && L1.High() )||( L2.High() && R1.High() && L1.High() )  )
      line_detected=true;
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
      delay(3000);
      Actuate_Low(V_PISTON);
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
      Accelerate_Bot();
      LAPTOP.println("Acceleration completed.");
      cons_kp = 1.5;
      pid.SetTunings(cons_kp,cons_ki,cons_kd);
      pid.SetOutputLimits(-55,55);
      base_pwm = VFAST;
      servo_angle_right = SERVO_ANG_R3;
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
      
      
      // turret rotation only after turn     
      /*encoder_turret = 0;
      turret_motor.Control(BCK);
      turret_motor.pwm(140);
      actuation_phase++;*/
      break;
/*   case RUN_STAGE_TWO:
      LAPTOP.println("Into Stage 2");
      path_phase = 5;  
      // encoder_turret = 0;
      encoder_left = 0; encoder_right = 0;
      /*
      turret_motor.Control(FWD);
      turret_motor.pwm(140);
      while(encoder_turret<TURRET_ANG_STAGE_TWO){
        LAPTOP.print("Turret Angle: "); LAPTOP.println(encoder_turret);
      }
      turret_motor.Brake(255);
      
      LAPTOP.println("Turret aligned for stage 2");
      Serial_Wait();
      pid_enable = false;
      LAPTOP.println("Moving forward slightly");
      left_motor.Control(FWD);
      right_motor.Control(FWD);
      left_motor.pwm(50);
      right_motor.pwm(50);
            
       // turret rotation only after turn     
      /*encoder_turret = 0;
      turret_motor.Control(BCK);
      turret_motor.pwm(140);
      actuation_phase++;
      break;
   */
   case LINEFOLLOW:
      // soft turn
      LAPTOP.println("Going into soft turn");
      right_motor.Brake(HARDBRAKE);
      left_motor.pwm(100);
      delay(100);
      while(!R2.High());      
      right_motor.Control(FWD);
      right_motor.pwm(10);
      left_motor.Brake(HARDBRAKE);
      while(!R1.High());
      delay(150);
      right_motor.Brake(HARDBRAKE);
      Serial_Wait();
      LAPTOP.println(" Beginning to follow line ");
      Query_Launchpad();
      encoder_left = encoder_right = 0;      
      //encoder_turret = 0;
      //turret_motor.Control(BCK);
      ///turret_motor.pwm(200);
      linefollow_enable = true;
     // actuation_phase++;
      Serial.println(actuation_phase);
      break;

   case LINEFOLLOWL1L2:
      // brake the bot
      LAPTOP.println("LinegollowL1L2 entered");
      left_motor.Brake(HARDBRAKE);
      right_motor.Brake(HARDBRAKE);
      Serial_Wait();
      Query_Launchpad();
      encoder_left = encoder_right = 0;
      actuation_phase++;
      LAPTOP.println("Setting turret and servo");
      
      //Adjust servo and turret to place leaf after stopping. TEMPORARY. 
      encoder_turret = 0;
      turret_motor.Control(BCK);
      turret_motor.pwm(200);       
      while(encoder_turret<TURRET_ANG4);
      turret_motor.Brake(255);
      servo_left.write(135);
      delay(50);
      Actuate_High(LEFT_VG);
      Serial_Wait();
      LAPTOP.println("Stopping near third leaf");
      Serial_Wait();
      encoder_turret = 0;
      turret_motor.Control(BCK);
      turret_motor.pwm(200);
      break;
      
    case LINEFOLLOWL1L2BRAKE:
      // brake the bot
      
      encoder_left = encoder_right = 0;
      left_motor.Brake(HARDBRAKE);
      right_motor.Brake(HARDBRAKE);
      servo_left.write(65);
      LAPTOP.println("Final alignment.");
      delay(200); 
      while(encoder_turret<TURRET_ANG5);
      turret_motor.Brake(255);
      //Serial_Wait();
      actuation_phase++;
      LAPTOP.println(actuation_phase);
      break;

    case REVERSE_TO_LATITUDE:
      // brake the bot
      left_motor.Brake(HARDBRAKE);
      right_motor.Brake(HARDBRAKE);
      Serial_Wait();
      while(encoder_turret<TURRET_ANG5);
      turret_motor.Brake(255);
      LAPTOP.println("Reached Bud");
      Actuate_High(GRIPPER);
      Serial_Wait();
      setpoint = 0;
      pid.SetMode(AUTOMATIC);
      pid.SetSampleTime(1);
      pid.SetOutputLimits(-35,35);  
      Query_Launchpad();
      encoder_left = encoder_right = 0;
      
      LAPTOP.println("Reverse to latitude");
      base_pwm = VFAST;
      left_motor.Control(BCK);
      right_motor.Control(BCK);
      left_motor.pwm(base_pwm);
      right_motor.pwm(base_pwm);
      linefollow_enable = false;
      pid_enable = true;
      break;
    case TOKYODRIFT:
      left_motor.Brake(HARDBRAKE);
      LAPTOP.println("Aligning onto latitude");
      Set_Turn(90);
      Query_Launchpad();
      encoder_left = encoder_right = 0;
      left_motor.Control(FWD);
      right_motor.Control(BCK);
      left_motor.pwm(120);
      right_motor.pwm(120);
      delay(200);
      linefollow_enable = true;
      line_detected = false; 
      break;
    case LATITUDEFOLLOW1:
      left_motor.Brake(HARDBRAKE);
      right_motor.Brake(HARDBRAKE);
      line_detected=false;
      Serial_Wait();
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
      while(1);      
    }
  }
}

void Set_Turn(float ang){
  setpoint = ang/360.0*encoder_count;
  cons_kp = 300.0/setpoint;
  distances[path_phase] = setpoint;
  pid.SetTunings(cons_kp, cons_ki, cons_kd);
  pid.SetOutputLimits(0,255);  
}

void LineFollow()
{
  if( ( R2.High() && R1.High() && L1.High() )||( L2.High() && R1.High() && L1.High() )  ){  moveforward(SLOW,SLOW);}
  else if(  R2.Low() && R1.Low() && L2.High()  ){  moveforward(SLOWEST,NORMAL);left_motor.Brake(255);}
  else if(  R2.Low() && R1.Low() && L1.High()  ){  moveforward(SLOW,NORMAL); }
  else if(  R2.High() && L1.Low() && L2.Low()  ){  moveforward(NORMAL,SLOWEST); right_motor.Brake(255);}
  else if(  R1.High() && L1.Low() && L2.Low()  ){  moveforward(NORMAL,SLOW); } 
    else if(  R2.Low() && R1.Low() && L1.Low() && L2.Low()  ){  moveforward(NORMAL,NORMAL);}
    delay(2);
}
void LineFollowL1L2()
{
  if( L1.Low() && L2.Low() ){moveforward(SLOW,SLOW);}
  else if( L1.High() && L2.Low() ){moveforward(50,0);}
  else if( L1.Low() && L2.High() ){moveforward(0,100);}
  else if( ( R2.High() && R1.High() && L1.High() )||( L2.High() && R1.High() && L1.High() )  ){mbreak(255,255);Serial.println("node....");delay(1);}
  
}
void LineFollowL1L2break()
{
  if( L1.Low() && L2.Low() ){moveforward(10,10);}
  else if( L1.High() && L2.Low() ){moveforward(30,0);right_motor.Brake(255);}
  else if( L1.Low() && L2.High() ){moveforward(0,30);left_motor.Brake(255);}  
}
void displaysensors()
{
    if(L2.High()) Serial.println("L2");
    if(L1.High()) Serial.println("L1");
    if(R1.High()) Serial.println("R1");
    if(R2.High()) Serial.println("R2");
}
