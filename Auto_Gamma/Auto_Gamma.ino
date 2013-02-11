// Core AutoBot Code
// 08-01-2013 04 47 PM
/**********************************************************************
  Bot Data:
    wheel to wheel                                - 50cm
    wheel dia                                     - 125mm
    no.of interrupts per wheel rotation           - 2688
    no.of interrupts per one full turret rotation - 
***********************************************************************/

/** Necessary Header Files **/
#include <PID_v1.h>
#include <Servo.h>
#include <inttypes.h>
#include <LiquidCrystal.h>

/** Class definition for motor **/
class Motor{
    unsigned int terminal_pin1, terminal_pin2, pwm_pin;
  public:
    Motor(){}
    Motor(unsigned int pin1, unsigned int pin2, unsigned int pin3){
      terminal_pin1 = pin1;
      terminal_pin2 = pin2;
      pwm_pin = pin3;
      pinMode(terminal_pin1, OUTPUT);
      pinMode(terminal_pin2, OUTPUT);
      pinMode(pwm_pin, OUTPUT);
      pwm(0);
    }
    
    void Attach(unsigned int pin1, unsigned int pin2, unsigned int pin3){
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
    
    void Control(boolean motor_direction, uint8_t motor_pwm){
      Control(motor_direction);
      pwm(motor_pwm);
    }

    void pwm(uint8_t pwm){   
     analogWrite(pwm_pin, pwm);
    }

    void Brake(uint8_t strength){
      digitalWrite(terminal_pin1, LOW);
      digitalWrite(terminal_pin2, LOW);
      analogWrite(pwm_pin, strength);
    }
    
    ~Motor(){}
};

class Sensor{
    uint8_t input_pin;
    int flicker_count;
  public:
    Sensor(){
      flicker_count = 10;
    }
    
    void Attach(uint8_t pin){
      input_pin = pin;
      pinMode(input_pin,INPUT);
    }
    
    boolean High(){
      int reading = 0;
      for(int i =0; i<flicker_count;i++)
        reading += digitalRead(input_pin);
      if(reading >= flicker_count/2)
        return true;
      else
        return false;
    }
    
    boolean Low(){
      int reading = 0;
      for(int i =0; i<flicker_count;i++)
        reading += !digitalRead(input_pin);
      if(reading>=flicker_count/2)
        return true;
      else
        return false;
    }
        
    ~Sensor(){}
};

class Custom_Servo{
    Servo servo;
    int angles[4], target_angle;
    float current_angle;
  public:
    Custom_Servo(){}
    
    void Attach(int servo_pin){
      servo.attach(servo_pin, 564, 2800);      
    }
    
    void SetAngles(int angle1, int angle2, int angle3){
      angles[1] = angle1;
      angles[2] = angle2;
      angles[3] = angle3;
      current_angle = angles[1];
      servo.write(current_angle);
    }
    
    void SetTargetAngle(int angle_index){
      target_angle = angles[angle_index];
    }
    
    void Home(){
      servo.write(angles[1]);
    }
    
    void Middle(){
      servo.write(angles[2]);
    }
    
    void Extend(){
      servo.write(angles[3]);
    }
    
    void Angle(int angle){
      servo.write(angle);  
    }

    void Sweep(float servo_speed){
      if(current_angle < target_angle){
        current_angle += servo_speed;
        servo.write((int)current_angle);
      }else if(current_angle > target_angle){
        current_angle -= servo_speed;
        servo.write((int)current_angle);
      }
    }
    
    float GetAngle(){
      return current_angle;
    }
    
    ~Custom_Servo(){}
};

/** Macros and constants**/
#define Actuate_High(pin) digitalWrite(pin,HIGH)
#define Actuate_Low(pin) digitalWrite(pin,LOW)
#define LAPTOP Serial
#define LAUNCHPAD Serial1
#define FWD 1
#define BCK 0

#define SERVO_LFT 7
#define SERVO_RGT 8
#define SERVO_ANG_L1 120
#define SERVO_ANG_L2 100
#define SERVO_ANG_L3 0
#define SERVO_ANG_R1 40
#define SERVO_ANG_R2 58
#define SERVO_ANG_R3 140

#define SERVO_ANG_L1F 120
#define SERVO_ANG_L2F 99
#define SERVO_ANG_L3F 52
#define SERVO_ANG_R1F 40
#define SERVO_ANG_R2F 58
#define SERVO_ANG_R3F 53

#define TURRET_SENSOR_PIN A10
#define PARALLELOGRAM_SENSOR_PIN 1

#define LEFT_VG 41
#define MIDDLE_VG 39
#define RIGHT_VG 44
#define V_PISTON 40
#define GRIPPER 37
#define PARALLELOGRAM_PIN1 48
#define PARALLELOGRAM_PIN2 49

#define VSLOW 20
#define FAST 100
#define VFAST 200
#define VSOFTBRAKE 100
#define HARDBRAKE 255

#define TURRET_ANG1 1180
#define TURRET_ANG2 2930
#define TURRET_ANG3 4730
#define TURRET_ANG4 6300
#define TURRET_ANG5 10330

#define TURRET_ANG1F 1180
#define TURRET_ANG2F 2930
#define TURRET_ANG3F 4730
#define TURRET_ANG4F 6300
#define TURRET_ANG5F 10330

#define STRAIGHT_LINE_PID 1
#define SOFT_TURN_PID 2

#define Move_Forward(pwm1,pwm2){\
  motor1.Control(FWD,pwm1);\
  motor2.Control(FWD,pwm2);\
}

#define Move_Back(pwm1,pwm2){\
  motor1.Control(BCK,pwm1);\
  motor2.Control(BCK,pwm2);\
}

#define Motors_Brake(pwm1,pwm2){\
  motor1.Brake(pwm1);\
  motor2.Brake(pwm2);\
}

#define Parallelogram_Up(){\
  digitalWrite(PARALLELOGRAM_PIN1,HIGH);\
  digitalWrite(PARALLELOGRAM_PIN2,LOW);\
}

#define Parallelogram_Down(){\
  digitalWrite(PARALLELOGRAM_PIN1,LOW);\
  digitalWrite(PARALLELOGRAM_PIN2,HIGH);\
}

#define Parallelogram_Stop(){\
  digitalWrite(PARALLELOGRAM_PIN1,LOW);\
  digitalWrite(PARALLELOGRAM_PIN2,LOW);\
}

#define Check_Abort(){\
  if( LAPTOP.available() )\
    Abort();\
}

// for Array of Functions
typedef void (*fn) (void);
fn Transform[] = {Initialise, Pick_Leaves, Accelerate_Bot, Decelerate_Bot, Drop_First_Leaf, Drop_Second_Leaf, Soft_Turn, Auto_Stage_One_Complete, Auto_Stage_Two};
fn Transform_Fallback[] = {Initialise, Pick_Leaves1, Accelerate_Bot1, Decelerate_Bot1, Detect_Line, Auto_Fallback_Begin, LineFollow_Fallback};
/** Configuration Constants: Affect behaviour **/
uint16_t distances[26] = {0, 2930, 13880, 21650, 29100, 565, 150,100};
uint16_t distances_fallback[26] = {0, 2930, 13880, 17000, 700, 100};

/** Global declarations **/
Motor motor1, motor2, turret_motor(27, 26, 11); // the order of pin numbers determine the direction  left_motor(22, 23, 9), right_motor(25, 24, 10),

//Sensor L1(A13),L2(A14),R1(A12),R2(A11);
Sensor S1, S2, S3, S4, SW;
const int actuations[] = {0, 37, 40, 44, 39, 41, 48, 49};

int NORMAL = 60;
int SLOW = 25;
int SLOWEST = 0; 

// for Servo
Custom_Servo servo1, servo2;
float servo_speeds[] = {0, 0.5, 0.75, 1.5, 0.25, 0.25, 0.25, 0.375, 0.5, 0};

// for PWM
const int mask = 0b11111000;

// for behaviour
const int stratergy = 1;  // 1 for 1-2-4 and 2 for 1-2-3
const int minimum_pwm = 20, maximum_pwm = 100, slowdown_pwm = 50;
const int acceleration_delay = 2, acceleration = 2;
const int deceleration_delay = 2, deceleration = 5;
const int encoder_count = 22134;  // Encoder count for hard turn 5802
const int delay_long = 5;
boolean stage_one_complete = false, mirror, fallback_begin = false;

// For PID
double setpoint, input, output;
double cons_kp = 1.0, cons_ki = 0, cons_kd = 0;
double aggressive_kp = 3.0, aggressive_ki = 0, aggressive_kd = 0;
PID pid(&input, &output, &setpoint, cons_kp, cons_ki, cons_kd, DIRECT);
int base_pwm, pid_type = STRAIGHT_LINE_PID;
boolean pid_enable = 0;

// For encoder values
uint16_t diff_encoder_left = 0, diff_encoder_right = 0;
long int encoder_motor1 = 0, encoder_motor2 = 0;
long int volatile encoder_turret = 0;

// for line sensors
boolean linefollow_enable = false, line_detected = false;

// Core phase variables
int actuation_phase = 0, path_phase = 0;

int parallelogram_count = 0; 
long int prevmillis = 0;

LiquidCrystal LCD(13, 34, 30, 31, 32, 33);

void setup(){
  SW.Attach(A0);
  LAPTOP.begin(115200);
  LAUNCHPAD.begin(115200);
  LCD.begin(20,4);
  LCD.print("Hello World!");
  attachInterrupt(0, Turret_ISR, RISING);
  attachInterrupt(PARALLELOGRAM_SENSOR_PIN, Parallelogram_ISR, FALLING);
  
  for(int i = 1; i<8; i++){
    pinMode(actuations[i],OUTPUT);
  }

  // for turret sensor
  pinMode(TURRET_SENSOR_PIN, INPUT);  

  // for PWM
  TCCR1B = TCCR1B & mask | 0x02;

  // for Serial_Wait
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  // for PID
  input = 0;
  setpoint = 0;  
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(1);
  pid.SetOutputLimits(-15,15);

  Initialise();
  
//  Handshake_Launchpad();
  LAPTOP.println("Initialised");
  
  char temp = Serial_Wait();
  while( temp != 'q' ){
    if( temp == 't' ){
      Turret_Reset();
    }else if( temp == 'p' ){ 
     temp = Serial_Wait();
      if( temp == 'u' ) 
        Move_Parallelogram(FWD,1);
      else if( temp == 'd' )
        Move_Parallelogram(BCK,1);
    }else if( temp == 'P' ){
      temp = Serial_Wait();
      if( temp == 'u' ) {
        Parallelogram_Up();
        Serial_Wait();
        Parallelogram_Stop();
      }else if( temp == 'd' ){
        Parallelogram_Down();
        Serial_Wait();
        Parallelogram_Stop();
      }
    }else{
      LAPTOP.println("Enter t for turret reset or p for parallelogram reset or q to continue");
    }
    temp = Serial_Wait();
  }
  Query_Launchpad();
  Serial_Print();
  encoder_motor1 = encoder_motor2 = 0;
  LAPTOP.println("Encoder values reset.");
  LAPTOP.println("Commencing Auto Stage One");
}

void loop(){
/*  risedown();
  Serial_Wait();
  stoprise();
  Serial_Wait();
/  Move_Forward(15,15);
  while(!Serial.available()){
    Query_Launchpad();
    Serial_Print();
  }
  Motors_Brake(255,255);
  Serial.read();
  Serial_Wait();*/
  Toggle_Wait();
  LCD.clear();
  LCD.print("Stage One:");
/*  Move_Parallelogram(FWD,1);
  Parallelogram_Up();
  delay(300);
  Parallelogram_Stop();
*/
  if(digitalRead(A1)){
    Auto_Stage_One();
    LCD.clear();
    LCD.print("Stage Two:");
    Auto_Stage_Two();
  }else{
    Auto_Fallback();
    LineFollow_Fallback();
  }
  LAPTOP.println("Bot going into hibernation");
  while(1);
}

void Initialise(){
  // Code to read external byte and set the parameters respectively
  LAPTOP.println("No external byte found :P");
  mirror = true;
  if( mirror ){
    motor1.Attach(25,24,10);
    motor2.Attach(22,23,9);
    servo1.Attach(SERVO_RGT);
    servo2.Attach(SERVO_LFT);
    servo1.SetAngles(SERVO_ANG_R1F,SERVO_ANG_R2F,SERVO_ANG_R3F);
    servo2.SetAngles(SERVO_ANG_L1F,SERVO_ANG_L2F,SERVO_ANG_L3F);
    S1.Attach(A11);
    S2.Attach(A12);
    S3.Attach(A13);
    S4.Attach(A14);
  }else{
    motor1.Attach(22,23,9);
    motor2.Attach(25,24,10);
    servo1.Attach(SERVO_LFT);
    servo2.Attach(SERVO_RGT);
    servo1.SetAngles(SERVO_ANG_L1,SERVO_ANG_L2,SERVO_ANG_L3);
    servo2.SetAngles(SERVO_ANG_R1,SERVO_ANG_R2,SERVO_ANG_R3);
    S1.Attach(A14);
    S2.Attach(A13);
    S3.Attach(A12);
    S4.Attach(A11);
  }
  servo1.SetTargetAngle(1);
  servo2.SetTargetAngle(1);
  servo1.Home();
  servo2.Home();
}
