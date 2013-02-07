// Core AutoBot Code
// 06-01-2013
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
#include <location.h>

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
    int n; 
  public:
    Sensor(){
      n = 10;
    }
    
    void Attach(uint8_t pin){
      input_pin = pin;
      pinMode(input_pin,INPUT);
    }
    
    boolean High(){
      int reading = 0;
      for(int i =0; i<n;i++)
       reading += digitalRead(input_pin);
      if(reading>=n/2)
          return true;
      else
          return false;
  }
    
   boolean Low(){
        int reading = 0;
      for(int i =0; i<n;i++)
       reading += !digitalRead(input_pin);
      if(reading>=n/2)
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

    void Sweep(int servo_speed){
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
#define SERVO_ANG_L1 110
#define SERVO_ANG_L2 100
#define SERVO_ANG_L3 0
#define SERVO_ANG_R1 40
#define SERVO_ANG_R2 55
#define SERVO_ANG_R3 140

#define TURRET_SENSOR_PIN A10
#define TURRET_DELAY 200
#define PARALLELOGRAM_SENSOR_PIN 3

#define LEFT_VG 41
#define MIDDLE_VG 39
#define RIGHT_VG 44
#define V_PISTON 40
#define GRIPPER 37

#define VSLOW 20
#define FAST 100
#define VFAST 200
#define VSOFTBRAKE 100
#define HARDBRAKE 255

#define TURRET_ANG1 1080
#define TURRET_ANG2 1750
#define TURRET_ANG3 2100
#define TURRET_ANG4 1700
#define TURRET_ANG5 3400
#define STRAIGHT_LINE_PID 1
#define SOFT_TURN_PID 2

#define Move_Forward(left_pwm,right_pwm){\
  left_motor.Control(FWD,left_pwm);\
  right_motor.Control(FWD,right_pwm);\
}

#define Move_Back(left_pwm,right_pwm){\
  left_motor.Control(BCK,left_pwm);\
  right_motor.Control(BCK,right_pwm);\
}

#define Motors_Brake(left_pwm,right_pwm){\
  left_motor.Brake(left_pwm);\
  right_motor.Brake(right_pwm);\
}

// for Array of Functions
typedef void (*fn) (void);
fn Transform[] = {Initialise, Pick_Leaves, Accelerate_Bot, Decelerate_Bot, Drop_Right_Leaf, Drop_Middle_Leaf, Soft_Turn, Auto_Stage_One_Complete};

/** Configuration Constants: Affect behaviour **/
uint16_t distances[26] = {0, 2930, 13880, 21200, 30100, 565, 90};

/** Global declarations **/
Motor left_motor(22, 23, 9), right_motor(25, 24, 10), turret_motor(27, 26, 11); // the order of pin numbers determine the direction
Motor motor1, motor2;

//Sensor L1(A13),L2(A14),R1(A12),R2(A11);
Sensor L1, L2, R1, R2;
const int actuations[] = {0, 37, 40, 44, 39, 41, 48, 49};

int NORMAL = 60;
int SLOW = 25;
int SLOWEST = 0; 

// for Servo
Custom_Servo servo_left, servo_right;
float servo_speeds[] = {0, 0.5, 0.5, 0.5, 0.25, 0.25, 0.25, 0.375, 0.5, 0};

// for PWM
const int mask = 0b11111000;

// for behaviour
const int minimum_pwm = 20, maximum_pwm = 100, slowdown_pwm = 50;
const int acceleration_delay = 2, acceleration = 2;
const int deceleration_delay = 2, deceleration = 5;
const int stratergy = 1;
const int encoder_count = 22134;  // Encoder count for hard turn 5802
const int delay_long = 5;
boolean stage_one_complete = false, mirror;

// For PID
double setpoint, input, output;
double cons_kp = 1.0, cons_ki = 0, cons_kd = 0;
double aggressive_kp = 3.0, aggressive_ki = 0, aggressive_kd = 0;
PID pid(&input, &output, &setpoint, cons_kp, cons_ki, cons_kd, DIRECT);
int base_pwm, pid_type = STRAIGHT_LINE_PID;
boolean pid_enable = 0;

// For encoder values
uint16_t diff_encoder_left = 0, diff_encoder_right = 0;
long int encoder_left = 0, encoder_right = 0;
long int encoder_motor1 = 0, encoder_motor2 = 0;
long int volatile encoder_turret = 0;

// for line sensors
boolean linefollow_enable = false, line_detected = false;

// Core phase variables
int actuation_phase = 0, path_phase = 0;

boolean volatile parallelogram_reset = false;
long int prevmillis = 0;

// for stage two

ENCODER Encoders(0);
ENCODER Parallelogram(1,Parallelogram_ISR,FALLING);
MOTOR Parallelogram_Motor(48,49);

//void Parallelogram_ISR(){Parallelogram.Calculate();}

#define riseup(){\
  Parallelogram_Motor.Back(0);\
}
#define risedown(){\
  Parallelogram_Motor.Forward(0);\
}
#define stoprise(){\
  Parallelogram_Motor.Stop();\
}


void setup(){
  LAPTOP.begin(115200);
  LAUNCHPAD.begin(115200);
  attachInterrupt(0, Turret_ISR, RISING);

  for(int i = 1; i<8; i++){
    pinMode(actuations[i],OUTPUT);
  }

  // for encoders
  pinMode(TURRET_SENSOR_PIN, INPUT);  
  pinMode(PARALLELOGRAM_SENSOR_PIN, INPUT);

  // for PWM
  TCCR1B = TCCR1B & mask | 0x02;

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
        riseup();
        Serial_Wait();
        stoprise();
      }else if( temp == 'd' ){
        risedown();
        Serial_Wait();
        stoprise();
      }
    }else{
      LAPTOP.println("Enter t for turret reset or p for parallelogram reset or q to continue");
    }
    temp = Serial_Wait();
  }
  Query_Launchpad();
  Serial_Print();
  encoder_left = encoder_right = 0;
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
  //Auto_Stage_One();
  //Serial_Wait();
  //LAPTOP.println("uncomment stage two");
  Auto_Stage_Two_Mirror();
  LAPTOP.println("Bot going into hibernation");
  while(1);
}

void Initialise(){
  // Code to read external byte and set the parameters respectively
  LAPTOP.println("No external byte found :P");
  mirror = false;
  servo_left.Attach(Check_Mirror(SERVO_RGT,SERVO_LFT));
  servo_right.Attach(Check_Mirror(SERVO_LFT,SERVO_RGT));
  // add code for set angles
  if( mirror ){
    servo_left.SetAngles(SERVO_ANG_R1,SERVO_ANG_R2,SERVO_ANG_R3);
    servo_right.SetAngles(SERVO_ANG_L1,SERVO_ANG_L2,SERVO_ANG_L3);
    L1.Attach(A12);
    L2.Attach(A11);
    R1.Attach(A13);
    R2.Attach(A14);
  }else{
    servo_left.SetAngles(SERVO_ANG_L1,SERVO_ANG_L2,SERVO_ANG_L3);
    servo_right.SetAngles(SERVO_ANG_R1,SERVO_ANG_R2,SERVO_ANG_R3);
    L1.Attach(A13);
    L2.Attach(A14);
    R1.Attach(A12);
    R2.Attach(A11);
  }
  servo_left.SetTargetAngle(1);
  servo_right.SetTargetAngle(1);
  servo_left.Home();
  servo_right.Home();
  motor1 = mirror?right_motor:left_motor;
  motor2 = mirror?left_motor:right_motor;
}
