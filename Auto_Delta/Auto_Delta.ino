// Core AutoBot Code
// 1639 170213

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
    int flicker_count, threshold_reading_low, threshold_reading_high;
    boolean analog_read;
  public:
    Sensor(boolean analog){
      flicker_count = 10;
      analog_read = analog;
      threshold_reading_high = 900;
      threshold_reading_low = 300;
    }
    
    void Attach(uint8_t pin){
      input_pin = pin;
      pinMode(input_pin,INPUT);
    }
    
    boolean High(){
      if(analog_read)
        return High_Analog();
      else
        return High_Digital();
    }
    
    boolean High_Digital(){
      int reading = 0;
      for(int i = 0; i<flicker_count; i++)
        reading += digitalRead(input_pin);
      if(reading >= flicker_count/2)
        return true;
      else
        return false;
    }
    
    boolean High_Analog(){
      int reading = 0;
      for(int i = 0; i<flicker_count; i++)
        reading += analogRead(input_pin);
      reading /= flicker_count;
      if(reading >= threshold_reading_high)
        return true;
      else
        return false;
    }
    
    boolean Low(){
      if(analog_read)
        return Low_Analog();
      else
        return Low_Digital();
    }
    
    boolean Low_Digital(){
      int reading = 0;
      for(int i =0; i<flicker_count;i++)
          reading += !digitalRead(input_pin);
      if(reading>=flicker_count/2)
        return true;
      else
        return false;
    }
    
    boolean Low_Analog(){
      int reading = 0;
      for(int i = 0; i<flicker_count; i++)
        reading += analogRead(input_pin);
      reading /= flicker_count;
      if(reading <= threshold_reading_low)
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
    
    void Shift(int angle){
        servo.write((int)current_angle+angle);
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

#define SERVO_LFT 6
#define SERVO_RGT 7

#define TURRET_ENCODER_PIN 0
#define TURRET_SENSOR_PIN A10
#define PARALLELOGRAM_SENSOR_PIN 1
#define SHARP_SENSOR_PIN A2
#define PARALLELOGRAM_TRIP_SWITCH_BOTTOM 20
#define PARALLELOGRAM_TRIP_SWITCH_TOP A4

#define LEFT_VG 44
#define MIDDLE_VG 45
#define RIGHT_VG 46
#define V_PISTON 42
#define GRIPPER 43
#define PARALLELOGRAM_PIN1 39
#define PARALLELOGRAM_PIN2 40

#define VSOFTBRAKE 100
#define HARDBRAKE 255

#define STRAIGHT_LINE_PID 1
#define SOFT_TURN_PID 2

#define AUTO_PID 1
#define AUTO_FALLBACK 2

#define SERVO_ANG_L1 120
#define SERVO_ANG_L2 100
#define SERVO_ANG_L3 0
#define SERVO_ANG_R1 40
#define SERVO_ANG_R2 58
#define SERVO_ANG_R3 140

#define SERVO_ANG_L1F 135
#define SERVO_ANG_L2F 97
#define SERVO_ANG_L3F 48
#define SERVO_ANG_R1F 20
#define SERVO_ANG_R2F 55
#define SERVO_ANG_R3F 55

#define TURRET_ANG1 1180
#define TURRET_ANG2 1750
#define TURRET_ANG3 4680
#define TURRET_ANG4 6250
#define TURRET_ANG5 10330

#define TURRET_ANG1F 950
#define TURRET_ANG2F 1750
#define TURRET_ANG3F 4630
#define TURRET_ANG4F 6600
#define TURRET_ANG5F 10330

#define COMM_TSOP_1 4
#define COMM_TSOP_2 5

#define VSLOW 20
#define FAST 100
#define VFAST 200
#define NORMAL 60
#define SLOW 25
#define SLOWEST 0
#define TURRET_PWM 150

//LineFollow PWMs

#define LINEFOLLOW_STRAIGHT_HIGH 60
#define LINEFOLLOW_STRAIGHT_MODERATE 50
#define LINEFOLLOW_STRAIGHT_LOW 10

#define LINEFOLLOW_CURVE_HIGH 50
#define LINEFOLLOW_CURVE_MODERATE 40
#define LINEFOLLOW_CURVE_LOW 10

#define LINEFOLLOW_CURVE2_HIGH 70
#define LINEFOLLOW_CURVE2_MODERATE 40
#define LINEFOLLOW_CURVE2_LOW 10

#define LINEFOLLOW12_OUTER_MOTOR 35
#define LINEFOLLOW12_INNER_MOTOR 25
#define LINEFOLLOW12_34CORRECTION 15

#define LINEFOLLOW34_OUTER_MOTOR 20
#define LINEFOLLOW34_INNER_MOTOR 30
#define LINEFOLLOW34_12CORRECTION 15

#define LINEFOLLOW_STRAIGHT_PRECISION_HIGH 55
#define LINEFOLLOW_STRAIGHT_PRECISION_LOW 35

#define LINEFOLLOW_CURVE_PRECISION_HIGH 30 
#define LINEFOLLOW_CURVE_PRECISION_LOW 20


#define Move_Forward(pwm1, pwm2){\
  motor1.Control(FWD, pwm1);\
  motor2.Control(FWD, pwm2);\
}

#define Move_Back(pwm1, pwm2){\
  motor1.Control(BCK, pwm1);\
  motor2.Control(BCK, pwm2);\
}

#define Motors_Brake(pwm1, pwm2){\
  motor1.Brake(pwm1);\
  motor2.Brake(pwm2);\
}

#define Parallelogram_Up(){\
  digitalWrite(PARALLELOGRAM_PIN1, HIGH);\
  digitalWrite(PARALLELOGRAM_PIN2, LOW);\
}

#define Parallelogram_Down(){\
  digitalWrite(PARALLELOGRAM_PIN1, LOW);\
  digitalWrite(PARALLELOGRAM_PIN2, HIGH);\
}

#define Parallelogram_Stop(){\
  digitalWrite(PARALLELOGRAM_PIN1, LOW);\
  digitalWrite(PARALLELOGRAM_PIN2, LOW);\
}

#define Check_Abort(){\
  if( LAPTOP.available() )\
    Abort();\
}

// for Array of Functions
typedef void (*fn) (void);
fn Transform[] = {Initialise, Pick_Leaves, Accelerate_Bot, Decelerate_Bot, Drop_First_Leaf, Drop_Second_Leaf, Soft_Turn, Auto_Stage_One_Complete, Auto_Stage_Two};
fn Transform_Fallback[] = {Initialise, Pick_LeavesF, Initial_Straight_Line, Detect_Line,
                           Turn_and_Align, First_LineFollow, Drop_Two_Leaves, To_Last_Leaf, Final_Run_To_Leaf3, Drop_Last_Leaf,
                           To_First_Bud, To_Junction, To_Bud_Transfer, Transfer_Bud, To_Curve2, To_Next_Bud,
                           Tokyo2, To_Bud_Transfer, Transfer_Bud, To_Curve2, To_Next_Bud, Tokyo2, To_Bud_Transfer,
                           Transfer_Bud, The_End, Toggle_Wait };

/** Configuration Constants: Affect behaviour **/
uint16_t distances[26] = {0, 2030, 13880, 21650, 29100, 565, 150,100};
uint16_t distances_fallback[26] = {0, 2930, 13880, 17000, 500, 100};

/** Global declarations **/
Motor motor1, motor2, turret_motor(27, 26, 11); // the order of pin numbers determine the direction
Sensor S1(false), S2(false), S3(false), S4(false), turret_sensor(false), parallelogram_sensor(false);
LiquidCrystal LCD(13, 34, 30, 31, 32, 33);
Custom_Servo servo1, servo2;

const int actuations[] = {0, 39, 40, 42, 43, 44, 45, 46};
const int external_byte[] = {0, 14, 50, 52, 53, 51, 36, 15};

// for Servo
float servo_speeds[] = {0, 0.5, 0.75, 1.5, 0.25, 0.25, 0.25, 0.375, 0.5, 0};

// for PWM
const int mask = 0b11111000;

// for behaviour
int strategy = 1;  // 1 for 1-2-4 and 2 for 1-2-3
const int minimum_pwm = 20, maximum_pwm = 100, slowdown_pwm = 50;
const int acceleration_delay = 2, acceleration = 2;
const int deceleration_delay = 2, deceleration = 5;
const int encoder_count = 22134;  // Encoder count for hard turn 5802
const int delay_long = 5;
boolean stage_one_complete = false, mirror, ps_complete = false;
boolean skip_reset = false, omit_leaf1 = false, omit_leaf2 = false, omit_leaf3 = false;

// For PID
double setpoint, input, output;
double cons_kp = 1.0, cons_ki = 0, cons_kd = 0;
double aggressive_kp = 3.0, aggressive_ki = 0, aggressive_kd = 0;
PID pid(&input, &output, &setpoint, cons_kp, cons_ki, cons_kd, DIRECT);
int base_pwm, pid_type = STRAIGHT_LINE_PID;
boolean pid_enable = false;

// For encoder values
uint16_t diff_encoder_left = 0, diff_encoder_right = 0;
long int encoder_motor1 = 0, encoder_motor2 = 0;
long int volatile encoder_turret = 0;

// for line sensors
boolean linefollow_enable = false, line_detected = false;

// Core phase variables
int actuation_phase = 0, path_phase = 0;

// for Parallelogram and Turret
int parallelogram_count = 0, turret_count = 0;
int encoder_turret_target = 0;
int bud_count = 0;

long int prevmillis = 0;

void setup(){
  // //
  LAPTOP.begin(115200);
  LAUNCHPAD.begin(115200);

  LCD.begin(20,4);
  LCD.print("Hello World!");

  attachInterrupt(TURRET_ENCODER_PIN, Turret_ISR, RISING);
  attachInterrupt(PARALLELOGRAM_SENSOR_PIN, Parallelogram_ISR, FALLING);
  turret_sensor.Attach(TURRET_SENSOR_PIN);
  parallelogram_sensor.Attach(PARALLELOGRAM_SENSOR_PIN);  
  for(int i = 1; i<8; i++){
    pinMode(actuations[i], OUTPUT);
    pinMode(external_byte[i], INPUT);
  }

  // for PWM
  TCCR1B = TCCR1B & mask | 0x02;

  // for strategy Switch
  pinMode(A1, INPUT);
  
  // for communication TSOP
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  /*while(1) {
    Serial.print(4);
    Serial.print(" ");
    Serial.print(digitalRead(4));
    Serial.print("\t");
    Serial.print(5);
    Serial.print(" ");
    Serial.print(digitalRead(5)); 
    
    Serial.print("\t");
    Serial.print("Communication ");
    if(digitalRead(4) == LOW || digitalRead(5) == LOW)
      Serial.println("Yes");
    else
      Serial.println("No");
    delay(100);
  }*/
  
  // for PID
  input = 0;
  setpoint = 0;  
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(1);
  pid.SetOutputLimits(-15,15);

//  Handshake_Launchpad();

  Initialise();
  LAPTOP.println("Initialised");

  if(!skip_reset){
    char input = Serial_Wait();
    while( input != 'q' ){
      if( input == 'c' ){
        Check_Motors(100);
      }else if( input == 't' ){
        Turret_Reset();
      }else if( input == 'p' ){ 
        Parallelogram_Reset();
      }else if( input == 'P' ){
        input = Serial_Wait();
        if( input == 'u' ) {
          Parallelogram_Up();
          Serial_Wait();
          Parallelogram_Stop();
        }else if( input == 'd' ){
          Parallelogram_Down();
          Serial_Wait();
          Parallelogram_Stop();
        }
      }/*else if (input == 'm'){
        Auto_MSC();
      }*/else if( input == 'k'){
        Actuate_High(GRIPPER);
      }else if( input == 'l'){
        Actuate_Low(GRIPPER);      
      }else{
        LAPTOP.println("Enter t for turret reset or p for parallelogram reset or q to continue");
      }
      input = Serial_Wait();
    }
  }
  Parameters_Reset();
  LAPTOP.println("Here we begin =>");
 
  
}

void loop(){
  Toggle_Wait();
  
  LCD.clear();
  LCD.print("Stage One:");
  
  //code run in case parallelogram wire is unwound 
  
  Parallelogram_Up();  
  while(digitalRead(PARALLELOGRAM_TRIP_SWITCH_BOTTOM));
  Parallelogram_Stop();
  
  

  Parallelogram_Up();
  delay(150);
  Parallelogram_Stop();

  if(strategy == AUTO_PID ){
    LAPTOP.println("Commencing Auto PID ");
    Auto_Stage_One();
    LCD.clear();
    LCD.print("Stage Two:");
    Auto_Stage_Two();
  }else if( strategy == AUTO_FALLBACK ){
    LAPTOP.println("Commencing Auto Fallback ");
    Auto_Fallback();
  }
  LAPTOP.println("I am done. Thank you.");
  while(1);
}

void Read_External_Byte(){
  LAPTOP.println(digitalRead(14));
  if( digitalRead(14) == HIGH ){
    mirror = true;
    LAPTOP.println(" mirron arena ");
  }
  if( digitalRead(52) == HIGH ){
    omit_leaf1 = true;
    LAPTOP.println(" leaf1 to be omitted ");
  }
  if( digitalRead(53) == HIGH ){
    omit_leaf2 = true;
    LAPTOP.println(" leaf2 to be omitted ");
  }
  if( digitalRead(51) == HIGH ){
    omit_leaf3 = true;
    LAPTOP.println(" leaf3 to be omitted ");
  }
  if( digitalRead(50) == HIGH ){
    strategy = AUTO_PID;
    LAPTOP.println(" Straight line PID strategy ");
  }else{
    strategy = AUTO_FALLBACK;
    LAPTOP.println(" Fallback LineFollow strategy ");
  }
  if( digitalRead(36) == HIGH ){
    skip_reset = true;
    LAPTOP.println(" Skip reset ");
  }
}

void Initialise(){
  // Code to read external byte and set the parameters respectively
  Read_External_Byte();
  if( mirror ){
    motor1.Attach(25,24,10);
    motor2.Attach(22,23,9);
    servo1.Attach(SERVO_RGT);
    servo2.Attach(SERVO_LFT);
    if( strategy == AUTO_FALLBACK ){
      servo1.SetAngles(SERVO_ANG_R1F,SERVO_ANG_R2F,SERVO_ANG_R3F);
      servo2.SetAngles(SERVO_ANG_L1F,SERVO_ANG_L2F,SERVO_ANG_L3F);
    }else if( strategy == AUTO_PID ){
      servo1.SetAngles(SERVO_ANG_R1, SERVO_ANG_R2, SERVO_ANG_R3);
      servo2.SetAngles(SERVO_ANG_L1, SERVO_ANG_L2, SERVO_ANG_L3);
    }
    S1.Attach(A11);
    S2.Attach(A12);
    S3.Attach(A13);
    S4.Attach(A14);
  }else{
    motor1.Attach(22,23,9);
    motor2.Attach(25,24,10);
    servo1.Attach(SERVO_LFT);
    servo2.Attach(SERVO_RGT);
    if( strategy == AUTO_FALLBACK ){
      servo1.SetAngles(SERVO_ANG_L1F, SERVO_ANG_L2F, SERVO_ANG_L3F);
      servo2.SetAngles(SERVO_ANG_R1F, SERVO_ANG_R2F, SERVO_ANG_R3F);
    }else if( strategy == AUTO_PID ){
      servo1.SetAngles(SERVO_ANG_L1, SERVO_ANG_L2, SERVO_ANG_L3);
      servo2.SetAngles(SERVO_ANG_R1, SERVO_ANG_R2, SERVO_ANG_R3);
    }
    S1.Attach(A14);
    S2.Attach(A13);
    S3.Attach(A12);
    S4.Attach(A11);
  }
  servo1.Home();
  servo2.Home();
}
