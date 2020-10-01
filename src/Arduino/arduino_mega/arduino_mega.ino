/*
  ENA LOW --> Nyala
  ENA HIGH --> Mati
  DIR LOW --> Clockwise
  DIR HIGH --> Counter Clockwise

  THIS CODE IS MADE FOR ARDUINO MEGA using the PWM controlled by Timer 5 !
  Don't forget to change the counter if you use another board !

  LogArduino.h:
    1) header
    2) steer
        a. setpoint (deg)
        b. actual (deg)
        c. delta (deg)
        d. absoolute_encoder
        e. state (-1: CCW || 0: IDLE || 1: CW)
    3) brake
        a. setpoint
        b. actual
        c. delta
        d. linear_encoder
        e. current (A)
        f. R_IS
        g. L_IS
        h. R_current (A)
        i. L_current (A)
        j. pwm
        k. state (0: IDLE || 1: PULL || -1: RELEASE)
    4) throttle
        a. setpoint (V)
        b. actual (V)
        c. state (0: Relay OFF || 1: Relay ON)

  Control.h:
    1) header
    2) steer
    3) throttle
    4) brake
 */

#include <ros.h>
#include <Wire.h>
#include <vehicle_control/Control.h>
#include <vehicle_control/Log_Arduino.h>
#include <Adafruit_MCP4725.h>
#define BAUD 500000

/********************** PIN CONFIGURATION *************************/
#define TRIGGER 22 // For communication purpose to the Arduino Nano
#define S1 23 // For communication purpose to the Arduino Nano
#define S2 24 // For communication purpose to the Arduino Nano
int ENCODER[ ] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11}; // Absolute Encoder Pin

/********************** PIN CONFIGURATION *************************/
#define RPWM 44 // See the counter at void setup()
#define LPWM 45 // See the counter at void setup()
#define LIN_ENC A1
#define PR_IS A2
#define PL_IS A3
#define THROTTLE_ENA A4
#define res_val 680.0f // Ohm (Value of The Resistor)
#define current_gain (8500.0f*4.95f/1023.0f/res_val)
#define OUT_5V 4.95f // (V) The output of the 5V pin used in this project (Robotdyn Arduino Mega 2560)
/******************************************************************/

/**********************TIMING*************************/
int update_steering_period = 10; // Milisekon, 100 Hz
int update_braking_period = 1000; // Microsecond, 1 kHz
int ros_period = 20; // Milisekon, 50 Hz
unsigned long update_steering_time = 0; // Milisecond
unsigned long update_braking_time = 0; // Microsecond
unsigned long ros_time = 0; // Milisecond
int check_stepper_stall_period = 150; // Milisekon, 10 Hz
int check_stepper_stall_num = check_stepper_stall_period / update_steering_period; // 10 count
int check_stepper_stall_count = 0;
/****************************************************/

/************* STEERING GLOBAL VARIABLE *************/
float steering_angle = 0; // Degree
float steering_setpoint = 0;
float steering_delta_min_move = 0.3; // degree (must be less than stay)
float steering_delta_min_stay = 0.5; // degree (must be greater than move)
float check_stepper_stall_min_val = 0.1; // degree
float check_stepper_stall_last_steer_angle = 0.00; // degree
bool steering_moving = false;
bool s1 = LOW;
bool s2 = LOW;
bool s1_prev = HIGH;
bool s2_prev = HIGH;
float angle = 0;
float delta_steer = 0;
int steering_state = 0; // (-1: CCW || 0: IDLE || 1: CW)
#define max_steer 28.0f
#define min_steer -35.0f
#define steering_gradient 0.09053716f
#define steering_bias -42.12905785f
/***************************************************/

/************* BRAKING GLOBAL VARIABLE *************/
float braking_setpoint = 0; // cm
float braking_position = 0; // cm
float braking_delta_min_move = 0.025; // cm (must be less than stay)
float braking_delta_min_stay = 0.100; // cm (must be greater than move)
float braking_zero_offset = 0.0f;
bool braking_moving = false;
int braking_state = 0; // 0: IDLE || 1: PULL || -1: RELEASE
#define max_brake 3.0f
#define min_brake 0.0f
#define breaking_gradient 0.01012f
#define breaking_bias 0.0f
//#define bf_coeff 0.75f
#define bf_coeff 0.f
/***************************************************/

/************* THROTTLE GLOBAL VARIABLE *************/
float throttle_setpoint = 0; // (0 - 1)
float throttle_voltage = 0; // (0 - 4095) setpoint
float throttle_value = 0; // (0 - 4095) actual voltage sent
float throttle_increment = float(update_steering_period)/1000.0f * 4095.0f / 2.0f; // From 0 to 1 (Max. val.) ~ Needs 2 seconds
int throttle_state = 0; // 0: Relay OFF || 1: Relay ON
Adafruit_MCP4725 dac;
/***************************************************/

ros::NodeHandle  nh;
vehicle_control::Log_Arduino pub_msg;
ros::Publisher pub("logging_arduino", &pub_msg);

void receive_message(const vehicle_control::Control& sub_msg) {
  steering_setpoint = min(max(sub_msg.action_steer, min_steer), max_steer);
  pub_msg.steer.setpoint = steering_setpoint;

  braking_setpoint = min(max(sub_msg.action_brake, min_brake), max_brake);
  pub_msg.brake.setpoint = braking_setpoint;

  throttle_setpoint = min(max(sub_msg.action_throttle, 0), 1);
  pub_msg.throttle.setpoint = throttle_setpoint * OUT_5V; // (V)
}

ros::Subscriber<vehicle_control::Control> sub("control_signal", &receive_message );

void setup() {
  // For Arduino Mega: Set PWM frequency for D44, D45 & D46
  // set timer 5 divisor to 8 for PWM frequency of 3921.16 Hz
  TCCR5B = TCCR5B & B11111000 | B00000010; 

  pinMode(TRIGGER, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  for (int i = 0; i < 10; i++) {
    pinMode(ENCODER[i], INPUT_PULLUP);
  }

  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  braking_state = 0; // STOP
  throttle_state = 0; // Relay OFF

  nh.getHardware()-> setBaud(BAUD);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);

  dac.begin(0x60); 
  dac.setVoltage(0, true);
  pinMode(THROTTLE_ENA, OUTPUT);

  pub_msg.header.frame_id = "/log_arduino_mega";
  pub_msg.steer.setpoint = steering_setpoint;
  pub_msg.brake.setpoint = braking_setpoint;
  pub_msg.steer.state = steering_state;
  pub_msg.brake.state = braking_state;
  pub_msg.throttle.setpoint = throttle_setpoint * OUT_5V;
  pub_msg.throttle.state = throttle_state;
}

void loop() {
  if ((millis() - update_steering_time) >= update_steering_period) { // Steering angle control
    update_steering_time = millis();
    
    sensing_steering();
    process_steering();
    process_throttling();
  }

  if ((micros() - update_braking_time) >= update_braking_period){
    update_braking_time = micros();
    
    sensing_braking_position();
    //sensing_braking_current(); // Kalau ga dipake, taruh di loop ros aja
    process_braking();
    
  }

  if ((millis() - ros_time) >= ros_period) { // Publish the pub_msg
    ros_time = millis();
    
    sensing_braking_current();
    pub_msg.header.stamp = nh.now();
    pub.publish( &pub_msg);
  }
  nh.spinOnce();
}

void sensing_steering() {
  angle = 0;
  for (int i = 0; i < 10; i++) {
    if (!digitalRead(ENCODER[i])){
      angle +=  pow(2, i);
    }
  }
  
  pub_msg.steer.absolute_encoder = int(angle + 0.5);
  
  //preprocessing
  if(angle<500) { angle += 1024;}
  angle -= 500;
  
  steering_angle = steering_gradient*angle + steering_bias; // convert encoder to angle (ackerman)
  pub_msg.steer.actual = steering_angle;
  
}

void process_steering() {
  delta_steer = steering_setpoint - steering_angle;

  pub_msg.steer.delta = delta_steer;

  if (!steering_moving && abs(delta_steer) >= steering_delta_min_stay){
    steering_moving = true;
  }
  else if (steering_moving && abs(delta_steer) <= steering_delta_min_move){
    steering_moving = false;
  }

  if(steering_moving){
    if (delta_steer >= 0) { // KANAN (CW)
      steering_state = 1;
      s1 = HIGH;
      s2 = LOW;
    }
    else{ // KIRI (CCW)
      steering_state = -1;
      s1 = LOW;
      s2 = HIGH;
    }
  }
  else{
    steering_state = 0; // (IDLE)
    s1 = LOW;
    s2 = LOW;
  }

  // Check, whether the stepper motor is stalling or not
  if (steering_moving){
    check_stepper_stall_count += 1;
    if ( (s1 != s1_prev) || (s2 != s2_prev) ){
      // If the direction is changed, we don't need to pay attention
      // because the timing will be reset in the Arduino Nano
      check_stepper_stall_last_steer_angle = steering_angle;
      check_stepper_stall_count = 0;
    }
    else{
      if (check_stepper_stall_count >= check_stepper_stall_num){ // if the count is equal to num (10)
        if (abs(steering_angle - check_stepper_stall_last_steer_angle) < check_stepper_stall_min_val){ // STALL !
          digitalWrite(S1, LOW);
          digitalWrite(S2, LOW);
          delayMicroseconds(10);
          digitalWrite(TRIGGER, HIGH);
          delayMicroseconds(10);
          digitalWrite(TRIGGER, LOW);
          delay(1);
        }
        check_stepper_stall_last_steer_angle = steering_angle;
        check_stepper_stall_count = 0;
      }
    }
  }
  else{ // If the stepper is not moving, the stepper won't be stalling
    check_stepper_stall_last_steer_angle = steering_angle;
    check_stepper_stall_count = 0;
  }
  
  //if ( (s1 != s1_prev) || (s2 != s2_prev) ) {
  digitalWrite(S1, s1);
  digitalWrite(S2, s2);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER, LOW);
  s1_prev = s1;
  s2_prev = s2;

  pub_msg.steer.state = steering_state;
}

void sensing_braking_position(){
  float lin_dist = analogRead(LIN_ENC);
  pub_msg.brake.linear_encoder = lin_dist;

  lin_dist = breaking_gradient*lin_dist + breaking_bias;
  pub_msg.brake.actual = lin_dist;
  braking_position = bf_coeff*braking_position + (1.0f - bf_coeff)*lin_dist;
}

void sensing_braking_current(){
  float current_read;
  
  current_read = analogRead(PR_IS);
  pub_msg.brake.R_IS = current_read;
  current_read = current_read * current_gain ; // A
  pub_msg.brake.R_current = current_read;

  current_read = analogRead(PL_IS);
  pub_msg.brake.L_IS = current_read;
  current_read = current_read * current_gain ; // A
  pub_msg.brake.L_current = current_read;
}

void process_braking(){
  int pwm = 0;
  int sign = 1;
  
  float delta_brake = braking_setpoint - braking_position;
  pub_msg.brake.delta = delta_brake;
  
  if (!braking_moving && abs(delta_brake) >= braking_delta_min_stay){
    braking_moving = true;
  }
  else if (braking_moving && abs(delta_brake) <= braking_delta_min_move){
    braking_moving = false;
  }  
  
  if(braking_moving){
    pwm = 255;
    if (braking_setpoint == 0.0){ pwm = 180; }
    else if (braking_setpoint <= 0.25 * max_brake){ pwm = 100; }
    else if (braking_setpoint <= 0.375 * max_brake){ pwm = 110; }
    else if (braking_setpoint <= 0.60 * max_brake){ pwm = 130; }
    else if (braking_setpoint <= 0.75 * max_brake){ pwm = 150; }
    else if (braking_setpoint <= 0.80 * max_brake){ pwm = 175; }
    else if (braking_setpoint <= 0.85 * max_brake){ pwm = 200; }

    if(delta_brake >= 0){
      braking_state = 1; // PULL
      sign = 1;
      analogWrite(LPWM, 0);
      analogWrite(RPWM, pwm);
    }
    else{
      braking_state = -1; // RELEASE
      sign = -1;
      analogWrite(RPWM, 0);
      analogWrite(LPWM, pwm);
    }
  }
  else {
    braking_state = 0; // IDLE
    pwm = 0;
    
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
  
  pub_msg.brake.pwm = pwm * sign;
  pub_msg.brake.state = braking_state;
}

void process_throttling(){  
  throttle_voltage = floor(throttle_setpoint * 4095.0f);
  
  throttle_value += throttle_increment;
  if (throttle_value >= throttle_voltage){
    throttle_value = throttle_voltage;
  }
   
  // ENABLE
  if(throttle_voltage > 0 || throttle_value > 0){ //perlu cari deadband throttle
    digitalWrite(THROTTLE_ENA, HIGH);
    throttle_state = 1;
  } else {
    digitalWrite(THROTTLE_ENA, LOW);
    throttle_state = 0;
  }
  pub_msg.throttle.state = throttle_state;

  // THROTTLE COMMAND
  dac.setVoltage(floor(throttle_value), false);
  pub_msg.throttle.actual = float(throttle_value) * OUT_5V / 4095.0f; // (V)
}
