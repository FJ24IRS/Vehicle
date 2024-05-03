#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

// Encoder pins
#define ENCODER_A 2 // YELLOW
#define ENCODER_B 3 // WHITE

// Driver pins
#define PH 8// Phase/Enable (Direction)
#define EN 5 // PWM (Speed)

// Encoder states (Use mainly to check for direction and position)
int encoderA_state, encoderB_state, prev_state = 0;

// Motor responses
int direction = 0;
long int pulses = 0;
float position = 0.0;
float rpm = 0.0;

// Time between pulses
unsigned long previous_time = 0;
unsigned long current_time = 0;
unsigned long time;

// ROS variables
ros::NodeHandle nh;

std_msgs::Float32 flPOS_msg;
std_msgs::Float32 flRPM_msg;

// Subscriber callback
void pwmCallback(std_msgs::Int16 &msg){
  int16_t pwm_signal = 0;
  if(msg.data > 0){ // Forward
    digitalWrite(PH, HIGH);
    pwm_signal = abs(msg.data);
  }else if(msg.data < 0){ // Backward
    digitalWrite(PH, LOW);
    pwm_signal = abs(msg.data);
  }else{
    digitalWrite(PH, LOW);
    pwm_signal = 0;
    rpm = 0;
  }
  analogWrite(EN, pwm_signal);
}

// ROS publishers and subscribers
ros::Publisher spin_pub("spin_pub", &flPOS_msg);
ros::Publisher rpm_pub("rpm_pub", &flRPM_msg);

ros::Subscriber<std_msgs::Int16> pwm_listener("pwm_listener", &pwmCallback);

void setup() {  
  // Driver signals
  pinMode(PH, OUTPUT);
  pinMode(EN, OUTPUT);

  // Encoder signals
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderB_check, CHANGE);

  // ROS initialization
  nh.initNode();
  nh.advertise(spin_pub);
  nh.advertise(rpm_pub);
  nh.subscribe(pwm_listener);
}

void loop(){
  flPOS_msg.data = position;
  flRPM_msg.data = rpm * direction;

  spin_pub.publish(&flPOS_msg);
  rpm_pub.publish(&flRPM_msg);

  nh.spinOnce();
  delay(10);
}

void encoderA_ISR(){
  encoderA_state = digitalRead(ENCODER_A);
  if(encoderA_state == 1){
    current_time = micros();
    time = current_time - previous_time;
    previous_time = current_time;
    rpm = 60000000 / (time * 2064);
  }
}

void encoderB_check(){
  encoderB_state = digitalRead(ENCODER_B);
  if(encoderB_state != prev_state){
    if(encoderA_state == encoderB_state){
      direction = -1;
      if(encoderB_state == 1){
        pulses++;
      }
    }else if(encoderA_state != encoderB_state){
      direction = 1;
      if(encoderB_state == 0){
        pulses--;
      }
    }
    position = float(pulses) / 2064.0;
  }
  prev_state = encoderB_state;
}
