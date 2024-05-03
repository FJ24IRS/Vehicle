#include <ros.h>
#include <TimerOne.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

// --------------------- PINS DEFINITIONS ---------------------
// Encoder pin
#define ENCODER_A 2 // YELLOW
#define ENCODER_B 3 // WHITE

// Driver pins
#define PH 8 // Phase/Enable (Direction)
#define EN 5  // PWM (Speed)

// Pulses per revolution (only counting rising edges of A)
#define ENCODER_N 2064

// ------------ VARIABLES DEFINITIONS AND INITIALIZATION ------------
// Motor responses
int direction = 0;
float current_rpm = 0.0;

// Time between pulses
unsigned long previous_time = 0;
unsigned long current_time = 0;
unsigned long time = 0;

// Control variables
const float Kp = 2.64; // Proportional gain
const float Kd = 0.009; // Derivative gain
float desired_rpm = 0;
float error = 0, last_error = 0, rate_error = 0, cum_error = 0;
float ts = 0.1;
int16_t control_signal = 0;

// --------- ROS VARIABLES AND PUBLISHERS/SUBSCRIBERS DEFINITIONS ---------
// ROS variables
ros::NodeHandle nh;

// Type of messages to be published
std_msgs::Float32 flRPM_msg; // To see rpm values
std_msgs::Int16 IntPWM_msg; // To check pwm values

// Subscriber callback (receives desired RPM)
void rpmTargetCallback(const std_msgs::Float32 &msg){
  desired_rpm = msg.data * 3.11;
}

// ROS publishers and subscriber
ros::Publisher rpm_pub("rpm_pub", &flRPM_msg);
ros::Publisher pwm_pub("pwm_pub", &IntPWM_msg);

ros::Subscriber<std_msgs::Float32> rpm_target_sub("rpm_target", &rpmTargetCallback);

// --------------------- FUNCTIONS ---------------------
// Adjusts the PWM signal according to the control signal
void pwmAdjustment(int16_t controlSignal){
  int16_t pwm_signal = 0;
  if(controlSignal > 0){ // Forward
    digitalWrite(PH, HIGH);
    pwm_signal = controlSignal;
    direction = 1;
  } else if(controlSignal < 0){ // Backward
    digitalWrite(PH, LOW);
    pwm_signal = abs(controlSignal);
    direction = -1;
  } else { // Stop
    digitalWrite(PH, LOW);
    current_rpm = 0;
    pwm_signal = 0;
  }
  analogWrite(EN, pwm_signal);
}

// -------------------------- MAIN CODE --------------------------
void setup(){
  // Driver signals
  pinMode(PH, OUTPUT);
  pinMode(EN, OUTPUT);
  
  // Encoder signals
  pinMode(ENCODER_A, INPUT);
  pinMode(ENCODER_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderAISR, RISING);

  // Timer initialization
  Timer1.initialize(10000); // 10 ms
  Timer1.attachInterrupt(timerISR);

  // ROS initialization
  nh.initNode();
  nh.advertise(rpm_pub);
  nh.advertise(pwm_pub);
  nh.subscribe(rpm_target_sub);

}

void loop(){
  // Calculate rpm
  if(time == 0){
    current_rpm = 0;
  } else {
    current_rpm = float((60000000) / (time * ENCODER_N)) * direction;
  }
  flRPM_msg.data = current_rpm * direction;

  // Calculate control signal and send it to genetate PWM signal
  pwmAdjustment(control_signal);
  IntPWM_msg.data = control_signal;

  // Publish rpm and pwm values
  rpm_pub.publish(&flRPM_msg);
  pwm_pub.publish(&IntPWM_msg);

  // Spin ROS
  nh.spinOnce();
  delay(10);
}

// --------------------- ISR ---------------------
void encoderAISR(){
  current_time = micros();
  time = current_time - previous_time;
  previous_time = current_time;
}


void timerISR(){
  error = desired_rpm - current_rpm;
  rate_error = (error - last_error) / ts;
  // cum_error = (cum_error + error) * ts;
  control_signal = Kp * error + Kd * rate_error; // + Ki * cum_error;
  last_error = error;  

  // Send PWM control signal
  // pwmAdjustment(control_signal);
}
