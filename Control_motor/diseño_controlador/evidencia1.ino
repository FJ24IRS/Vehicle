//JCŚ VERSION + DANYS VERSION
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>


rcl_subscription_t subscriber_pwm_duty_cycle;
std_msgs__msg__Int32 pwm_duty_cycle_msg;

rcl_publisher_t publisherADC;
rcl_publisher_t publisherPWM;
rcl_publisher_t publisherGAIN;
std_msgs__msg__Int32 ADC;
std_msgs__msg__Float32 PWM;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_1;
rcl_timer_t timer_2;
rcl_publisher_t publisherPPR;
std_msgs__msg__Int32 ppr_msg;

// PWM properties
const int freq = 5000;
const int ledChan = 0;
const int resolu = 8;
volatile int encoder_pulse_count = 0;

//rpm
volatile int pulsos = 0;
unsigned long timeold = 0;
rcl_timer_t tiempo_rpm;
std_msgs__msg__Int32 rpm;

int flanco = 0;
unsigned long tiempo_uno = 0;
unsigned long tiempo_dos = 0;
unsigned long resta_tiempo = 0;
float pwm_value = 0;

//control
//float ki = 13049.4344;
float kp = 0.1; //0.15
//float kd = 108.527799;
float ts = 0.01;
float gn  = 0;

float error = 0;
int error_anterior = 0;
float ref = 0;

float ref_vel = 0;

unsigned int timestamp_1 = 0;
unsigned int timestamp_2 = 0;
unsigned int delta = 0;

#define RIGHT_PIN 13
#define LEFT_PIN 12
#define PIN_36 36
#define PWM_PIN 15
#define encoder_A_pin 25
#define encoder_B_pin 23
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(RIGHT_PIN, !digitalRead(RIGHT_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer_1, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer_1 != NULL) {
    ADC.data = analogRead(PIN_36);
    //PWM.data = ADC.data*(3.3/4095.0);
  }
}

void timer_callback2(rcl_timer_t * timer_2, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer_2 != NULL) {
    RCSOFTCHECK(rcl_publish(&publisherADC, &error, NULL));
    RCSOFTCHECK(rcl_publish(&publisherPWM, &rpm, NULL));
    RCSOFTCHECK(rcl_publish(&publisherGAIN, &ref_vel, NULL));
    
  }
}

void subscription_callback(const void * msgin){  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  float pwm_value = msg->data;

  ref = pwm_value*255;
  ref_vel = pwm_value*100;
  pwm_value = pwm_value*255;
  error = ref - rpm.data * 3;

  gn = kp*error; //+ (kd/ts)*(error - error_anterior); //+ (ki*ts)*(error + error_anterior);
  pwm_value += gn;
  pwm_value = constrain(pwm_value + gn, -255, 255);

  /*if(ref > 245){
    ref = 255;
  }*/



  if(ref < 0){
    pwm_value = pwm_value * (-1);
    analogWrite (RIGHT_PIN, 0);
    analogWrite (LEFT_PIN, pwm_value);

  }

  else if(ref > 0){

    analogWrite(LEFT_PIN, 0);
    analogWrite(RIGHT_PIN, pwm_value);
  
  }

}

void IRAM_ATTR encoder_callback(){
    /*timestamp_2 = millis();
    if (timestamp_1 != 0) {
        delta = timestamp_1 - timestamp_2;
        // Aquí puedes usar delta para calcular RPM o lo que necesites
    }
    timestamp_1 = timestamp_2;
    */

    timestamp_1 = micros() - timestamp_2;
    timestamp_2 = micros();

    rpm.data = (60000000)/(timestamp_1 * 24 * 165);
}

void setup() {
  set_microros_transports();
  ledcSetup(ledChan, freq, resolu);
  ledcAttachPin(PWM_PIN, ledChan);
  
  pinMode(RIGHT_PIN, OUTPUT);
  pinMode(PIN_36, INPUT);
  pinMode(PWM_PIN, OUTPUT);
   pinMode(encoder_A_pin, INPUT_PULLUP); // Configura el pin del encoder como entrada
  attachInterrupt(digitalPinToInterrupt(encoder_A_pin), encoder_callback, CHANGE); // Adjunta la interrupción


  digitalWrite(RIGHT_PIN, HIGH);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_publisher_init_default(&publisherADC, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "error"));
  RCCHECK(rclc_publisher_init_default(&publisherPWM, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "rpm"));
  RCCHECK(rclc_publisher_init_default(&publisherGAIN, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "vel_ref"));
  
  RCCHECK(rclc_subscription_init_default(&subscriber_pwm_duty_cycle, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),"/signal"));

  const unsigned int timer_timeout_1 = 10;
  RCCHECK(rclc_timer_init_default(&timer_1, &support, RCL_MS_TO_NS(timer_timeout_1), timer_callback));

  const unsigned int timer_timeout_2 = 1000;
  RCCHECK(rclc_timer_init_default(&timer_2, &support, RCL_MS_TO_NS(timer_timeout_2), timer_callback2));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_2));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_pwm_duty_cycle, &pwm_duty_cycle_msg, &subscription_callback, ON_NEW_DATA));

  pinMode(encoder_A_pin, INPUT);
  pinMode(encoder_B_pin, INPUT);

  ADC.data = 0;
  PWM.data = 0.0;
}

void loop() {
  // Spin the executor to process callbacks
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10); // Small delay to prevent running the loop too fast

}