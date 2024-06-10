
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float64.h>

rcl_subscription_t coords_subscriber;
std_msgs__msg__Float64 desired_pos_msg;
std_msgs__msg__Float64 real_rpm;

std_msgs__msg__Float64 spins;
rcl_subscription_t rpm_subscriber;
std_msgs__msg__Int32 rpm_msg;

// rcl_subscription_t aruco_subscriber;
// std_msgs__msg__Bool aruco_msg;

rcl_subscription_t fusioned_subscriber;
std_msgs__msg__Float64 fusioned_distance_msg;

rcl_publisher_t distance_publisher;
std_msgs__msg__Float64 distance_msg;

rcl_publisher_t control_signal_publisher;
std_msgs__msg__Float64 control_msg;

rcl_publisher_t pos_reached_publisher;
std_msgs__msg__Bool pos_reached_msg;

rcl_publisher_t charge_battery_publisher;
std_msgs__msg__Bool charge_battery_msg;

std_msgs__msg__Int32 msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_1;
rcl_timer_t timer_2;


// Motor responses
volatile int dir = 1;
//float wheel_perimeter = 2 * 3.14159 * wheel_radius;
//float distance_per_count = wheel_perimeter / ENCODER_N;
// Estado de los encoders
volatile int desired_rpm = 0;
// Time between pulses
volatile unsigned long previous_time = 0;
volatile unsigned long current_time = 0;
volatile unsigned long pulse_duration = 0;
volatile long int pulsos = 0;
volatile bool control = true; // if false  speed control if true position control
// Control variables
const float Kp_speed = 2.68; // Proportional gain
const float Kd_speed = 0.009; // Derivative gain
const float Kp_pos = 35; // Proportional gain
const float Kd_pos = 0.3; // Derivative gain


float desired_pos = 0;
float error = 0, last_error = 0, rate_error = 0;
//float ts = 0.01;
double control_signal = 0;

const unsigned int timer_timeout1 = 10; // 1000 ms = 1 second
const unsigned int timer_timeout2 = 100; // 1000 ms = 0.01 second

const int sensorPin = A0;   // seleccionar la entrada para el sensor
int sensorValue;      // variable que almacena el valor raw (0 a 4095)
float value;   

const int csPin = 36; // Pin analógico conectado al pin CS (A1)
float currentSenseVoltage; // Voltaje leído del pin CS
float currentThreshold = 1.1; // Umbral de corriente en voltios para 1A
int currentSensorValue; // Valor leído del pin analógico

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Encoder pin
#define ENCODER_A 12 // YELLOW
#define ts 0.1

// Driver pins
#define PH 25 // Phase/Enable (dir)
#define EN 26  // PWM (Speed)


#define ENCODER_N 2061.48
#define WHEEL_D 0.00761


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}

void timer1_callback(rcl_timer_t * timer_1, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if(timer_1 != NULL){
    RCSOFTCHECK(rcl_publish(&distance_publisher, &distance_msg, NULL));
    RCSOFTCHECK(rcl_publish(&pos_reached_publisher, &pos_reached_msg, NULL));
    RCSOFTCHECK(rcl_publish(&control_signal_publisher, &control_msg, NULL));
  }
}

void timer2_callback(rcl_timer_t * timer_2, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if(timer_2 != NULL){
    sensorValue = analogRead(sensorPin);        // realizar la lectura
    value = fmap(sensorValue, 0, 4095, 0.0, 16.8);

    sensorValue = analogRead(csPin);
    currentSenseVoltage = sensorValue * (3.3 / 4095.0); // Ajusta según la resolución y referencia del ADC
    float current = currentSenseVoltage / 1.1; // 1.1V por A para DRV8874
    // if(current > 1){
    //   analogWrite(EN, 0);
    //   error_loop();
    // }
    if(value < 14.5){
      charge_battery_msg.data = true;
      RCSOFTCHECK(rcl_publish(&charge_battery_publisher, &charge_battery_msg, NULL));
    } else{
      charge_battery_msg.data = false;
    }

    
    if(control == true){
      error = (desired_pos/0.1570796325) - spins.data;
      rate_error = (error- last_error) / ts;
      control_signal = (Kp_pos * error + Kd_pos * rate_error); // + Ki * cum_error;
      control_msg.data = control_signal;
      last_error = error; 
      if(control_signal > -0.5 && control_signal < 0.5){
        control_signal = 0;
        pos_reached_msg.data = true;
      } else if (control_signal > -75 && control_signal < 75){
        if(control_signal<0){
          control_signal = -75;
        } else{
          control_signal = 75;
        }
      }
      control_signal = constrain(control_signal, -255, 255);

      if(control_signal >= 0){
        dir = 1;
        digitalWrite(PH, LOW);
        analogWrite(EN, control_signal);

      } else {
        dir=-1;
        digitalWrite(PH, HIGH);
        analogWrite(EN, -control_signal);
      }
    } 
    else {
      pos_reached_msg.data = false;
      if(desired_rpm != 0){
        error = (float(desired_rpm)*3.11) - real_rpm.data;
        rate_error = (error - last_error) / ts;
        control_signal = (Kp_speed * error + Kd_speed * rate_error); // + Ki * cum_error;
        control_msg.data = control_signal;
        last_error = error; 
        control_signal = constrain(control_signal, -255, 255);

        if(control_signal > 0){
          dir = 1;
          digitalWrite(PH, LOW);
          analogWrite(EN, control_signal);
        } else {
          dir = -1;
          digitalWrite(PH, HIGH);
          analogWrite(EN, -control_signal);
        }
      } else {
        analogWrite(EN, 0);
        real_rpm.data = 0;
      }
    }
  }
}



void rpm_callback(const void * msgin){
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  desired_rpm = msg->data;

}

// void aruco_callback(const void * msgin){
//   const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
//   control = msg->data;
// }

void desired_pos_callback(const void * msgin){
  const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
  desired_pos = msg->data;

}


void IRAM_ATTR encoder_isr(){
  current_time = micros();
  pulse_duration = current_time - previous_time;
  previous_time = current_time;
  pulsos+=dir;
  real_rpm.data = (60000000.0 / (pulse_duration * ENCODER_N)) * dir;
  spins.data = float(pulsos/ENCODER_N);
  distance_msg.data = spins.data*0.15707963267;
  
}

void setup(){
  set_microros_transports();
  ledcSetup(0, 5000, 8);
  ledcAttachPin(EN, 0);

  pinMode(ENCODER_A, INPUT);
  pinMode(PH, OUTPUT);
  pinMode(EN, OUTPUT);

  analogSetPinAttenuation(sensorPin, ADC_11db);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr, RISING);
  


  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "agv_node", "", &support));

  RCCHECK(rclc_publisher_init_default(&distance_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "distance"));
  RCCHECK(rclc_publisher_init_default(&pos_reached_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "pos_reached"));
  RCCHECK(rclc_publisher_init_default(&control_signal_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "control_signal"));


  //RCCHECK(rclc_subscription_init_default(&aruco_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "aruco_reached"));
  RCCHECK(rclc_subscription_init_default(&coords_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64), "go_to"));
  RCCHECK(rclc_subscription_init_default(&rpm_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "rpm"));


  RCCHECK(rclc_timer_init_default(&timer_1, &support, RCL_MS_TO_NS(timer_timeout1), timer1_callback));
  RCCHECK(rclc_timer_init_default(&timer_2, &support, RCL_MS_TO_NS(timer_timeout2), timer2_callback));

  rclc_executor_init(&executor, &support.context, 5, &allocator);
  RCCHECK(rclc_executor_add_timer(&executor, &timer_1));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_2));
  //RCCHECK(rclc_executor_add_subscription(&executor, &aruco_subscriber, &aruco_msg, &aruco_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &rpm_subscriber, &rpm_msg, &rpm_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &coords_subscriber, &desired_pos_msg, &desired_pos_callback, ON_NEW_DATA));

  distance_msg.data = 0;
  pos_reached_msg.data = false;

}

void loop(){
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}   