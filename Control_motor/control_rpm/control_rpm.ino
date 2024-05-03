#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>


#define ENCODER_PIN 2
#define ENABLE 5
#define ENCODER_N   2064
#define PH 8


// Variables de ros
ros::NodeHandle nh;

std_msgs::Float32 flRPM_msg;

unsigned long T1 = 0, T2 = 0, T;
bool MeasDone = 0;
float Motor_RPM = 0;

// Respuestas del motor
long int pulsos = 0;
float posicion = 0.0;
float rpm = 0.0;

// Callback del subscriber
void pwmCallback(const std_msgs::Int16 & msg){
  int16_t pwm_signal;
  if(msg.data > 0){
    digitalWrite(PH, HIGH);
    pwm_signal = msg.data;
  } else if(msg.data < 0){
    digitalWrite(PH, LOW);
    pwm_signal = abs(msg.data);
  } else{
    pwm_signal = 0;
    Motor_RPM = 0;
  }
  analogWrite(ENABLE, pwm_signal);
}

ros::Publisher rpm_pub("rpm_pub", &flRPM_msg);

ros::Subscriber<std_msgs::Int16> pwm_listener("pwm_listener", &pwmCallback);

void INT0_ISR(void)
{
  if(MeasDone)
  {
    T2 = micros();
    T = T2 - T1;
    MeasDone = 0;
  }
  else
  {
    T1 = micros();
    MeasDone = 1;
  }
}
void setup()
{
  Serial.begin(9600);
  pinMode(ENCODER_PIN, INPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(PH, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), INT0_ISR, RISING);
    // Inicializacion de ROS
  nh.initNode();
  nh.advertise(rpm_pub);
  nh.subscribe(pwm_listener);
}
void loop()
{ 
  Motor_RPM = float((60000000) / (T * ENCODER_N));
  if(Motor_RPM <= 4){
    Motor_RPM = 0;
  }
  flRPM_msg.data = Motor_RPM;
  rpm_pub.publish(&flRPM_msg);
  
  nh.spinOnce();
  delay(10);
}