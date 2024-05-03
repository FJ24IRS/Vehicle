#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#define ENCODER_PIN 2
#define ENABLE 5
#define ENCODER_N 2064
#define PH 8

// ROS Variables
ros::NodeHandle nh;

std_msgs::Float32 flRPM_msg;

unsigned long T1 = 0, T2 = 0, T;
bool MeasDone = 0;
float Motor_RPM = 0;
int sentido=1;
// Motor Response Variables
long int pulsos = 0;
float posicion = 0.0;
float rpm = 0.0;

// PD gains
const float Kp = 2.64; // Proportional gain, adjust as needed
const float Kd = 0.009; // Derivative gain, adjust as needed

int desired_rpm = 0;
int last_error = 0;
unsigned long last_time = 0;

// Callback for RPM target
void rpmTargetCallback(const std_msgs::Int16& msg) {
    desired_rpm = msg.data;
}

// Callback for PWM adjustment
void pwmCallback(int outputSignal) {
    int16_t pwm_signal;
    if (outputSignal > 0) {
        digitalWrite(PH, HIGH);
        pwm_signal = outputSignal;
        sentido = 1;
    } else if (outputSignal < 0) {
        digitalWrite(PH, LOW);
        pwm_signal = abs(outputSignal);
        sentido = -1;
    } else {
        pwm_signal = 0;
        Motor_RPM = 0;
    }
    analogWrite(ENABLE, pwm_signal);
}

ros::Publisher rpm_pub("rpm_pub", &flRPM_msg);
ros::Subscriber<std_msgs::Int16> rpm_target_subscriber("rpm_target", &rpmTargetCallback);

void INT0_ISR(void) {
    if (MeasDone) {
        T2 = micros();
        T = T2 - T1;
        MeasDone = 0;
    } else {
        T1 = micros();
        MeasDone = 1;
    }
}

void setup() {
    Serial.begin(9600);
    pinMode(ENCODER_PIN, INPUT);
    pinMode(ENABLE, OUTPUT);
    pinMode(PH, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), INT0_ISR, RISING);
    nh.initNode();
    nh.advertise(rpm_pub);
    nh.subscribe(rpm_target_subscriber);
}

void loop() {
    Motor_RPM = float((60000000) / (T * ENCODER_N));
    if (Motor_RPM > 100000){
      Motor_RPM = 0;
    }
    flRPM_msg.data = Motor_RPM * sentido;
    rpm_pub.publish(&flRPM_msg);

    // PD Control Calculations
    unsigned long current_time = millis();
    float dt = (current_time - last_time) / 1000.0; // Time in seconds
    if (dt >= 0.01) { // Update every 10 ms
        int error = desired_rpm - Motor_RPM;
        int derivative = (error - last_error) / dt;
        int output = Kp * error + Kd * derivative;

        // Create PWM message and send PWM signal
        pwmCallback(output);

        // Update for next iteration
        last_error = error;
        last_time = current_time;
    }

    nh.spinOnce();
    delay(10);
}