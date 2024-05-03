#define ENCODER_PIN 2
#define ENABLE 5
#define ENCODER_N   2064
unsigned long T1 = 0, T2 = 0, T;
bool MeasDone = 0;
int Motor_RPM = 0;
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
  pinMode(8, OUTPUT);
  digitalWrite(8, HIGH);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), INT0_ISR, RISING);
  analogWrite(ENABLE, 255);
}
void loop()
{ 
  Motor_RPM = (60000000) / (T * ENCODER_N);
  Serial.print("Motor RPM: ");
  Serial.println(Motor_RPM);
  delay(250);
}