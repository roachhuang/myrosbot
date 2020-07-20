#include "TimerOne.h"
#include <PID_v1.h>

volatile unsigned int counter = 0;

//int b1a = 6;  // L9110 B-1A
//int b1b = 9;  // L9110 B-1B
const unsigned int IN1 = 7;
const unsigned int IN2 = 8;
const unsigned int ENA = 6;
const unsigned int interruptPin = 2;
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
unsigned int rpm;

//Specify the links and initial tuning parameters
double Kp = 0.5205, Ki = 0.14, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void docount()  // counts from the speed sensor
{
  counter++;  // increase +1 the counter value
}

void timerIsr()
{
  Timer1.detachInterrupt();  //stop the timer
  // Serial.print("Motor Speed: ");
  // int rotation = (counter*60 / 20);  // divide by number of holes in Disc
  rpm = counter * 3; // divide by number of holes in Disc
  Serial.println(rpm, DEC);
  // Serial.println(" Rotation per seconds");
  counter = 0; //  reset counter to zero
  Timer1.attachInterrupt( timerIsr );  //enable the timer
}

void setup()
{
  Serial.begin(9600);
  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  rpm = counter = 0;

  Timer1.initialize(1000000); // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt(interruptPin), docount, RISING);  // increase counter when speed sensor pin goes High
  Timer1.attachInterrupt( timerIsr ); // enable the timer
  Setpoint = 77;

  //turn the PID on
  myPID.SetSampleTime(1000);  // adj this number to half makes difference. why?
  myPID.SetMode(AUTOMATIC);
  analogWrite(ENA, 0);  // set speed of motor (0-255
  forward();
}

void loop()
{
  // int i;
  // int potvalue = analogRead(1);  // Potentiometer connected to Pin A1
  // int motorspeed = map(potvalue, 0, 680, 255, 0);
  if (Serial.available()) {
    Setpoint = Serial.parseInt();
    if (Serial.peek() == '\n') Serial.read();  // consume LF
    Serial.print("Set to: ");
    Serial.println(Setpoint);
  }
  Input = rpm;
  if (myPID.Compute()) {
    Serial.print("output: ");
    Serial.println(Output, DEC);
    analogWrite(ENA, (int)Output);  // set speed of motor (0-255
  }
}

void stop()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  //digitalWrite(IN3, LOW);
  //digitalWrite(IN4, LOW);
}
void forward()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  //digitalWrite(IN3, HIGH);
  //digitalWrite(IN4, LOW);
}
