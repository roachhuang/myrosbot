#include "TimerOne.h"
#include <PID_v1.h>

#define PI 3.14159265359
/*******************************************************************************
  Robot geometric parameters
*******************************************************************************/
#define TURNING_RADIUS WHEEL_SEPARATION / 2 // meter (HCR : 0.1375, BURGER : 0.080, WAFFLE : 0.1435)
const float r = 0.0325;                     // 3.25cm
const float L = 0.115;                      // 11.5cm
const double maxSpeed = 0.6;                // robot max speed in m/s
// left motor
const uint8_t ENA = 6;
const uint8_t IN1 = 7;
const uint8_t IN2 = 8;

/* Pins 9 and 10: controlled by timer 1; PWM pins: 3, 5, 6, 9, 10, 11
** right. servo.h uses timer1. Cannot include TimerOne.h & Servo.h at the same time.
*/
const uint8_t ENB = 11;
const uint8_t IN3 = 4;
const uint8_t IN4 = 5;

const uint8_t interruptPinL = 2;
const uint8_t interruptPinR = 3;
//Define Variables we'll be connecting to
volatile unsigned int counterL, counterR = 0;
double SetpointL, InputL, OutputL;
double SetpointR, InputR, OutputR;
volatile uint16_t rpmL, rpmR;

/*******************************************************************************
  Velocity parameters
*******************************************************************************/
// HCR
#define VELOCITY_CONSTANT_VALUE 272.83640729 // V = r * w = r * (RPM * 2 * pi / 60)
//   = 0.068 * RPM * 0.10472
// RPM = V * 140.43050375
#define SPOKE 20
#define TICK2RAD ((2 * PI) / SPOKE)    // 2 * pi / ENCODER_TICK
#define DEG2RAD(x) (x * 0.01745329252) // *PI/180
#define RAD2DEG(x) (x * 57.2957795131) // *180/PI
#deinfe tickPerMeter((2.0 * PI * r) / SPOKE)

//Specify the links and initial tuning parameters
// kpL and kpR?
double Kp = 0.53, Ki = 0.14, Kd = 0;
PID myPidL(&InputL, &OutputL, &SetpointL, Kp, Ki, Kd, DIRECT);
PID myPidR(&InputR, &OutputR, &SetpointR, Kp, Ki, Kd, DIRECT);

void encoderL() // counts from the speed sensor
{
  counterL++; // increase +1 the counter value
}

void encoderR() // counts from the speed sensor
{
  counterR++; // increase +1 the counter value
}

void timerIsr()
{
  Timer1.detachInterrupt(); //stop the timer
  rpmL = counterL * 3;      // (counter/spoke) * 60
  rpmR = counterR * 3;
  dL = counterL * tickPerMeter;
  dR = counterR * tickPerMeter;
  counterL = counterR = 0;  //  reset counter to zero asap
  dC = (dR + dL) / 2;
  phi = (dr - dL) / L;
  currentPhi = prevPhi + phi;
  currentX = preX + (dC * cos(w));
  currentY = preY + (dC * cos(w));

  Timer1.attachInterrupt(timerIsr); //enable the timer
}

void setup()
{
  Serial.begin(9600);
  // init motors
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
  rpmL = rpmR = counterL = counterR = 0;

  //  init encoders
  pinMode(interruptPinL, INPUT_PULLUP);
  pinMode(interruptPinR, INPUT_PULLUP);
  Timer1.initialize(1000000);                                              // set timer for 1sec
  attachInterrupt(digitalPinToInterrupt(interruptPinL), encoderL, RISING); // increase counter when speed sensor pin goes High
  attachInterrupt(digitalPinToInterrupt(interruptPinR), encoderR, RISING); // increase counter when speed sensor pin goes High
  Timer1.attachInterrupt(timerIsr);                                        // enable the timer

  //  init PIDs
  myPidL.SetSampleTime(1000); // adj this number to half makes difference. why?
  myPidL.SetMode(AUTOMATIC);
  myPidR.SetSampleTime(1000); // adj this number to half makes difference. why?
  myPidR.SetMode(AUTOMATIC);

  analogWrite(ENA, 0); // set speed of motor (0-255
  analogWrite(ENB, 0); // set speed of motor (0-255
  forward();
}

void loop()
{
  double v;
  // int potvalue = analogRead(1);  // Potentiometer connected to Pin A1
  // int motorspeed = map(potvalue, 0, 680, 255, 0);
  if (Serial.available())
  {
    v = Serial.parseInt();
    if (Serial.peek() == '\n')
      Serial.read(); // consume LF
    Serial.print("Set to: ");
    SetpointL = calSetpointL(v / 10, 0);
    SetpointR = calSetpointR(v / 10, 0);
    Serial.println(SetpointL);
    analogWrite(ENA, (int)SetpointL);
    analogWrite(ENB, (int)SetpointL);
  }
  InputL = rpmL;
  InputR = rpmR;

  Serial.print("inputL: ");
  Serial.println(InputL, DEC);
  Serial.print("inputR: ");
  Serial.println(InputR, DEC);
  myPidL.Compute();
  myPidR.Compute();
  analogWrite(ENA, (int)OutputL);
  analogWrite(ENB, (int)OutputR);
  Serial.print("outPutL: ");
  Serial.println(OutputL, DEC);
  Serial.print("OutputR: ");
  Serial.println(OutputR, DEC);
}

// input v in m/s, ret rpm
double calSetpointL(double v, double omega)
{
  double Vl;

  Vl = ((2 * v) - (omega * L)) / (2 * r);
  // RPM = (60/2*pi*r)*v
  // rpmR = Vr / 0.10472;  // new v linear is Vr*r = m/s
  return (Vl / 0.10472);
}

double calSetpointR(double v, double omega)
{
  double Vr;
  Vr = ((2 * v) + (omega * L)) / (2 * r); // now Vr is rad/s, r and L are in meter
  // RPM = (60/2*pi*r)*v
  return (Vr / 0.10472); // new v linear is Vr*r = m/s
}

void stop()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void forward()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}