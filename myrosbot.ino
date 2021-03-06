
/*
   rosserial Subscriber Example
   roscore
   rosrun rosserial_python serial_node.py /dev/ttyACM0
   rosrun rosserial_python serial_node.py /dev/ttyUSB0

   rosrun teleop_twist_keyboard teleop_twist_keyboard.py
*/

#include <ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// #include "TimerOne.h"
#include <PID_v1.h>

/*
 * Declaration Globales
 */
geometry_msgs::Range range_msg;
nav_msgs::Odometry odom;

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

#define TRIG 2
#define ECHO 3
#define LASH 15   // damage distance of obstacle
#define SAMPLE_TIME 100  // 100ms

const uint8_t interruptPinL = 2;
const uint8_t interruptPinR = 3;
//Define Variables we'll be connecting to
volatile unsigned int counterL, counterR = 0;
double SetpointL, InputL, OutputL;
double SetpointR, InputR, OutputR;
volatile uint16_t rpmL, rpmR;
unsinged long lastMs = 0;

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
#define tickPerMeter ((2.0 * PI * r) / SPOKE)
typedef struct
{
    float x;
    float y;
    float theta;
} xy_theta;

xy_theta pose;
double actVel=0, theta = 0;
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

void Odom()
{
    double dL, dR, dC;

    // Timer1.detachInterrupt(); //stop the timer
    rpmL = counterL * (3000/SAMPLE_TIME);      // (counter/spoke) * 60s * 1000ms/SAMPLE_TIME
    rpmR = counterR * (3000/SAMPLE_TIME);
    dL = counterL * tickPerMeter;
    dR = counterR * tickPerMeter;
    counterL = counterR = 0; //  reset counter to zero asap

    dC = (dR + dL) / 2.0;
    actVel = dC * (1000/SAMPLE_TIME);  // m/s 
    pose.x += dC * cos(pose.theta);
    pose.y += dC * sin(pose.theta);
    theta = (dR - dL) / L;
    pose.theta += theta;
    // constrain theta in [-pi, pi]
    pose.theta = atan2(sin(pose.theta), cos(pose.theta));

    // Timer1.attachInterrupt(timerIsr); //enable the timer
}

void cmd_velCb(const geometry_msgs::Twist &cmd_vel)
{
    double v, w;
    v = cmd_vel.linear.x;
    w = cmd_vel.angular.z;

    // uni_to_diff to RPM
    SetpointL = (((2.0 * v) - (w * L)) / (2.0 * r)) * 9.5493;
    SetpointR = (((2.0 * v) + (w * L)) / (2.0 * r)) * 9.5493;
}

ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

ros::Publisher pub_range("range_data", &range_msg);
ros::Publisher pub_odom("Odometry", &odom);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmd_velCb);

unsigned long range_timer;

/*
   getRange() - samples the analog input from the ranger
   and converts it into meters.

   NOTE: This function is only applicable to the HC-SR04 !!
   Using this function with other Rangers will provide incorrect readings.
*/
int getRange()
{
    digitalWrite(TRIG, HIGH);
    digitalWrite(TRIG, LOW);
    int dist = pulseIn(ECHO, HIGH) / 50;
    return dist;
}

void setup()
{
    // Serial.begin(57600);

    // init ros
    nh.initNode();
    // nh.advertise(pub_range);
    nh.advertise(pub_odom);
    nh.subscribe(sub);

    /* init ir_ranger
      pinMode(TRIG, OUTPUT);
      pinMode(ECHO, INPUT);
      range_timer = 0;
      range_msg.radiation_type = sensor_msgs::Range::INFRARED;
      range_msg.header.frame_id = "utlrasound";
      range_msg.field_of_view = 0.10000000149;
      range_msg.min_range = 0.02;
      range_msg.max_range = 4; // in meter
    */
    // init odom
    odom.header.frame_id="/odom");
    odom.child_frame_id = "/base_link";

    // init motors
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENB, OUTPUT);
    rpmL = rpmR = counterL = counterR = 0;
    pose.x = pose.y = pose.theta = 0;

    //  init encoders
    pinMode(interruptPinL, INPUT_PULLUP);
    pinMode(interruptPinR, INPUT_PULLUP);
    // Timer1.initialize(1000000);                                              // set timer for 1sec
    attachInterrupt(digitalPinToInterrupt(interruptPinL), encoderL, RISING); // increase counter when speed sensor pin goes High
    attachInterrupt(digitalPinToInterrupt(interruptPinR), encoderR, RISING); // increase counter when speed sensor pin goes High
    // Timer1.attachInterrupt(timerIsr);                                        // enable the timer

    //  init PIDs
    myPidL.SetSampleTime(SAMPLE_TIME); // adj this number to half makes difference. why?
    myPidL.SetMode(AUTOMATIC);
    myPidR.SetSampleTime(SAMPLE_TIME); // adj this number to half makes difference. why?
    myPidR.SetMode(AUTOMATIC);

    analogWrite(ENA, 0); // set speed of motor (0-255
    analogWrite(ENB, 0); // set speed of motor (0-255
    forward();
}

void loop()
{
    /*
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
      setpoint are set in cmd_velCb
      SetpointL = calSetpointL(v, w);
      SetpointR = calSetpointR(v, w);
    */

    // publish the range value every 50 milliseconds
      //   since it takes that long for the sensor to stabilize
    if ((millis() - lastMs) > SAMPLE_TIME) // run pid every 100ms
    {
        InputL = rpmL;
        InputR = rpmR;
        myPidL.Compute();
        myPidR.Compute();
        analogWrite(ENA, (int)OutputL);
        analogWrite(ENB, (int)OutputR);
        /*
        range_msg.range = getRange();
        range_msg.header.stamp = nh.now();
        pub_range.publish(&range_msg);
        */
        odom();
        t.transform.translation.x = pose.x;
        t.transform.translation.y = pose.y;
        t.transform.rotation = tf::createQuaternionFromYaw(pose.theta);

        odom.header.stamp = nh.now();
        odom.pose.pose.position.x = pose.x;
        odom.pose.pose.position.y = pose.y;
        odom.pose.pose.position.z = pose.theta;
        odom.pose.pose.orientation = odom_quat;

        // output Velocity
        odom.twist.twist.linear.x = actVel;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = theta;
        pub_odom.publish(&odom);
        broadcaster.sendTransform(t);
        lastMs = millis();
    }
    /*
      Serial.print("inputL: ");
      Serial.println(InputL, DEC);
      Serial.print("inputR: ");
      Serial.println(InputR, DEC);
    */

    /*
      Serial.print("outPutL: ");
      Serial.println(OutputL, DEC);
      Serial.print("OutputR: ");
      Serial.println(OutputR, DEC);
    */

    nh.spinOnce();
    // delay(10);
}
/*
double calSetpointR(double v, double omega)
{
  double Vr;
  Vr = ((2 * v) + (omega * L)) / (2 * r); // now Vr is rad/s, r and L are in meter
  // RPM = (60/2*pi*r)*v
  // return (Vr / 0.10472); // new v linear is Vr*r = m/s
  return Vr * 9.5493;
}
*/

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
