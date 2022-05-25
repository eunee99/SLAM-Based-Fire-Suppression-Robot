#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

////////////////// Tick Data Publishing Variables and Constants ///////////////
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 18
#define ENC_IN_RIGHT_A 20

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 19
#define ENC_IN_RIGHT_B 21

boolean Direction_left = true; // forward
boolean Direction_right = true; // forward

// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

////////////////// Motor Controller Variables and Constants ///////////////////

// Motor A connections(LEFT)
const int enA = 2;
const int in1 = 38;
const int in2 = 40;

// Motor B connections(RIGHT)
const int enB = 3;
const int in3 = 42;
const int in4 = 44;

// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;

// Number of ticks per wheel revolution. We won't use this in this code.
const int TICKS_PER_REVOLUTION = 4028;

// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER = 9872; // 1 / (2*pi*WHEEL_RADIUS) * TICKS_PER_REVOLUTION

// Proportional constant, which was measured by measuring the
// PWM-Linear Velocity relationship for the robot.
const int K_P = 250;

// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 40;

// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120;

// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 40;

// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 40;
const int PWM_MAX = 255;

// Set linear velocity and PWM variable values for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;

double pwmLeftReq = 0;
double pwmRightReq = 0;

// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;


float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}
/////////////////////// Tick Data Publishing Functions ////////////////////////

// Increment the number of ticks
void right_wheel_tick()
{
  // Read the value for the encoder for the right wheel
  int rval = digitalRead(ENC_IN_RIGHT_B);
  if (rval == LOW)
  {
    Direction_right = false; // Reverse
  }
  else
  {
    Direction_right = true; // Forward
  }

  if (Direction_right) // Forward
  {

    if (right_wheel_tick_count.data == encoder_maximum)
    {
      right_wheel_tick_count.data = encoder_minimum;
    }

    else
    {
      right_wheel_tick_count.data++;
    }
  }

  else // Backward
  {
    if (right_wheel_tick_count.data == encoder_minimum)
    {
      right_wheel_tick_count.data = encoder_maximum;
    }

    else
    {
      right_wheel_tick_count.data--;
    }
  }
}

// Increment the number of ticks
void left_wheel_tick()
{
  // Read the value for the encoder for the left wheel
  int lval = digitalRead(19);

  if (lval == LOW)
  {
    Direction_left = true; // Forward
  }
  else 
  {
    Direction_left = false; // Reverse
  }


  if (Direction_left)
  {
    if (left_wheel_tick_count.data == encoder_maximum)
    {
      left_wheel_tick_count.data = encoder_minimum;
    }
    
    else
    {
      left_wheel_tick_count.data++;
    }
  }

  else
  {
    if (left_wheel_tick_count.data == encoder_minimum)
    {
      left_wheel_tick_count.data = encoder_maximum;
    }
    
    else
    {
      left_wheel_tick_count.data--;
    }
  }
}

/////////////////////// Motor Controller Functions ////////////////////////////

// Calculate the left wheel linear velocity in m/s every time a
// tick count message is rpublished on the /left_ticks topic.
void calc_vel_left_wheel() {
  // Previous timestamp
  static double prevTime = 0;

  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;

  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;

  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  // Calculate wheel velocity in meters per second
  velLeftWheel = numOfTicks / TICKS_PER_METER / ((millis() / 1000) - prevTime);

  // Keep track of the previous tick count
  prevLeftCount = left_wheel_tick_count.data;

  // Update the timestamp
  prevTime = (millis() / 1000);

}

// Calculate the right wheel linear velocity in m/s every time a
// tick count message is published on the /right_ticks topic.
void calc_vel_right_wheel() {

  // Previous timestamp
  static double prevTime = 0;

  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;

  // Manage rollover and rollunder when we get outside the 16-bit integer range
  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;

  if (numOfTicks > 10000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  // Calculate wheel velocity in meters per second
  velRightWheel = numOfTicks / TICKS_PER_METER / ((millis() / 1000) - prevTime);

  prevRightCount = right_wheel_tick_count.data;

  prevTime = (millis() / 1000);

}

// cmd_vel callback
void calc_pwm_values(const geometry_msgs::Twist& msg) {
    // These variables will hold our desired PWM values
   
  float l = (msg.linear.x - msg.angular.z) / 2;
  float r = (msg.linear.x + msg.angular.z) / 2;

  if(msg.linear.x == 0)
  {
   if (msg.angular.z > 0) { // go right
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
   }
  else { // go left
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  }
  
   
  else { // go forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }

  uint16_t lpwm = mapPwm(fabs(l), PWM_MIN, PWM_MAX);
  uint16_t rpwm = mapPwm(fabs(r), PWM_MIN, PWM_MAX);

  analogWrite(enA, lpwm);
  analogWrite(enB, rpwm);
}

// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );

void setup() {

  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);

  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  // Motor control pins are outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  // Set the motor speed
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
}

void loop() {

  nh.spinOnce();

  // Record the time
  currentMillis = millis();

  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval) {

    previousMillis = currentMillis;

    // Publish tick counts to topics
    leftPub.publish( &left_wheel_tick_count );
    rightPub.publish( &right_wheel_tick_count );

    // Calculate the velocity of the right and left wheels
    calc_vel_right_wheel();
    calc_vel_left_wheel();

  }

  // Stop the car if there are no cmd_vel messages
  if ((millis() / 1000) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
}
