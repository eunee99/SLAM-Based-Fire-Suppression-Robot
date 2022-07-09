#include <ros.h>
#include <Servo.h> 
#include <std_msgs/Int16.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>

#define ENC_IN_LEFT_A 18
#define ENC_IN_RIGHT_A 20

#define ENC_IN_LEFT_B 19
#define ENC_IN_RIGHT_B 21

#define P_GAIN 25
#define I_GAIN 5

#define WHEEL_RADIUS 0.065 // 0.065 meter
#define ROBOT_WIDTH 0.375 // 0.375 meter

Servo servo;

const double pi = 3.141592;

boolean Direction_right = true;
boolean Direction_left = true;

// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

long previousMillis = 0;

long currentMillis = 0;

const int interval = 30;

boolean light_sel = true;

// Motor A connections(LEFT)
const int enA = 2;
const int in1 = 38;
const int in2 = 40;

// Motor B connections(RIGHT)
const int enB = 3;
const int in3 = 42;
const int in4 = 44;

// Servo Motor Connection
const int servo_motor = 11;

// Head light Connection
const int light = 8;

const int TICKS_PER_REVOLUTION = 4400; // original 4028
const int TICKS_PER_METER = 11000; // 2.43 revolution * 4400(TICKS_PER_REVOLUTION) (0.41m per revolution --> 1m / 0.41m == 2.43 revolution)

// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 50;
const int PWM_MAX = 255;

// previous encoder value
int prev_left_enc = 0;
int prev_right_enc = 0;

// current_encoder_data - prev_encoder_data
int diff_left_enc = 0;
int diff_right_enc = 0;

// measured rpm by encoder 
double enc_left_rpm = 0;
double enc_right_rpm = 0;

// calculated rpm by cmd_vel
double target_left_rpm = 0;
double target_right_rpm = 0;

// target(calculated)_rpm - measured_rpm 
double left_realError = 0;
double right_realError = 0;

// accumulate error
double left_accError = 0;
double right_accError = 0;

// P Control
unsigned int left_pControl = 0;
unsigned int right_pControl = 0;

// I Control
unsigned int left_iControl = 0;
unsigned int right_iControl = 0;

// PI Control
unsigned int left_piControl = 0;
unsigned int right_piControl = 0;

// Current PWM Value
unsigned int curr_pwm_l = 0;
unsigned int curr_pwm_r = 0;

// Previous PWM Value
unsigned int last_pwm_l = 0;
unsigned int last_pwm_r = 0;


// ROS Setting
ros::NodeHandle nh;

std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);


// P Control System
void pControl_System() {
  left_pControl = P_GAIN * left_realError;
  right_pControl = P_GAIN * right_realError;
}

// I Control System
void iControl_System() {
  left_iControl = I_GAIN * (left_accError * 0.033);
  right_iControl = I_GAIN * (right_accError * 0.033);
}

// PI Control System
void piControl_System() {
   left_piControl = left_pControl + left_iControl;
   right_piControl = right_pControl + right_iControl;
}


unsigned int pwm_limit(unsigned int x)
{
  if (x >= PWM_MAX)
  {
    x = PWM_MAX;
  }

  else if (x <= PWM_MIN)
  {
    x = PWM_MIN;
  }

  else;
  
  return x;
}

//////////////// ROS Publisher Function ////////////////
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
  
  diff_right_enc = right_wheel_tick_count.data - prev_right_enc;
  enc_right_rpm = (60 * diff_right_enc) / (TICKS_PER_REVOLUTION * 0.033 * 2);
  prev_right_enc = right_wheel_tick_count.data;
}


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
  
  diff_left_enc = left_wheel_tick_count.data - prev_left_enc;
  enc_left_rpm = (60 * diff_left_enc) / (TICKS_PER_REVOLUTION * 0.033 * 2);
  prev_left_enc = left_wheel_tick_count.data;
}
////////////////////////////////////////////////////


//////////////// ROS Subscriber Callback Function ////////////////
void calc_rpm_values(const geometry_msgs::Twist& msg) {
  
  target_left_rpm = ((msg.linear.x * 60) - (msg.angular.z * 60 * ROBOT_WIDTH / 2)) / (2 * pi * WHEEL_RADIUS);
  target_right_rpm = ((msg.linear.x * 60 + msg.angular.z * 60 * ROBOT_WIDTH / 2)) / (2 * pi * WHEEL_RADIUS);
  
  left_realError = target_left_rpm - enc_left_rpm;
  right_realError = target_right_rpm - enc_right_rpm;

  left_accError += left_realError;
  right_accError += right_realError;

  pControl_System();
  iControl_System();
  piControl_System();

  // set motor direction
  if (msg.linear.x == 0) 
  {
   
   if (msg.angular.z > 0) { // go right
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
   }
   
  else if (msg.angular.z < 0) { // go left 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }

  else // stop
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  }
   
  else { // go forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
   
}

void fire_launcher(const std_msgs::Int8 &msg)
{
  if(msg.data == 1)
  {
    servo.write(90);
    delay(3000);
  }
  
  else;
}

void set_pwm_values()
{
  curr_pwm_l = pwm_limit(last_pwm_l + left_piControl);
  curr_pwm_r = pwm_limit(last_pwm_r + right_piControl);
  
  analogWrite(enA, curr_pwm_l);
  analogWrite(enB, curr_pwm_r);

  last_pwm_l = curr_pwm_l;
  last_pwm_r = curr_pwm_r;
}
////////////////////////////////////////////////////

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_rpm_values );
ros::Subscriber<std_msgs::Int8> fire("launch_flag", &fire_launcher);

void setup() {

  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT_PULLUP);

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
  pinMode(light, OUTPUT);
  servo.attach(servo_motor);

  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  servo.write(0);

  
  for(int i = 0; i < 3; i++)
  {
    digitalWrite(light, light_sel);
    delay(1000);
    light_sel = ~light_sel;
  }

  // Set the motor speed
  analogWrite(enA, 0);
  analogWrite(enB, 0);

  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
  nh.subscribe(fire);
}

void loop() {

  nh.spinOnce(); // call subscriber callback function

  currentMillis = millis();
  
  if (currentMillis - previousMillis > interval) 
  {
    leftPub.publish(&left_wheel_tick_count);
    rightPub.publish(&right_wheel_tick_count);
    previousMillis = currentMillis;
  }

  set_pwm_values();
}
