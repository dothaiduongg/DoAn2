

#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

////////////////// Tick Data Publishing Variables and Constants ///////////////

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
// Nano hỗ trợ ngắt trên chân 2 và 3 (INT0, INT1) -> sua chan enc left va right -> done
#define ENC_IN_LEFT_A 3
#define ENC_IN_RIGHT_A 2

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 12
#define ENC_IN_RIGHT_B 4

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;

// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
// dang ky topic trong ROS
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);

std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;

////////////////// Motor Controller Variables and Constants ///////////////////

// Motor A connections
const int enA = 11;
const int in1 = 10;
const int in2 = 9;

// Motor B connections
const int enB = 5;
const int in3 = 7;
const int in4 = 6;

// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;

// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER = 2880;  // Originally 2880

// Proportional constant, which was measured by measuring the
// PWM-Linear Velocity relationship for the robot.
const int K_P = 200;

// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 52;

// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 100;

// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 100;

// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 100;
const int PWM_MAX = 150;

// Set linear velocity and PWM variable values for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;

// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;

/////////////////////// Tick Data Publishing Functions ////////////////////////

// Increment the number of ticks
void right_wheel_tick() {
  int val = digitalRead(ENC_IN_RIGHT_B);
  Direction_right = (val != LOW);

  if (Direction_right) {
    right_wheel_tick_count.data = (right_wheel_tick_count.data == encoder_maximum) ? encoder_minimum : right_wheel_tick_count.data + 1;
  } else {
    right_wheel_tick_count.data = (right_wheel_tick_count.data == encoder_minimum) ? encoder_maximum : right_wheel_tick_count.data - 1;
  }
}

// Increment the number of ticks
void left_wheel_tick() {
  int val = digitalRead(ENC_IN_LEFT_B);
  Direction_left = (val == LOW);

  if (Direction_left) {
    left_wheel_tick_count.data = (left_wheel_tick_count.data == encoder_maximum) ? encoder_minimum : left_wheel_tick_count.data + 1;
  } else {
    left_wheel_tick_count.data = (left_wheel_tick_count.data == encoder_minimum) ? encoder_maximum : left_wheel_tick_count.data - 1;
  }
}

/////////////////////// Motor Controller Functions ////////////////////////////

void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
  lastCmdVelReceived = (millis() / 1000);

  pwmLeftReq = K_P * (cmdVel.linear.x - cmdVel.angular.z) + b;
  pwmRightReq = K_P * (cmdVel.linear.x + cmdVel.angular.z) + b;

  if (cmdVel.linear.x < 0) {
    pwmLeftReq = -abs(pwmLeftReq);
    pwmRightReq = -abs(pwmRightReq);
  }

  pwmLeftReq = constrain(pwmLeftReq, -PWM_MAX, PWM_MAX);
  pwmRightReq = constrain(pwmRightReq, -PWM_MAX, PWM_MAX);

  if (abs(pwmLeftReq) < PWM_MIN) pwmLeftReq = 0;
  if (abs(pwmRightReq) < PWM_MIN) pwmRightReq = 0;

  String logLeft = "Left PWM: " + String(pwmLeftReq);
  String logRight = "Right PWM: " + String(pwmRightReq);
  nh.loginfo("print: -------");
  nh.loginfo(logLeft.c_str());
  nh.loginfo(logRight.c_str());
}

void set_pwm_values() {
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;

  if (pwmLeftReq * pwmLeftOut < 0) pwmLeftOut = 0;
  if (pwmRightReq * pwmRightOut < 0) pwmRightOut = 0;

  if (pwmLeftReq > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (pwmLeftReq < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  if (pwmRightReq > 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (pwmRightReq < 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  pwmLeftOut = min(PWM_MAX, max(-PWM_MAX, pwmLeftReq));
  pwmRightOut = min(PWM_MAX, max(-PWM_MAX, pwmRightReq));

  analogWrite(enA, abs(pwmLeftOut));
  analogWrite(enB, abs(pwmRightOut));
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values);

void setup() {
  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  analogWrite(enA, 0);
  analogWrite(enB, 0);

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
}

void loop() {
  nh.spinOnce();
  currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    leftPub.publish(&left_wheel_tick_count);
    rightPub.publish(&right_wheel_tick_count);
  }

  if ((millis() / 1000) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  set_pwm_values();
}
