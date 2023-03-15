#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ROS libraries
#include <ros.h>
#include <std_msgs/Int16.h>

// Initialize ros node as nh
ros::NodeHandle nh;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN   60    // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX   500   // This is the 'maximum' pulse length count (out of 4096)
#define USMIN      600   // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX      2400  // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50    // Analog servos run at ~50 Hz updates


class ServoMotor {
public:
  // Initialize class variables
  int16_t servo_angle      = SERVOMIN;
  int16_t fake_servo_angle = SERVOMIN;
  uint8_t servonum         = 1;
  char cmd_vel_topic[30]   = "";

  // As ros::Subscriber has no default constructor, it relies on ServoMotor's constructor.
  // So it needs to be initialized outside the constructor or it'll loop, i.e. needs a 
  // ready-constructor to start, but it's also called when readying the constructor.)
  ros::Subscriber<std_msgs::Int16, ServoMotor> servo_sub;

  // Servo callback
  void servoCb(const std_msgs::Int16& servo_msg){
    fake_servo_angle += servo_msg.data;

    if (SERVOMIN < fake_servo_angle && fake_servo_angle < SERVOMAX){
      servo_angle += servo_msg.data;
      pwm.setPWM(servonum, 0, servo_angle);
    }
  }

  // Constructor
  ServoMotor(uint8_t servoNumber)
  // Known as member initialisation list syntax (more efficient, but less readable)
  : servo_sub("diff_angle/servo_1", &ServoMotor::servoCb, this),
    servonum(servoNumber)
  {
    snprintf(cmd_vel_topic, 30, "diff_angle/servo_%d", servoNumber);
    //nh.initNode();
    nh.subscribe(servo_sub);
  }
};


void setup() {
  Serial.begin(57600);  // CANNOT use Serial.println when using rosserial

  // Initialize PWM servo driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  // Reset all servo motors back to 0 deg.
  int i = 0;
  for (i = 0; i < 16; i++)
  {
    pwm.setPWM(i, 0, SERVOMIN);
    delay(100);
  }

  //Initiate ROS nodes and subscribers.
  nh.initNode();

  ServoMotor servo1(1);
  ServoMotor servo2(2);
  ServoMotor servo3(3);
  ServoMotor servo4(4);
  
  delay(10);
}


void loop() {
  nh.spinOnce();
  delay(10);
}