#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ROS libraries
#include <ros.h>
#include <std_msgs/Int16.h>

// Initialize ros node as nh
ros::NodeHandle nh;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN   50    // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX   500   // This is the 'maximum' pulse length count (out of 4096)
#define USMIN      600   // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX      2400  // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50    // Analog servos run at ~50 Hz updates


class ServoMotor {
  // Initialize class variables
  int16_t servo_angle      = SERVOMIN;
  int16_t fake_servo_angle = SERVOMIN;
  uint8_t servonum         = 1;

public:
  // As ros::Subscriber has no default constructor, it relies on ServoMotor's constructor.
  // So it needs to be initialized outside the constructor or it'll loop, i.e. needs a 
  // ready-constructor to start, but it's also called when readying the constructor.)
  ros::Subscriber<std_msgs::Int16, ServoMotor> servo_sub;

  // Servo callback
  void servoCb(const std_msgs::Int16& servo_msg){
    this->fake_servo_angle += servo_msg.data;

    if (SERVOMIN < this->fake_servo_angle && this->fake_servo_angle < SERVOMAX){
      this->servo_angle += servo_msg.data;
      pwm.setPWM(this->servonum, 0, this->servo_angle);
    } else {
      this->fake_servo_angle -= servo_msg.data;
    }
  }

  // Constructor
  ServoMotor(uint8_t servoNumber, const char* topicName)
  // Known as member initialisation list syntax (more efficient, but less readable)
  : servo_sub(topicName, &ServoMotor::servoCb, this),
    servonum(servoNumber-1)
  {
    nh.subscribe(servo_sub);
  }
};


// Initialize servo motor classes
ServoMotor servo1(1, "diff_angle\/servo_1");
ServoMotor servo2(3, "diff_angle\/servo_3");
ServoMotor servo3(5, "diff_angle\/servo_5");
ServoMotor servo4(7, "diff_angle\/servo_7");


void setup() {
  Serial.begin(57600);  // CANNOT use Serial.println when using rosserial

  //Initiate ROS nodes.
  nh.initNode();

  // Initialize PWM servo driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  
  delay(10);
}



bool firstRun = true;
void loop() {
  // Reset all servo motors back to 0 deg when beginning.
  if (firstRun) {
    for (uint8_t i = 0; i < 16; i++) {
      pwm.setPWM(i, 0, SERVOMIN);
      delay(100);
    }
    firstRun = false;
  }

  nh.spinOnce();
}
