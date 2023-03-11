#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ROS libraries
#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  60 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  500 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;
char val;
int16_t servo_angle1 = SERVOMIN;
int16_t fake_servo_angle1 = SERVOMIN;
int16_t servo_angle2 = SERVOMIN;
int16_t fake_servo_angle2 = SERVOMIN;


void servoCb1(const std_msgs::Int16& servo_1){
  fake_servo_angle1 += servo_1.data;
  
  if (SERVOMIN < fake_servo_angle1 && fake_servo_angle1 < SERVOMAX) {
    servo_angle1 += servo_1.data;
    pwm.setPWM(0, 0, servo_angle1);
  }

  fake_servo_angle1 = servo_angle1;
}

void servoCb2(const std_msgs::Int16& servo_2){
  fake_servo_angle2 += servo_2.data;
  
  if (SERVOMIN < fake_servo_angle2 && fake_servo_angle2 < SERVOMAX) {
    servo_angle2 += servo_2.data;
    pwm.setPWM(1, 0, servo_angle2);
  }

  fake_servo_angle2 = servo_angle2;
}




/* Subscribes to servo topic */
ros::Subscriber<std_msgs::Int16> servo_1("servo_1", &servoCb1);
ros::Subscriber<std_msgs::Int16> servo_2("servo_2", &servoCb2);


void setup() {
  Serial.begin(57600);  // CANNOT use Serial.println when using rosserial

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  int i = 0;
  for (i = 0; i < 16; i++) {
    pwm.setPWM(0, 0, SERVOMIN);
    delay(200);
  }
  
  nh.initNode();
  nh.subscribe(servo_1);
  nh.subscribe(servo_2);
  
  delay(10);
}


void loop() {
  
  nh.spinOnce();
  delay(10);
  
}
