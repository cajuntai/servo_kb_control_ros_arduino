/*VS Code Arduino CLI: To switch which.ino file to verify/upload: Ctrl + Shift + P --> Select sketch*/
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
int16_t servo_angle3 = SERVOMIN;
int16_t fake_servo_angle3 = SERVOMIN;
int16_t servo_angle4 = SERVOMIN;
int16_t fake_servo_angle4 = SERVOMIN;


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

void servoCb3(const std_msgs::Int16& servo_3){
 fake_servo_angle3 += servo_3.data;
 
 if (SERVOMIN < fake_servo_angle3 && fake_servo_angle3 < SERVOMAX) {
   servo_angle3 += servo_3.data;
   pwm.setPWM(2, 0, servo_angle3);
 }

 fake_servo_angle3 = servo_angle3;
}

void servoCb4(const std_msgs::Int16& servo_4){
 fake_servo_angle4 += servo_4.data;
 
 if (SERVOMIN < fake_servo_angle4 && fake_servo_angle4 < SERVOMAX) {
   servo_angle4 += servo_4.data;
   pwm.setPWM(3, 0, servo_angle4);
 }

 fake_servo_angle4 = servo_angle4;
}


/* Subscribes to servo topic */
ros::Subscriber<std_msgs::Int16> servo_1("diff_angle/servo_1", &servoCb1);
ros::Subscriber<std_msgs::Int16> servo_2("diff_angle/servo_2", &servoCb2);
ros::Subscriber<std_msgs::Int16> servo_3("diff_angle/servo_3", &servoCb3);
ros::Subscriber<std_msgs::Int16> servo_4("diff_angle/servo_4", &servoCb4);


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
 nh.subscribe(servo_3);
 nh.subscribe(servo_4);
 
 delay(10);
}


void loop() {
 
 nh.spinOnce();
 delay(10);
 
}
