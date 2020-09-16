#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;

Servo servo1;
Servo servo2;

void servovel_cb( const std_msgs::Int16& cmd_msg){
   
  servo1.write(cmd_msg.data); 
  }
  
  setMotor(a, reverse);
  }

void servoang_cb( const std_msgs::Int16& cmd2_msg) {

  servo2.write(cmd2_msg.data); //50~80~110
}

ros::Subscriber<std_msgs::Int16> subvel("servovel", servovel_cb);
ros::Subscriber<std_msgs::Int16> subang("servoang", servoang_cb);

int enable = 11;
int in1 = 10;
int in2 = 9;

void setup(){

  nh.initNode();
  nh.subscribe(subvel);
  nh.subscribe(subang);
  
  servo1.attach(11); //attach it to pin 11 = enable pin
  servo2.attach(7);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
