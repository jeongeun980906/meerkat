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
   
  int a = cmd_msg.data;
  boolean reverse;

  if( 0 < a < 90){
    reverse = false;
  }
  else if( 90 < a < 180){
    reverse = true;
    a=180-a;
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
  //pinMode(7, OUTPUT);  //angle
  
  pinMode(10, OUTPUT);  //in1, front
  pinMode(9, OUTPUT);   //in2, rear

  pinMode(13, OUTPUT);  //LED

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

void setMotor(int a, boolean reverse){
  digitalWrite(in1, ! reverse); // 0~89 입력시 정주행
  digitalWrite(in2, reverse);   // 91~179 입력시 역주행
  servo1.write(a); //set servo angle, should be from 0-180  
  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  

}
