#include <ros.h>
#include <std_msgs/String.h>
#include <Wire.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

void setup(){

  nh.initNode();
  nh.advertise(chatter);
  
  Wire.begin();
  Serial.begin(57600);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x0);
  Wire.endTransmission();
}

void loop(){

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();

  int ax = 0;
  int ay = 0;
  int az = 0;
  int tmp = 0;
  int gx = 0;
  int gy = 0;
  int gz = 0;

  Wire.requestFrom(0x68, 14);

  gx = Wire.read() << 8;
  gx |= Wire.read();
  gy = Wire.read() << 8;
  gy |= Wire.read();
  gz = Wire.read() << 8;
  gz |= Wire.read();
  tmp = Wire.read() << 8;
  tmp |= Wire.read();
  ax = Wire.read() << 8;
  ax |= Wire.read();
  ay = Wire.read() << 8;
  ay |= Wire.read();
  az = Wire.read() << 8;
  az |= Wire.read();

  Serial.print("AX : ");
  Serial.print(ax);
  Serial.print("  AY : ");
  Serial.print(ay);
  Serial.print("  AZ : ");
  Serial.print(az);
  Serial.print("  GX : ");
  Serial.print(gx);
  Serial.print("  GY : ");
  Serial.print(gy);
  Serial.print("  GZ : ");
  Serial.println(gz);

  String AX = String(ax);
  String AY = String(ay);
  String AZ = String(az);
  String GX = String(gx);
  String GY = String(gy);
  String GZ = String(gz);

  String data = " A " + AX + " B " + AY + " C " + AZ + " D " + GX + " E " + GY + " F " + GZ + " G ";
  int length = data.indexOf( " G " ) + 2;
  char data_final[length+1];
  data.toCharArray(data_final, length+1);
  Serial.print(data_final);
  str_msg.data = data_final;
  chatter.publish( &str_msg );
  nh.spinOnce();
  
  delay(500); 

  
}
