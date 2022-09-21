#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <Arduino.h>


ros::NodeHandle nh;

std_msgs::Float32MultiArray puressure_msg;
ros::Publisher pub_puressure("puressure",&puressure_msg);

const int puressure1 = 0;
const int puressure2 = 1;

void setup() {
  nh.initNode();
  nh.advertise(pub_puressure);
  puressure_msg.data = (float*)malloc(sizeof(float)*2);
  puressure_msg.data_length = 2;

}

void loop() {  
  puressure_msg.data[0] = analogRead(puressure1);
  puressure_msg.data[1] = analogRead(puressure2);
  pub_puressure.publish(&puressure_msg);
  nh.spinOnce();
  delay(10);
}
