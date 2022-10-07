#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <Arduino.h>


ros::NodeHandle nh;

std_msgs::Float32MultiArray puressure_msg;
std_msgs::Bool switch_msg;
ros::Publisher pub_puressure("puressure",&puressure_msg);
ros::Publisher pub_switch("switch",&switch_msg);

const int puressure1 = 0;
const int puressure2 = 1;
const int button_pin = 7;
const int EMA = 9;
const int EMB = 10;
int i = 0;
bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = true;

void setup() {
  nh.initNode();
  nh.advertise(pub_puressure);
  nh.advertise(pub_switch);
  
  puressure_msg.data = (float*)malloc(sizeof(float)*2);
  puressure_msg.data_length = 2;

  pinMode(button_pin, INPUT);
  last_reading =  digitalRead(button_pin);

  pub_switch.publish(digitalRead(button_pin));
  
}

void loop() {
  
  puressure_msg.data[0] = analogRead(puressure1);
  puressure_msg.data[1] = analogRead(puressure2);
  pub_puressure.publish(&puressure_msg);
  
  bool reading = ! digitalRead(button_pin);
  
  if (last_reading!= reading){
      last_debounce_time = millis();
      published = false;
  }
  if ( !published && (millis() - last_debounce_time)  > debounce_delay) {
    switch_msg.data = reading;
    pub_switch.publish(&switch_msg);
    published = true;
    for (i = 255; i <= 0; i = i - 10){
      analogWrite(EMA,i);
      analogWrite(EMB,i);
      delay(30);
    }
  }
  last_reading = reading;
  
  nh.spinOnce();
  delay(10);

}
