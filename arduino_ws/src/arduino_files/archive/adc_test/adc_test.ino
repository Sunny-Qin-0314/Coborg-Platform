/* 
 * rosserial ADC Example
 * 
 * This is a poor man's Oscilloscope.  It does not have the sampling 
 * rate or accuracy of a commerical scope, but it is great to get
 * an analog value into ROS in a pinch.
 */


#include <Arduino.h>
#include <ros.h>
#include <rosserial_arduino/CMU.h>

ros::NodeHandle nh;

rosserial_arduino::CMU cmu_msg;
ros::Publisher p("CMU", &cmu_msg);

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();

  nh.advertise(p);
}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

long adc_timer;
int a = 0;

void loop()
{
  cmu_msg.Potentiometer = averageAnalog(0);
  cmu_msg.Flex_Sensor = averageAnalog(1);
  cmu_msg.IR_Sensor = averageAnalog(2);
  cmu_msg.Ultrasonic_Sensor = averageAnalog(3);
  cmu_msg.Button_State = averageAnalog(4);
    
  p.publish(&cmu_msg);

  nh.spinOnce();
}
