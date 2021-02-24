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

// State-Switching Button declarations
int Button0 = 3;  // Button 0 connected to digital pin 3
int state = 1;    // Initialize state as 0
unsigned long B0Press = -400;  // The last time button 0 was pressed
unsigned long debounceDelay = 500;    // The debounce time

// Sensor Declarations
// Potentiometer declarations
#define WINDOW_SIZE_Pot 10
int POTENTIOMETER_PIN = A0;
int readingsPot[WINDOW_SIZE_Pot];
int sumPot = 0;
int indexPot = 0;

// Flex Sensor declarations
#define WINDOW_SIZE_Flex 10
int flexPin = A1;
int readingsFlex[WINDOW_SIZE_Flex];
int sumFlex = 0;
int indexFlex = 0;

// IR Sensor declarations
#define WINDOW_SIZE_IR 10
int IRpin = A2;
int readingsIR[WINDOW_SIZE_IR];
int sumIR = 0;
int indexIR = 0;

// Ultrasonic Sensor declarations
int triggerPin = 13;
int echoPin = 12;
#define WINDOW_SIZE_Ultra 10
int readingsUltra[WINDOW_SIZE_Ultra];
int sumUltra = 0;
int indexUltra = 0;

ros::NodeHandle nh;

rosserial_arduino::CMU cmu_msg;
ros::Publisher pub("CMU", &cmu_msg);

void setup()
{ 
 // State Button attachments
  pinMode(Button0, INPUT);    // Set Button 0 to an input
  attachInterrupt(digitalPinToInterrupt(Button0), State_Change, RISING);  // Attach interrupt to Button 0
  
  // Ultrasonic Sensor attachments
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  nh.initNode();
  nh.advertise(pub);
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
  //Sensor Mode code runs regardless of state
  //Potentiometer
  int analogValuePot = analogRead(POTENTIOMETER_PIN);
  int avgValuePot= avgFilterPot(analogValuePot);
  float anglePot = map(avgValuePot, 0, 1024, 0, 180.0);
  
  // Flex sensor  (resistance VS force)
  // https://www.tinkercad.com/things/g7jKPpMlgkY-powerful-blorr-gaaris/editel 
  // All the flex sensor + servo motor code is in the above simulation circuit
  int val = analogRead(flexPin);  // 707-880 -> 3.45V-4.29V (0-1023 -> 0-5V) . R = 10k . Rx = 22k to 60k
  int avg = avgFilterFlex(val);
  float angleFlex = map(avg, 700, 860, 0, 180.0);
  
  //IR Sensor
  int reading = analogRead(IRpin);
  int average = avgFilterIR(reading);
  float voltage = (average/1024.0)*5;
  float dist = 50.6/(voltage-0.173);   // cm
  
  // Ultrasonic Sensor
  digitalWrite(triggerPin, LOW);  
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10); // Hold trigger for 10 microseconds, which is signal for sensor to measure distance.
  digitalWrite(triggerPin, LOW);
  unsigned long durationMicroSec = pulseIn(echoPin, HIGH);
  int avgDurationMicroSec = avgFilterUltra((int) durationMicroSec);
  double distanceCm = avgDurationMicroSec / 2.0 * 0.0340;  // 340m/s  0.0340cm/microsec
  if(distanceCm > 100.0){
      distanceCm = 100.0;
  }

  //publish sensor data to GUI
  cmu_msg.Potentiometer = anglePot;
  cmu_msg.Flex_Sensor = angleFlex;
  cmu_msg.IR_Sensor = dist;
  cmu_msg.Ultrasonic_Sensor = distanceCm;
  cmu_msg.Button_State = state;
    
  pub.publish(&cmu_msg);

  nh.spinOnce();
}


//Helper Functions
// Function to debounce switch and change state
void State_Change() {
  if ((millis() - B0Press) > debounceDelay) { // Check to see when button was last pressed
    state += 1;   // If Button 0 is pushed, increment the state
    if (state == 2){
      state = 0;    // If currently state 1, cycle back to state 0
      B0Press = millis(); // Grab the time at which the button was pressed
    }

  }
}

//Average functions are seperate to prevent data collision between motors
int avgFilterIR(int reading)
{
  sumIR -= readingsIR[indexIR];  // remove the earliest reading
  readingsIR[indexIR] = reading;  // update the reading array
  sumIR += readingsIR[indexIR];   // add new reading to the sum
  indexIR = (indexIR+1) % WINDOW_SIZE_IR;   // move the “index” pointer to the next
  return sumIR/WINDOW_SIZE_IR;
}

int avgFilterUltra(int reading)
{
  sumUltra -= readingsUltra[indexUltra];  // remove the earliest reading
  readingsUltra[indexUltra] = reading;  // update the reading array
  sumUltra += readingsUltra[indexUltra];   // add new reading to the sum
  indexUltra = (indexUltra+1) % WINDOW_SIZE_Ultra;   // move the “index” pointer to the next
  return sumUltra/WINDOW_SIZE_Ultra;
}

int avgFilterFlex(int reading)
{
  sumFlex -= readingsFlex[indexFlex];  // remove the earliest reading
  readingsFlex[indexFlex] = reading;  // update the reading array
  sumFlex += readingsFlex[indexFlex];   // add new reading to the sum
  indexFlex = (indexFlex+1) % WINDOW_SIZE_Flex;   // move the “index” pointer to the next
  return sumFlex/WINDOW_SIZE_Flex;
}

int avgFilterPot(int reading)
{
  sumPot -= readingsPot[indexPot];  // remove the earliest reading
  readingsPot[indexPot] = reading;  // update the reading array
  sumPot += readingsPot[indexPot];   // add new reading to the sum
  indexPot = (indexPot+1) % WINDOW_SIZE_Pot;   // move the “index” pointer to the next
  return sumPot/WINDOW_SIZE_Pot;
}
