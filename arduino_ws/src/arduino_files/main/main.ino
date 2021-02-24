// Team C Sensors and Motors Lab
// Header Inclusions
#include <Servo.h> // servomotor control
#include <PID_v1.h> // DC motor encoder control

// ROS Headers
#include <Arduino.h>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <rosserial_arduino/CMU.h>

////////////////////////
// Variable Declarations
////////////////////////

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

// Motor Declarations
// Servo Motor declarations
const int SERVO_PIN = 8;
Servo servo;

// DC Motor Encoder declarations
#define enable_motor 4
#define motorpin1 5
#define motorpin2 6
int encoder_a_value = 0; // Initialize digital record of encoder_a
int count = 0;
float angle = 0;
#define encoder_a 20 // Put encoder output A pin here
#define encoder_b 21
float countpercycle = 172;

// Stepper Motor declarations
#define dirPin 10
#define stepPin 11
const int stepsPerRevolution = 400;
int currentStep = 0;
int desiredStep = 0;
double Kp = 5;
double Kd = 10;
double Ki = 0.00001;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp, Kd, Ki, DIRECT);

// ROS declarations
// Message callback declaration
void messageCb(const sensor_msgs::JointState& msg) { //don't move this function. doesn't work if moved under void loop().
  if (state == 1){ //only run this code in GUI state (1)
    //servo msg
    int set_angle = msg.position[0]*(180/3.14); //servo motor convert rad2deg (pi -> 180 deg rotation)
    servo.write(set_angle); // valid inputs: 0-180
  
    //dc motor msg
    int set_pwm = msg.position[1]*(255/1.57); //dc motor range is from pi/2 to -pi/2. each side denotes max PWM speed (255) in that direction                                       
    if(set_pwm > 0){//CounterClockwise
        digitalWrite(motorpin1,HIGH);
        digitalWrite(motorpin2,LOW);
        analogWrite(enable_motor, set_pwm);
    }
    else if(set_pwm < 0){//Clockwise
        digitalWrite(motorpin1,LOW);
        digitalWrite(motorpin2,HIGH);
        analogWrite(enable_motor, -set_pwm); //only positive PWM values. --set_pwm = +set_pwm
    }
  
    //stepper message
    int set_step = msg.position[2]*(stepsPerRevolution/6.28); //convert rad2steps (2 pi = 360 deg rotation) 
    if(set_step > currentStep) digitalWrite(dirPin,HIGH);
    else digitalWrite(dirPin,LOW);
    for(int x = 0; x < abs(set_step-currentStep); x++){
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(1000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(1000);
    }
    currentStep = set_step; //ensure no discrepancy
  }
}

// ROS node type(s) declaration
ros::NodeHandle nh; //create a NodeHandle called nh
ros::Subscriber<sensor_msgs::JointState> sub("joint_states", messageCb);  //subscribe to joint_states
rosserial_arduino::CMU cmu_msg; //define CMU message type
ros::Publisher pub("CMU", &cmu_msg); //create a publisher to cmu_message of type CMU

void setup(){
    // State Button attachments
    pinMode(Button0, INPUT);    // Set Button 0 to an input
    attachInterrupt(digitalPinToInterrupt(Button0), State_Change, RISING);  // Attach interrupt to Button 0
    
    // Ultrasonic Sensor attachments
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    
    // Servo Motor attachments
    pinMode(SERVO_PIN,OUTPUT);
    servo.attach(SERVO_PIN);
    
    // DC Motor attachments
    pinMode(encoder_a, INPUT);    // Set encoder_a to an input
    attachInterrupt(digitalPinToInterrupt(encoder_a), Increment, FALLING);  // Attach interrupt to encoder_a
    pinMode(motorpin1, OUTPUT);
    pinMode(motorpin2, OUTPUT);
    pinMode(enable_motor, OUTPUT);
    myPID.SetOutputLimits(-255,255);
    myPID.SetMode(AUTOMATIC);
    
    // Stepper Motor Attachments
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    
    //ROS Setup
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub);
}

void loop(){
    //Sensor Mode code runs regardless of state
    //Potentiometer
    int analogValuePot = analogRead(POTENTIOMETER_PIN);
    int avgValuePot= avgFilterPot(analogValuePot);
    float anglePot = map(avgValuePot, 0, 1024, 0, 180.0);
    if(anglePot > 180.0){//bounds check
        anglePot = 180.0;
    }
    if(anglePot < 0){
        anglePot = 0;
    }
        
    // Flex sensor  (resistance VS force)
    // https://www.tinkercad.com/things/g7jKPpMlgkY-powerful-blorr-gaaris/editel 
    // All the flex sensor + servo motor code is in the above simulation circuit
    int val = analogRead(flexPin);  // 707-880 -> 3.45V-4.29V (0-1023 -> 0-5V) . R = 10k . Rx = 22k to 60k
    int avg = avgFilterFlex(val);
    float angleFlex = map(avg, 700, 860, 0, 180.0);
    if(angleFlex > 180.0){//bounds check
        angleFlex = 180.0;
    }
    if(angleFlex < 0){
        angleFlex = 0;
    }
    
    //IR Sensor
    int reading = analogRead(IRpin);
    int average = avgFilterIR(reading);
    float voltage = (average/1024.0)*5;
    float dist = 50.6/(voltage-0.173);   // cm
    if(dist > 150.0){//bounds check
        dist = 150.0;
    }
    if(dist < 0){
        dist = 150;
    }
    
    // Ultrasonic Sensor
    digitalWrite(triggerPin, LOW);  
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10); // Hold trigger for 10 microseconds, which is signal for sensor to measure distance.
    digitalWrite(triggerPin, LOW);
    unsigned long durationMicroSec = pulseIn(echoPin, HIGH);
    int avgDurationMicroSec = avgFilterUltra((int) durationMicroSec);
    double distanceCm = avgDurationMicroSec / 2.0 * 0.0340;  // 340m/s  0.0340cm/microsec
    if(distanceCm > 100.0){//bounds check
        distanceCm = 100.0;
    }
    if(distanceCm < 0){
        distanceCm = 0;
    }
    
    if (state == 0){ // sensors updates motor values only in state 0
        //Update Servo Motor
        if (angleFlex < 15.0){ //threshold between flex sensor and potentiometer
        servo.write(anglePot);
        }else{
        servo.write(angleFlex);
        }

        // Update DC Motor
        Setpoint = map(dist, 20, 150, 0, 360);

        Input = (360/countpercycle)*float(count);
        myPID.Compute();

        if(Output > 0){//CounterClockwise
            digitalWrite(motorpin1,HIGH);
            digitalWrite(motorpin2,LOW);
            analogWrite(enable_motor,Output);
        }
        else if(Output < 0){//Clockwise
            digitalWrite(motorpin1,LOW);
            digitalWrite(motorpin2,HIGH);
            analogWrite(enable_motor,-Output);
        }

        // Update Stepper Motor
        desiredStep = round(map(distanceCm, 0, 100, 0, stepsPerRevolution));
        if(desiredStep > currentStep) digitalWrite(dirPin,HIGH);
        else digitalWrite(dirPin,LOW);
        for(int x = 0; x < abs(desiredStep-currentStep); x++){
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(1000);
            digitalWrite(stepPin, LOW);
            delayMicroseconds(1000);
        }
        currentStep = desiredStep;
    }

    //package sensor date to cmu_msg
    cmu_msg.Potentiometer = anglePot;
    cmu_msg.Flex_Sensor = angleFlex;
    cmu_msg.IR_Sensor = dist;
    cmu_msg.Ultrasonic_Sensor = distanceCm;
    cmu_msg.Button_State = state;
    
    pub.publish(&cmu_msg); //send it!
    nh.spinOnce(); //check for ROS callbacks for joint_states movement
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

void Increment() {
  if(digitalRead(motorpin1)==HIGH){ //CounterClockwise
    count++;
  }
  else if(digitalRead(motorpin2)==HIGH){ //Clockwise
    count--;
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
