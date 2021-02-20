// Team C Sensors and Motors Lab

// Header Inclusions
#include <Servo.h> // servomotor control
#include <PID_v1.h> // DC motor encoder control

////////////////////////
// Variable Declarations
////////////////////////

// State-Switching Button declarations
int Button0 = 3;  // Button 0 connected to digital pin 2
int state = 0;    // Initialize state as 0
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
int IRpin = A5;
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

void setup(){
    Serial.begin(9600);
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

}

void loop(){
    unsigned long StartTime = millis();
    if (state == 0){
        //Sensor Mode code
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
        //Serial.print(Input); Serial.print("\t"); Serial.print(Setpoint); Serial.print("\t"); Serial.println(Output);

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
        //Serial.print("\t"); Serial.println(desiredStep);
        if(desiredStep > currentStep) digitalWrite(dirPin,HIGH);
        else digitalWrite(dirPin,LOW);
        for(int x = 0; x < abs(desiredStep-currentStep); x++){
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(500);
            digitalWrite(stepPin, LOW);
            delayMicroseconds(500);
        }
        currentStep = desiredStep;
    }
    if(state == 1){
        //Serial.println(state);
    }
    unsigned long CurrentTime = millis();
    unsigned long ElapsedTime = CurrentTime - StartTime;
    Serial.println(state);
}

//Helper Functions

// Function to debounce switch and change state
void State_Change() {
  if ((millis() - B0Press) > debounceDelay) { // Check to see when button was last pressed
    state += 1;   // If Button 0 is pushed, increment the state
    if (state == 3){
      state = 0;    // If currently state 2, cycle back to state 0
      B0Press = millis(); // Grab the time at which the button was pressed
    }
    Serial.print("State:");
    Serial.println(state);
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
  //Serial.println(readingsUltra[1]);
  //Serial.println(indexUltra);
  sumUltra -= readingsUltra[indexUltra];  // remove the earliest reading
  readingsUltra[indexUltra] = reading;  // update the reading array
  sumUltra += readingsUltra[indexUltra];   // add new reading to the sum
  indexUltra = (indexUltra+1) % WINDOW_SIZE_Ultra;   // move the “index” pointer to the next
  return sumUltra/WINDOW_SIZE_Ultra;
}

int avgFilterFlex(int reading)
{
  //Serial.println(readingsFlex[1]);
  //Serial.println(indexFlex);
  sumFlex -= readingsFlex[indexFlex];  // remove the earliest reading
  readingsFlex[indexFlex] = reading;  // update the reading array
  sumFlex += readingsFlex[indexFlex];   // add new reading to the sum
  indexFlex = (indexFlex+1) % WINDOW_SIZE_Flex;   // move the “index” pointer to the next
  return sumFlex/WINDOW_SIZE_Flex;
}

int avgFilterPot(int reading)
{
  //Serial.println(readingsPot[1]);
  //Serial.println(indexPot);
  sumPot -= readingsPot[indexPot];  // remove the earliest reading
  readingsPot[indexPot] = reading;  // update the reading array
  sumPot += readingsPot[indexPot];   // add new reading to the sum
  indexPot = (indexPot+1) % WINDOW_SIZE_Pot;   // move the “index” pointer to the next
  return sumPot/WINDOW_SIZE_Pot;
}
