// Team C Sensors and Motors Lab

// Header Inclusions
#include <Servo.h> // servomotor control
#include <PID_v1.h> // DC motor encoder control

////////////////////////
// Variable Declarations
////////////////////////

// State-Switching Button declarations
const int state_button_pin = 3;  // Button 0 connected to digital pin 2
int state = 0;    // Initialize state as 0
unsigned long state_button_press = -400;  // The last time button 0 was pressed
unsigned long debounce_delay = 500;    // The debounce time


// Sensor Declarations
// Potentiometer declarations
#define pot_window_size 10
const int pot_pin = A0;
int pot_readings[pot_window_size];
int pot_sum = 0;
int pot_index = 0;
// Flex Sensor declarations
#define flex_window_size 10
const int flex_pin = A1;
int flex_readings[flex_window_size];
int flex_sum = 0;
int flex_index = 0;
// IR Sensor declarations
#define ir_window_size 10
int ir_pin = A5;
int ir_readings[ir_window_size];
int ir_sum = 0;
int ir_index = 0;
// Ultrasonic Sensor declarations
int ultra_trigger_pin = 13;
int ultra_echo_pin = 12;
#define ultra_window_size 10
int ultra_readings[ultra_window_size];
int ultra_sum = 0;
int ultra_index = 0;

// Motor Declarations
// Servo Motor declarations
const int servo_pin = 8;
Servo servo;
// DC Motor Encoder declarations
#define enable_motor 4
#define dc_encoder_a_pin 5
#define dc_encoder_b_pin 6
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
    pinMode(state_button_pin, INPUT);    // Set Button 0 to an input
    attachInterrupt(digitalPinToInterrupt(state_button_pin), State_Change, RISING);  // Attach interrupt to Button 0
    // Ultrasonic Sensor attachments
    pinMode(ultra_trigger_pin, OUTPUT);
    pinMode(ultra_echo_pin, INPUT);
    // Servo Motor attachments
    pinMode(servo_pin,OUTPUT);
    servo.attach(servo_pin);
    // DC Motor attachments
    pinMode(encoder_a, INPUT);    // Set encoder_a to an input
    attachInterrupt(digitalPinToInterrupt(encoder_a), Increment, FALLING);  // Attach interrupt to encoder_a
    pinMode(dc_encoder_a_pin, OUTPUT);
    pinMode(dc_encoder_b_pin, OUTPUT);
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
        int analogValuePot = analogRead(pot_pin);
        int avgValuePot= avgFilterPot(analogValuePot);
        float anglePot = map(avgValuePot, 0, 1024, 0, 180.0);
        // Flex sensor  (resistance VS force)
        // https://www.tinkercad.com/things/g7jKPpMlgkY-powerful-blorr-gaaris/editel 
        // All the flex sensor + servo motor code is in the above simulation circuit
        int val = analogRead(flex_pin);  // 707-880 -> 3.45V-4.29V (0-1023 -> 0-5V) . R = 10k . Rx = 22k to 60k
        int avg = avgFilterFlex(val);
        float angleFlex = map(avg, 700, 860, 0, 180.0);
        //IR Sensor
        int reading = analogRead(ir_pin);
        int average = avgFilterIR(reading);
        float voltage = (average/1024.0)*5;
        float dist = 50.6/(voltage-0.173);   // cm
        // Ultrasonic Sensor
        digitalWrite(ultra_trigger_pin, LOW);  
        delayMicroseconds(2);
        digitalWrite(ultra_trigger_pin, HIGH);
        delayMicroseconds(10); // Hold trigger for 10 microseconds, which is signal for sensor to measure distance.
        digitalWrite(ultra_trigger_pin, LOW);
        unsigned long durationMicroSec = pulseIn(ultra_echo_pin, HIGH);
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
        Serial.print(Input); Serial.print("\t"); Serial.print(Setpoint); Serial.print("\t"); Serial.println(Output);

        if(Output > 0){//CounterClockwise
            digitalWrite(dc_encoder_a_pin,HIGH);
            digitalWrite(dc_encoder_b_pin,LOW);
            analogWrite(enable_motor,Output);
        }
        else if(Output < 0){//Clockwise
            digitalWrite(dc_encoder_a_pin,LOW);
            digitalWrite(dc_encoder_b_pin,HIGH);
            analogWrite(enable_motor,-Output);
        }

        // Update Stepper Motor
        desiredStep = round(map(distanceCm, 0, 100, 0, stepsPerRevolution));
        Serial.print("\t"); Serial.println(desiredStep);
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
        Serial.println(state);
    }
    unsigned long CurrentTime = millis();
    unsigned long ElapsedTime = CurrentTime - StartTime;
}

//Helper Functions

// Function to debounce switch and change state
void State_Change() {
  if ((millis() - state_button_press) > debounce_delay) { // Check to see when button was last pressed
    state += 1;   // If Button 0 is pushed, increment the state
    if (state == 3){
      state = 0;    // If currently state 2, cycle back to state 0
      state_button_press = millis(); // Grab the time at which the button was pressed
    }
    Serial.print("State:");
    Serial.println(state);
  }
}


void Increment() {
  if(digitalRead(dc_encoder_a_pin)==HIGH){ //CounterClockwise
    count++;
  }
  else if(digitalRead(dc_encoder_b_pin)==HIGH){ //Clockwise
    count--;
  }
}


int avgFilterIR(int reading)
{
  ir_sum -= ir_readings[ir_index];  // remove the earliest reading
  ir_readings[ir_index] = reading;  // update the reading array
  ir_sum += ir_readings[ir_index];   // add new reading to the sum
  ir_index = (ir_index+1) % ir_window_size;   // move the “index” pointer to the next
  return ir_sum/ir_window_size;
}

int avgFilterUltra(int reading)
{
  //Serial.println(ultra_readings[1]);
  //Serial.println(ultra_index);
  ultra_sum -= ultra_readings[ultra_index];  // remove the earliest reading
  ultra_readings[ultra_index] = reading;  // update the reading array
  ultra_sum += ultra_readings[ultra_index];   // add new reading to the sum
  ultra_index = (ultra_index+1) % ultra_window_size;   // move the “index” pointer to the next
  return ultra_sum/ultra_window_size;
}

int avgFilterFlex(int reading)
{
  //Serial.println(flex_readings[1]);
  //Serial.println(flex_index);
  flex_sum -= flex_readings[flex_index];  // remove the earliest reading
  flex_readings[flex_index] = reading;  // update the reading array
  flex_sum += flex_readings[flex_index];   // add new reading to the sum
  flex_index = (flex_index+1) % flex_window_size;   // move the “index” pointer to the next
  return flex_sum/flex_window_size;
}

int avgFilterPot(int reading)
{
  //Serial.println(pot_readings[1]);
  //Serial.println(pot_index);
  pot_sum -= pot_readings[pot_index];  // remove the earliest reading
  pot_readings[pot_index] = reading;  // update the reading array
  pot_sum += pot_readings[pot_index];   // add new reading to the sum
  pot_index = (pot_index+1) % pot_window_size;   // move the “index” pointer to the next
  return pot_sum/pot_window_size;
}
