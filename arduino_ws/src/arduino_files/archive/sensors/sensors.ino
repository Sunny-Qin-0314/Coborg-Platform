//IR sensor, Ultrasonic (not use this anymore), flex sensor
#define WINDOW_SIZE_IR 10
#define WINDOW_SIZE_Flex 10
#define WINDOW_SIZE_Ultra 10
#define WINDOW_SIZE_Pot 10

#include <Servo.h>

int triggerPin = 13;  // for ultrasonic sensor
int echoPin = 12; // for ultrasonic sensor

int flexPin = A1; // flex sensor
int IRpin = A5; // IR sensor

int POTENTIOMETER_PIN = A0; // for potentiometer


const int SERVO_PIN = 8; // for servo motor
Servo servo;

// average filtering variables
int readingsIR[WINDOW_SIZE_IR];
int readingsFlex[WINDOW_SIZE_Flex];
int readingsUltra[WINDOW_SIZE_Ultra];
int readingsPot[WINDOW_SIZE_Pot];
int sumIR = 0;
int sumFlex = 0;
int sumUltra = 0;
int sumPot = 0;
int indexIR = 0;
int indexFlex = 0;
int indexUltra = 0;
int indexPot = 0;

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


void setup(void) {
  Serial.begin(9600);
  pinMode(SERVO_PIN,OUTPUT);
  servo.attach(SERVO_PIN);
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //for (int i=0; i< WINDOW_SIZE_IR; i++){
  //  readingsIR[i] = 0;
  //}

}

void loop(void) {
  unsigned long StartTime = millis();
  // IR sensor
  int reading = analogRead(IRpin);
  //int average = avgFilterIR(reading);
  int average = reading;
  //Serial.println(average);
  float voltage = (average/1024.0)*5;
  float dist = 50.6/(voltage-0.173);   // cm
  //Serial.println(dist);
  
  
  // Ultrasonic sensor 
  digitalWrite(triggerPin, LOW);  
  delayMicroseconds(2);
  // Hold trigger for 10 microseconds, which is signal for sensor to measure distance.
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  unsigned long durationMicroSec = pulseIn(echoPin, HIGH);
  int avgDurationMicroSec = avgFilterUltra((int) durationMicroSec);
  double distanceCm = avgDurationMicroSec / 2.0 * 0.0340;  // 340m/s  0.0340cm/microsec

  //Potentiometer
  int analogValuePot = analogRead(POTENTIOMETER_PIN);
  int avgValuePot= avgFilterPot(analogValuePot);
  float anglePot = map(avgValuePot, 0, 1024, 0, 180.0);
  //servo.write(avgValuePot/5.71);
  
  
  // Flex sensor  (resistance VS force)
  // https://www.tinkercad.com/things/g7jKPpMlgkY-powerful-blorr-gaaris/editel 
  // All the flex sensor + servo motor code is in the above simulation circuit
    
  int val = analogRead(flexPin);  // 707-880 -> 3.45V-4.29V (0-1023 -> 0-5V) . R = 10k . Rx = 22k to 60k
  int avg = avgFilterFlex(val);
  float angleFlex = map(avg, 700, 860, 0, 180.0);

  //threshold between flex sensor and potentiometer
  //to control the servomotor
  if (angleFlex < 15.0){
    servo.write(anglePot);
  }else{
    servo.write(angleFlex);
  }
  
  unsigned long CurrentTime = millis();

  unsigned long ElapsedTime = CurrentTime - StartTime;

  //Serial.println(ElapsedTime);
  Serial.println(angleFlex);

    
  
}
