#include <Romi32U4.h>
Romi32U4Encoders encoders;
Romi32U4Motors motors;
Romi32U4ButtonA buttonA;

// ===== Constants =====
// --- Robot ---
#define WHEEL_D           7
#define ROBOT_D           15
#define ROBOT_R           8
#define COUNT_PER_REV     1437
#define PIN_TRIG          11
#define PIN_ECHO          2
// --- Times ---
#define TIME_DELAY        2000
#define TIME_PAUSE        250
// --- Speeds ---
#define SPEED_STRAIGHT_L  79
#define SPEED_STRAIGHT_R  80
#define SPEED_TURN_L      38
#define SPEED_TURN_R      40
#define SPEED_STOP        0
// --- Measuring ---
#define CM                58.0
#define NUM_READINGS      10
#define SENSOR_MIN        0
#define SENSOR_MAX        150
#define ANGLE_COR         0.93
// --- Logic ---
#define ON                1
#define OFF               0

// ===== Global Variables =====


void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  ledRed(ON);
}

void loop() {
  // Wait for Button A
  buttonA.waitForButton();
  ledRed(OFF);
  delay(TIME_DELAY);

  // =-=-=-=-= Functions =-=-=-=-=
  //drive(dist, speedL, speedR)   // Drives straight
  //rotate(ang, speedL, speedR)   // Rotates in place
  //ultrasonic                    // Returns median measurement
}

// DRIVE STRAIGHT FUNCTION
void drive(int dist, int speedL, int speedR) {
  // ===== Pre Loop =====
  int countL = encoders.getCountsAndResetLeft();
  int countR = encoders.getCountsAndResetRight();
  int speedLC = speedL;
  countL = encoders.getCountsAndResetLeft();
  countR = encoders.getCountsAndResetRight();
  int dir = 1;

  // ===== Calculate Distance =====
  if(dist < 0){ dir = 1; dist = -dist; }
  float pulseCount = countForDistance(dist);

  // ===== Drive Loop =====
  while(countL*dir < pulseCount  ||  countR*dir < pulseCount){
    // --- Read encoders ---
    countL = encoders.getCountsLeft();
    countR = encoders.getCountsRight();

    // --- Calculate speeds ---
    speedL = speedLC + (countR*dir - countL*dir)l;
    if(countL*dir >= pulseCount){ speedL = SPEED_STOP; }
    if(countR*dir >= pulseCount){ speedR = SPEED_STOP; }

    // --- Set speeds ---
    motors.setSpeeds(speedL*dir, speedR*dir);
  }

  // ===== Conclude Drive =====
  motors.setSpeeds(SPEED_STOP, SPEED_STOP);
  countL = encoders.getCountsAndResetLeft();
  countR = encoders.getCountsAndResetRight();
  delay(TIME_PAUSE);
}

// ROTATE IN-PLACE FUNCTION
void rotate(int ang, int speedL, int speedR){
  // ===== Pre Loop =====
  int countL = encoders.getCountsAndResetLeft();
  int countR = encoders.getCountsAndResetRight();
  int speedLC = speedL;
  countL = encoders.getCountsAndResetLeft();
  countR = encoders.getCountsAndResetRight();
  int dir = 1;

  // ===== Calculate Distance =====
  if(ang < 0){ dir = -1; ang = -ang; }
  float pulseCount = countForAngle(ang);

  // ===== Drive Loop =====
  while(countL*dir < pulseCount  ||  -countR*dir < pulseCount){
    // --- Read Encoders ---
    countL = encoders.getCountsLeft();
    countR = encoders.getCountsRight();

    // --- Calculate Speeds ---
    speedL = speedLC + (-countR*dir - countL*dir);
    if(countL*dir >= pulseCount){ speedL = SPEED_STOP; }
    if(-countR*dir >= pulseCount){ speedR = SPEED_STOP; }

    // --- Set Speeds ---
    motors.setSpeeds(speedL*dir, -speedR*dir);
  }

  // ===== Conclude Drive =====
  motors.setSpeeds(SPEED_STOP, SPEED_STOP);
  countL = encoders.getCountsAndResetLeft();
  countR = encoders.getCountsAndResetRight();
  delay(TIME_PAUSE);
}

// ULTRASOUND FUNCTION
float ultrasonic(){
  //Serial.println(" ===== Start 'ultrasonic' Function =====");
  // ===== Declare =====
  int arraySize = NUM_READINGS;
  float measurements[arraySize];
  float distTemp; // Temporary distance value

  // ===== Measure/Filter =====
  for(int i=0; i<arraySize; i++){
    // --- Measure Distance ---
    digitalWrite(PIN_TRIG, LOW);  // Ensures a clean pulse beforehand
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH); // Sends a pulse
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);  // Disables pulse
    distTemp = ( pulseIn(PIN_ECHO, HIGH, 25000) ) / CM; // Calculates distance based on pulse time

    // --- Filter/Store ---
    if(distTemp >= SENSOR_MAX){ i--; } // Invalid, discard and retry
    else{ measurements[i] = distTemp; }
  }

  // ===== Sort =====
  for(int i=0; i<arraySize; i++){
    for(int j=0; j<arraySize; j++;){
      if(measurements[j] > measurements[i]){
        float temp = measurements[i];
        measurements[i] = measurements[j];
        measurements[j] = temp;
      }
    }
  }

  // ===== Return Median =====
  int medPos = NUM_READINGS / 2;  // Calculates median position
  return measurements[medPos];    // Returns median
}

uint16_t countForDistance(float distance){
  float temp = (WHEEL_D * PI) / COUNT_PER_REV;
  temp = distance / temp;
  return int(temp);
}

uint16_t countForAngle(float angle){
  float pulses = (180 - angle) / 360;
  pulses = pulses * (ROBOT_D * PI);
  pulses = pulses * COUNT_PER_REV / (WHEEL_D * PI);
  return int(pulses * ANGLE_COR);
}