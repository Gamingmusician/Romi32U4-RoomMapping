#include <Romi32U4.h>
#include <Arduino.h>
#include <math.h>
#include <float.h>

Romi32U4Encoders encoders;
Romi32U4Motors motors;
Romi32U4ButtonA buttonA;

// ===== Constants =====
// --- Robot ---
#define WHEEL_D             7
#define ROBOT_D             15
#define ROBOT_R             8
#define COUNT_PER_REV       1437

#define PIN_TRIG            11
#define PIN_ECHO            2

// --- Times ---
#define TIME_DELAY          2000
#define TIME_PAUSE          250

// --- Speeds ---
#define SPEED_STRAIGHT_L    79
#define SPEED_STRAIGHT_R    80
#define SPEED_TURN_L        38
#define SPEED_TURN_R        40
#define SPEED_STOP          0

// --- Measuring ---
#define CM                  58.0
#define NUM_READINGS        50
#define SENSOR_MIN          0
#define SENSOR_MAX          150
#define ANGLE_COR           0.93

#define MAX_EDGE_LEN        10.0//cm

// --- Logic ---
#define ON                  1
#define OFF                 0

// ===== Global Variables =====
bool mapping = false;
float positionX = 0;
float positionY = 0;

// ===== Lined-List Node =====
// Each measurement becomes a Node in a circular list
struct Node {
  int x, y;     // Coordinates of the point
  Node* cw;     // Next point clockwise
  Node* ccw;    // Next point counter-clockwise
};
Node* head = nullptr;   // Start of circular linked list


// ===== Function Declarations =====
uint16_t countForDistance(float distance);
uint16_t countForAngle(float angle);
void rotate(int ang, int speedL, int speedR);
float ultrasonic();
float degToRad(float degrees);
float radToDeg(float radians);
void insert(int x, int y);
void findGaps();
void printOutline();
void clearList();


void setup(){
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
}

void loop(){
    // Wait for Button A
    ledRed(ON);
    buttonA.waitForButton();
    ledRed(OFF);
    delay(TIME_DELAY);
    
    // Clear previous data
    clearList();

    // ===== Scan Area =====
    const float step = 360.0f / NUM_READINGS;
    float distances[NUM_READINGS];
    for (int i = 0; i < NUM_READINGS; i++) {
      // Measure distance
      distances[i] = ultrasonic();
      // Rotate by step
      rotate(step, SPEED_TURN_L, SPEED_TURN_R);
    }
    // return to start
    rotate(-360.0f, SPEED_TURN_L, SPEED_TURN_R);

    // ===== Insert Points =====
    for (int i=0; i<NUM_READINGS; i++) {
      float thetaRad = degToRad(i * step);
      float r = distances[i] + ROBOT_R;
      int xi = positionX + int(r * cosf(thetaRad));
      int yi = positionY + int(r * sinf(thetaRad));
      insert(xi, yi);
    }

    // ===== Output Outline and Gaps =====
    printOutline();
    findGaps();
}


// ===== Linked-List Functions =====
// Euclidean distance between node and (x,y)
static float distPt(const Node* a, int x, int y) {
  float dx = a->x - x;
  float dy = a->y - y;
  return sqrtf(dx*dx + dy*dy);
}

void insert(int x, int y) {
  // New node
  Node* P = (Node*)malloc(sizeof(Node));
  P->x = x;
  P->y = y;

  // Empty list?
  if(!head) {
    head = P;
    P->cw = P;
    P->ccw = P;
    return;
  }

  // Find best insertion edge
  Node* bestA = nullptr;
  Node* bestB = nullptr;
  float bestCost = FLT_MAX;

  Node* cur = head;
  do {
    Node* nxt = cur->cw;
    float oldLen = distPt(cur, nxt->x, nxt->y);
    float cost = distPt(cur, x, y) + distPt(nxt, x, y) - oldLen;
    if (cost < bestCost) {
      bestCost = cost;
      bestA = cur;
      bestB = nxt;
    }
    cur = nxt;
  } while (cur != head);

  // Insert P between bestA and bestB
  bestA->cw = P;
  P->ccw = bestA;
  P->cw = bestB;
  bestB->ccw = P;
}

void findGaps() {
  if (!head) return;
  Node* cur = head;
  Serial.println("Gaps at midpoints: ");
  do{
    Node* nxt = cur->cw;
    float edgeLen = distPt(cur, nxt->x, nxt->y);
    if (edgeLen > MAX_EDGE_LEN) {
      float midX = (cur->x + nxt->x) * 0.5f;
      float midY = (cur->y + nxt->y) * 0.5f;
      Serial.print(midX);
      Serial.print(',');
      Serial.print(midY);
    }
    cur = nxt;
  } while (cur != head);
}

void printOutline() {
  if (!head) return;
  Serial.println("Outline:");
  Node* cur = head;
  do {
    Serial.print(cur->x);
    Serial.print(',');
    Serial.println(cur->y);
    cur = cur->cw;
  } while (cur != head);
}

void clearList() {
  if (!head) return;
  Node* cur = head->cw;
  while (cur != head) {
    Node* nxt = cur->cw;
    free(cur);
    cur = nxt;
  }
  free(head);
  head = nullptr;
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
    speedL = speedLC + (countR*dir - countL*dir);
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
    for(int j=0; j<arraySize; j++){
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
  /*float pulses = (180 - angle) / 360;
  pulses = pulses * (ROBOT_D * PI);
  pulses = pulses * COUNT_PER_REV / (WHEEL_D * PI);
  return int(pulses * ANGLE_COR);*/

  float fraction   = angle / 360.0f;
  float turnDist   = fraction * (ROBOT_D * PI);  
  float revolutions= turnDist / (WHEEL_D * PI);
  float counts     = revolutions * COUNT_PER_REV;
  return uint16_t(counts * ANGLE_COR);
}


float degToRad(float degrees){
  float temp = M_PI / 180;
  return temp * degrees;
}

float radToDeg(float radians){
  float temp = 180 / M_PI;
  return temp * radians;
}