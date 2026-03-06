#include <Arduino.h>  // pinMode, digitalWrite, analogWrite, delay, millis, pulseIn, Serial
#include <Servo.h>    // Servo control for angle values
#include <Adafruit_NeoPixel.h> //Num, Pin, Color, begin, setPixelColor, show

#define POSFin 0b10001111 //adress of the bot to signal finish

// GRIPPER OPEN POSITION SHOULD BE 120 AND CLOSED MAXIMUM OF 60

bool raceStarted = false; // becomes true when flag up
bool checkFlagStart(); //checks flagstate to start
int flagBaselineCm = -1; // baseline distance for flag detection
bool finishSent = false; // ensure finish packet sent only once

enum LineState { LINE_NONE, LINE_LEFT, LINE_RIGHT, LINE_CENTER, LINE_T, LINE_END };
enum State { 
  WAIT_FLAG,
  START_FORWARD,
  START_GRAB,
  START_TURN_LEFT,
  START_FORWARD_AFTER_TURN,
  RUN_FOLLOW,
  AVOID_TURN_LEFT,
  AVOID_FORWARD_1,
  AVOID_CURVE_RIGHT,
  AVOID_FORWARD_2,
  AVOID_TURN_BACK,
  AVOID_SEEK_LINE,
  DROP_BACKUP_BEFORE_OPEN,
  DROP_OPEN_WHILE_BACKING,
  DROP_BACKUP_AFTER_OPEN,
  DONE
};

void sendFinishPacketOnce(){ //sends finish packet
  if(finishSent) return;
  finishSent = true;
  Serial.write((uint8_t)POSFin); //send it twice to ensure reception
  Serial.write((uint8_t)POSFin);
}

// NeoPixels
const int NEO_PIN = 4;
const int NEO_COUNT = 4;
Adafruit_NeoPixel pixels(NEO_COUNT, NEO_PIN, NEO_GRB + NEO_KHZ800); //Neo_KHZ800 standard communication speed

uint32_t COL_BLUE; //32-bit color values
uint32_t COL_GREEN;
uint32_t COL_YELLOW;
uint32_t COL_RED;
uint32_t COL_PURPLE;
uint32_t COL_WHITE;

// gripper
const int gripperPin = 12;
Servo gripper;

// motor pins
const int motorL_fwd = 6;
const int motorL_rev = 5;
const int motorR_fwd = 9;
const int motorR_rev = 10;

// flip direction if wiring is reversed
bool invertLeft  = false;
bool invertRight = false;

// sonar
const int trigger = 7;
const int echo    = 8;
const unsigned long SONAR_INTERVAL_MS = 300;
unsigned long lastSonarPingMs = 0;
int lastDistanceCm = 999;

// line sensors
const int sensorCount = 8;
const int sensorPins[sensorCount] = {A0, A1, A2, A3, A4, A5, A6, A7};
int sensorValues[sensorCount];
const int BLACK = 850; // decrease for more sensitivity


// timings (ms)
unsigned long startBlindMs        = 850;
unsigned long grabHoldMs          = 350;
unsigned long turnLeftMs          = 630;
unsigned long afterTurnForwardMs  = 600;

// drop timing
unsigned long backupMs                = 3000;  // total reverse time
unsigned long releaseAfterReverseMs   = 250;   // reverse first, then open
unsigned long dropHoldMs              = 450;   // keep open while still reversing

// speeds while blind start
float startL = 0.85;
float startR = 1.00;
float turnSpd = 0.70;
float afterTurnL = 0.70;
float afterTurnR = 0.65;
float backL = 0.80;
float backR = 0.85;

// line follow speeds
float followCenterL = 0.85;
float followCenterR = 1.00;
float followLeftTurnL  = 0.55;
float followLeftTurnR  = 0.85;
float followRightTurnL = 0.85;
float followRightTurnR = 0.50;

// gripper angles
int gripOpen  = 120;
int gripClose = 60;

// gripper keep-alive (20x/sec)
const unsigned long GRIPPER_HZ_MS = 80; 
bool gripperHoldOpen = false;            
unsigned long lastGripCmd = 0;            

// drop marker detect
int markerBlackCount = 7;   
int markerStableMs   = 200;  
unsigned long dropMinMs     = 2000; 
unsigned long nonBlackArmMs = 200; 

// flag detection
const int FLAG_MIN_CM = 5;
const int FLAG_MAX_CM = 20;

// sonar avoid
int obstacleCm = 20; 
unsigned long avoidTurnLeftMs    = 500; 
unsigned long avoidForward1Ms    = 650;
unsigned long avoidCurveRightMs  = 800; 
unsigned long avoidForward2Ms    = 200;
unsigned long avoidTurnBackMs    = 0;
unsigned long avoidSeekMs        = 10;

float avoidTurnSpd = 0.70;
float avoidFwdL    = 0.85;
float avoidFwdR    = 0.85;
float avoidTurnRightL = 0.50;   
float avoidTurnRightR = 0.50;
float avoidTurnLeftL = 0.00;   
float avoidTurnLeftR = 0.90;
float avoidTurnBackL = 0.60;   
float avoidTurnBackR = 0.00;  

State state = WAIT_FLAG;
unsigned long stateStart = 0;

// drop logic
bool dropArmed = false;
unsigned long runFollowStart = 0;
unsigned long nonBlackSince = 0;

// marker stable
bool markerStable = false;
unsigned long markerSince = 0;

// drop reverse timing
unsigned long dropReverseStart = 0;

// ==================== LED Functions ====================

void ledsAll(uint32_t c){ 
  for(int i=0; i<NEO_COUNT; i++){
    pixels.setPixelColor(i, c);
  }
  pixels.show();
}

void ledsLeftRight(uint32_t leftC, uint32_t rightC){
  pixels.setPixelColor(3, leftC);
  pixels.setPixelColor(0, leftC);
  pixels.setPixelColor(1, rightC);
  pixels.setPixelColor(2, rightC);
  pixels.show();
}

void setLedsForState(State s){
  if(s == START_FORWARD || s == START_GRAB || s == START_TURN_LEFT || s == START_FORWARD_AFTER_TURN){ 
    ledsAll(COL_BLUE);
  }
  else if(s == RUN_FOLLOW){
    ledsAll(COL_GREEN);
  }
  else if(s == AVOID_TURN_LEFT || s == AVOID_FORWARD_1 || s == AVOID_CURVE_RIGHT || s == AVOID_FORWARD_2 || s == AVOID_TURN_BACK || s == AVOID_SEEK_LINE){
    ledsAll(COL_RED);
  }
  else if(s == DROP_BACKUP_BEFORE_OPEN || s == DROP_OPEN_WHILE_BACKING || s == DROP_BACKUP_AFTER_OPEN || s == DONE){
    ledsAll(COL_PURPLE);
  }
  else{
    ledsAll(COL_GREEN);
  }
}

void setLedsForLine(LineState line){ 
  if(line == LINE_LEFT){
    ledsLeftRight(COL_YELLOW, COL_GREEN);
  }
  else if(line == LINE_RIGHT){
    ledsLeftRight(COL_GREEN, COL_YELLOW);
  }
  else{
    ledsAll(COL_GREEN);
  }
}

// ==================== Motor Functions ====================

void motorSpeedAdjuster(int pinFwd, int pinRev, float speed, boolean forward, boolean invertDir){
  int pwm = (int)(255.0f * speed); 
  pwm = constrain(pwm, 0, 255);
  bool dir = forward;
  if(invertDir) dir = !dir;
  if(dir){
    analogWrite(pinRev, 0);
    analogWrite(pinFwd, pwm);
  } else {
    analogWrite(pinFwd, 0);
    analogWrite(pinRev, pwm);
  }
}

void stopMotors(void){ 
  analogWrite(motorL_fwd, 0);
  analogWrite(motorL_rev, 0);
  analogWrite(motorR_fwd, 0);
  analogWrite(motorR_rev, 0);
}

void driveForward(float l, float r){ 
  motorSpeedAdjuster(motorL_fwd, motorL_rev, l, true,  invertLeft);
  motorSpeedAdjuster(motorR_fwd, motorR_rev, r, true,  invertRight);
}

void driveBackward(float l, float r){ 
  motorSpeedAdjuster(motorL_fwd, motorL_rev, l, false, invertLeft);
  motorSpeedAdjuster(motorR_fwd, motorR_rev, r, false, invertRight);
}

void turnLeftBlind(float spd){
  motorSpeedAdjuster(motorL_fwd, motorL_rev, spd, false, invertLeft);
  motorSpeedAdjuster(motorR_fwd, motorR_rev, spd, true,  invertRight);
}

void obstaclePivotRight(float leftSpd, float rightSpd){
  motorSpeedAdjuster(motorL_fwd, motorL_rev, leftSpd,  true,  invertLeft);
  motorSpeedAdjuster(motorR_fwd, motorR_rev, rightSpd, false, invertRight);
}

void obstaclePivotLeft(float leftSpd, float rightSpd){
  motorSpeedAdjuster(motorL_fwd, motorL_rev, leftSpd,  true, invertLeft);
  motorSpeedAdjuster(motorR_fwd, motorR_rev, rightSpd, true,  invertRight);
}

void curveRight(float leftSpd, float rightSpd){
  motorSpeedAdjuster(motorL_fwd, motorL_rev, leftSpd,  true, invertLeft);
  motorSpeedAdjuster(motorR_fwd, motorR_rev, rightSpd, true, invertRight);
}

// ==================== Sonar ====================

int readDistanceCm(void){
  if(state != RUN_FOLLOW && state != WAIT_FLAG){
    return 999;
  }
  unsigned long now = millis();
  if(now - lastSonarPingMs < SONAR_INTERVAL_MS){
    return lastDistanceCm;
  }
  lastSonarPingMs = now;
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  long us = pulseIn(echo, HIGH, 6000);
  if(us == 0){
    lastDistanceCm = 999;
  } else {
    lastDistanceCm = (int)((us * 0.034f) / 2.0f);
  }
  return lastDistanceCm;
}

bool checkFlagStart(){
  int d = readDistanceCm();
  if(d == 999) return false;
  return (d >= FLAG_MIN_CM && d <= FLAG_MAX_CM);
}

// ==================== Line ====================

LineState readLine(void){
  for(int i=0; i<sensorCount; i++){
    sensorValues[i] = analogRead(sensorPins[i]); 
  }
  bool right  = sensorValues[0] > BLACK || sensorValues[1] > BLACK || sensorValues[2] > BLACK;
  bool center = sensorValues[3] > BLACK || sensorValues[4] > BLACK;
  bool left   = sensorValues[5] > BLACK || sensorValues[6] > BLACK || sensorValues[7] > BLACK;
  if(left && center && right) return LINE_T;
  if(!left && !center && !right) return LINE_END;
  if(center) return LINE_CENTER;
  if(left) return LINE_LEFT;
  if(right) return LINE_RIGHT;
  return LINE_NONE; 
}

bool lineSeenNow(void){
  for(int i=0; i<sensorCount; i++){
    if(sensorValues[i] > BLACK) return true;
  }
  return false;
}

void handleLine(LineState line){
  if(line == LINE_CENTER){
    driveForward(followCenterL, followCenterR);
  }
  else if(line == LINE_LEFT){
    motorSpeedAdjuster(motorL_fwd, motorL_rev, followLeftTurnL, false, invertLeft);
    motorSpeedAdjuster(motorR_fwd, motorR_rev, followLeftTurnR, true,  invertRight);
  }
  else if(line == LINE_RIGHT){
    motorSpeedAdjuster(motorL_fwd, motorL_rev, followRightTurnL, true,  invertLeft);
    motorSpeedAdjuster(motorR_fwd, motorR_rev, followRightTurnR, false, invertRight);
  }
  else{
    driveForward(followCenterL, followCenterR);
  }
}

// ==================== Marker ====================

void updateMarkerStable(void){
  unsigned long now = millis();
  int blackCount = 0;
  for(int i=0; i<sensorCount; i++){
    if(sensorValues[i] > BLACK) blackCount++;
  }
  if(blackCount >= markerBlackCount){
    if(markerSince == 0) markerSince = now;
    if(now - markerSince >= (unsigned long)markerStableMs){
      markerStable = true;
    }
  } else {
    markerSince = 0;
    markerStable = false;
  }
}

// ==================== Gripper ====================

void gripperHoldUpdate(void){
  unsigned long now = millis();
  if(now - lastGripCmd < GRIPPER_HZ_MS) return;
  lastGripCmd = now;
  if(gripperHoldOpen){
    gripper.write(gripOpen);  
  } else {
    gripper.write(gripClose); 
  }
}

// ==================== State ====================

void enterState(State s){
  state = s;
  stateStart = millis();
  if(s == DONE && !finishSent){
    finishSent = true;
    Serial.write((uint8_t)POSFin); 
    Serial.write((uint8_t)POSFin);
  }
  if(s == START_GRAB){ 
    gripperHoldOpen = false;
    gripper.write(gripClose);
  }
  if(s == RUN_FOLLOW){
    dropArmed = false;
    runFollowStart = millis();
    nonBlackSince = 0;
    gripperHoldOpen = false;
  }
  if(s == DROP_BACKUP_BEFORE_OPEN){
    dropReverseStart = millis();
    gripperHoldOpen = false;
  }
  if(s == DROP_OPEN_WHILE_BACKING){
    gripperHoldOpen = true;
    gripper.write(gripOpen);
  }
  setLedsForState(s);
}

// ==================== Setup ====================

void setup(){
  Serial.begin(9600);
  pixels.begin();
  pixels.setBrightness(40);
  COL_BLUE   = pixels.Color(0, 0, 240);
  COL_GREEN  = pixels.Color(0, 240, 0);
  COL_YELLOW = pixels.Color(245, 170, 0);
  COL_RED    = pixels.Color(245, 0, 0);
  COL_PURPLE = pixels.Color(160, 0, 245);
  COL_WHITE  = pixels.Color(245, 245, 245);
  ledsAll(COL_WHITE);

  pinMode(motorL_fwd, OUTPUT); pinMode(motorL_rev, OUTPUT);
  pinMode(motorR_fwd, OUTPUT); pinMode(motorR_rev, OUTPUT);
  pinMode(trigger, OUTPUT); pinMode(echo, INPUT);
  for(int i=0; i<sensorCount; i++){ pinMode(sensorPins[i], INPUT); }

  gripper.attach(gripperPin);
  gripperHoldOpen = true;
  gripper.write(gripOpen);

  stopMotors();
  state = WAIT_FLAG;
  setLedsForState(state);
  stateStart = millis();
}

// ==================== Loop ====================

void loop(){
  if(state == WAIT_FLAG){
    stopMotors();
    ledsAll(COL_BLUE);
    lastSonarPingMs = 0;   
    int ok = 0;
    for(int i = 0; i < 5; i++){
      if(checkFlagStart()) ok++;
    }
    if(ok >= 3){
      raceStarted = true;
      enterState(START_FORWARD);
    }
    return;
  }

  unsigned long now = millis();
  LineState line = readLine();
  updateMarkerStable();
  gripperHoldUpdate();

  // blind start
  if(state == START_FORWARD){
    driveForward(startL, startR);
    if(now - stateStart >= startBlindMs){
      stopMotors();
      enterState(START_GRAB);
    }
  }
  else if(state == START_GRAB){
    if(now - stateStart >= grabHoldMs){
      enterState(START_TURN_LEFT);
    }
  }
  else if(state == START_TURN_LEFT){
    turnLeftBlind(turnSpd);
    if(now - stateStart >= turnLeftMs){
      stopMotors();
      enterState(START_FORWARD_AFTER_TURN);
    }
  }
  else if(state == START_FORWARD_AFTER_TURN){
    driveForward(afterTurnL, afterTurnR);
    if(now - stateStart >= afterTurnForwardMs){
      stopMotors();
      enterState(RUN_FOLLOW);
    }
  }

  // run follow
  else if(state == RUN_FOLLOW){
    int d = readDistanceCm();
    if(d <= obstacleCm){
      stopMotors();
      enterState(START_GRAB);
      return;
}
//    int d = readDistanceCm();
//    if(d <= obstacleCm){
//      stopMotors();
//      enterState(AVOID_TURN_LEFT);
//      return;
    }
    handleLine(line);
    setLedsForLine(line);

    if(!dropArmed){
      if(now - runFollowStart >= dropMinMs){
        if(!markerStable){
          if(nonBlackSince == 0) nonBlackSince = now;
          if(now - nonBlackSince >= nonBlackArmMs) dropArmed = true;
        } else { nonBlackSince = 0; }
      }
    }

    if(dropArmed && markerStable){
      stopMotors();
      enterState(DROP_BACKUP_BEFORE_OPEN);
    }
  }

  // obstacle avoidance
  else if(state == AVOID_TURN_LEFT){
    obstaclePivotRight(avoidTurnRightL, avoidTurnRightR);
    if(now - stateStart >= avoidTurnLeftMs){
      stopMotors();
      enterState(AVOID_FORWARD_1);
    }
  }
  else if(state == AVOID_FORWARD_1){
    driveForward(avoidFwdL, avoidFwdR);
    if(now - stateStart >= avoidForward1Ms){
      stopMotors();
      enterState(AVOID_CURVE_RIGHT);
    }
  }
  else if(state == AVOID_CURVE_RIGHT){
    obstaclePivotLeft(avoidTurnLeftL, avoidTurnLeftR);
    readLine();
    if(lineSeenNow()){ enterState(RUN_FOLLOW); return; }
    if(now - stateStart >= avoidCurveRightMs){
      stopMotors();
      enterState(AVOID_FORWARD_2);
    }
  }
  else if(state == AVOID_FORWARD_2){
    driveForward(avoidFwdL, avoidFwdR);
    if(now - stateStart >= avoidForward2Ms){
      stopMotors();
      enterState(AVOID_TURN_BACK);
    }
  }
  else if(state == AVOID_TURN_BACK){
    motorSpeedAdjuster(motorL_fwd, motorL_rev, avoidTurnBackL, true,  invertLeft);
    motorSpeedAdjuster(motorR_fwd, motorR_rev, avoidTurnBackR, false, invertRight);
    readLine();
    if(lineSeenNow()){ enterState(RUN_FOLLOW); return; }
    if(now - stateStart >= avoidTurnBackMs){
      stopMotors();
      enterState(AVOID_SEEK_LINE);
    }
  }
  else if(state == AVOID_SEEK_LINE){
    driveForward(afterTurnL, afterTurnR);
    readLine();
    if(lineSeenNow()){ enterState(RUN_FOLLOW); }
    else if(now - stateStart >= avoidSeekMs){ enterState(RUN_FOLLOW); }
  }

  // drop sequence
  else if(state == DROP_BACKUP_BEFORE_OPEN){
    driveBackward(backL, backR);
    if(now - dropReverseStart >= releaseAfterReverseMs){
      enterState(DROP_OPEN_WHILE_BACKING);
    }
  }
  else if(state == DROP_OPEN_WHILE_BACKING){
    driveBackward(backL, backR);
    if(now - dropReverseStart >= (releaseAfterReverseMs + dropHoldMs)){
      enterState(DROP_BACKUP_AFTER_OPEN);
    }
  }
  else if(state == DROP_BACKUP_AFTER_OPEN){
    driveBackward(backL, backR);
    if(now - dropReverseStart >= backupMs){
      stopMotors();
      enterState(DONE);
    }
  }
  else if(state == DONE){
    stopMotors();
  }
}
