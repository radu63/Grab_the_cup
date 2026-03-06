#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// ==================== Config ====================

#define POSFin 0b10001111

// --- Pins ---
const int NEO_PIN     = 4;
const int gripperPin  = 12;
const int motorL_fwd  = 6;
const int motorL_rev  = 5;
const int motorR_fwd  = 9;
const int motorR_rev  = 10;
const int trigPin     = 7;
const int echoPin     = 8;
const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

bool invertLeft  = false;
bool invertRight = false;

const int BLACK = 850;

const int gripOpen  = 120;
const int gripClose = 60;

const int FLAG_MIN_CM = 3;
const int FLAG_MAX_CM = 20;

// --- Timings ---
unsigned long startBlindMs       = 850;
unsigned long turnLeftMs         = 630;
unsigned long afterTurnForwardMs = 600;

// --- Speeds ---
float startL      = 0.95, startR     = 1.00;
float turnSpd     = 0.70;
float afterTurnL  = 0.70, afterTurnR = 0.65;
float backL       = 0.95, backR      = 1.00;

float followCenterL    = 0.95, followCenterR    = 1.00;
float followLeftTurnL  = 0.00, followLeftTurnR  = 1.00;  // full pivot left
float followRightTurnL = 1.00, followRightTurnR = 0.00;  // full pivot right

// --- Obstacle avoidance ---
int obstacleCm = 20;
unsigned long avoidTurnLeftMs   = 500;
unsigned long avoidForward1Ms   = 650;
unsigned long avoidCurveRightMs = 800;
unsigned long avoidForward2Ms   = 200;
unsigned long avoidTurnBackMs   = 0;
unsigned long avoidSeekMs       = 10;
float avoidFwdL = 0.85, avoidFwdR = 0.85;
float avoidTurnRightL = 0.50, avoidTurnRightR = 0.50;
float avoidTurnLeftL  = 0.00, avoidTurnLeftR  = 0.90;
float avoidTurnBackL  = 0.60, avoidTurnBackR  = 0.00;

// --- Drop ---
unsigned long backupMs              = 3000;
unsigned long releaseAfterReverseMs = 250;
unsigned long dropHoldMs            = 450;
unsigned long dropMinMs             = 2000;
unsigned long nonBlackArmMs         = 200;
int markerBlackCount = 7;
int markerStableMs   = 200;

const unsigned long SONAR_INTERVAL_MS = 300;
const unsigned long GRIPPER_HZ_MS     = 80;

// ==================== State Machine ====================

enum LineState { LINE_NONE, LINE_LEFT, LINE_RIGHT, LINE_CENTER, LINE_T, LINE_END };

enum State {
  STARTUP,
  WAIT_FLAG,
  START_OPEN_GRIPPER,
  START_FORWARD,
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

State state = STARTUP;
unsigned long stateStart = 0;

// ==================== Runtime Vars ====================

Servo gripper;
Adafruit_NeoPixel pixels(4, NEO_PIN, NEO_GRB + NEO_KHZ800);
uint32_t COL_WHITE, COL_BLUE, COL_GREEN, COL_YELLOW, COL_RED, COL_PURPLE;

int sensorValues[8];
int lastDistanceCm = 999;
unsigned long lastSonarPingMs = 0;

bool gripperHoldOpen = false;
unsigned long lastGripCmd = 0;

bool dropArmed = false;
unsigned long runFollowStart = 0;
unsigned long nonBlackSince  = 0;

bool markerStable = false;
unsigned long markerSince = 0;

unsigned long dropReverseStart = 0;
bool finishSent = false;

int startupStep = 0;
unsigned long startupTimer = 0;

// ==================== LEDs ====================

void ledsAll(uint32_t c) {
  for (int i = 0; i < 4; i++) pixels.setPixelColor(i, c);
  pixels.show();
}

void ledsLeftRight(uint32_t l, uint32_t r) {
  pixels.setPixelColor(3, l); pixels.setPixelColor(0, l);
  pixels.setPixelColor(1, r); pixels.setPixelColor(2, r);
  pixels.show();
}

void setLedsForState(State s) {
  switch (s) {
    case STARTUP:
    case WAIT_FLAG:
    case START_OPEN_GRIPPER:
    case START_FORWARD:
    case START_TURN_LEFT:
    case START_FORWARD_AFTER_TURN: ledsAll(COL_BLUE);   break;
    case RUN_FOLLOW:               ledsAll(COL_GREEN);  break;
    case AVOID_TURN_LEFT:
    case AVOID_FORWARD_1:
    case AVOID_CURVE_RIGHT:
    case AVOID_FORWARD_2:
    case AVOID_TURN_BACK:
    case AVOID_SEEK_LINE:          ledsAll(COL_RED);    break;
    default:                       ledsAll(COL_PURPLE); break;
  }
}

void setLedsForLine(LineState line) {
  if      (line == LINE_LEFT)  ledsLeftRight(COL_YELLOW, COL_GREEN);
  else if (line == LINE_RIGHT) ledsLeftRight(COL_GREEN,  COL_YELLOW);
  else                         ledsAll(COL_GREEN);
}

// ==================== Motors ====================

void motorSet(int fwd, int rev, float speed, bool forward, bool invert) {
  int pwm = constrain((int)(255.0f * speed), 0, 255);
  if (invert) forward = !forward;
  if (forward) { analogWrite(rev, 0); analogWrite(fwd, pwm); }
  else         { analogWrite(fwd, 0); analogWrite(rev, pwm); }
}

void stopMotors() {
  analogWrite(motorL_fwd, 0); analogWrite(motorL_rev, 0);
  analogWrite(motorR_fwd, 0); analogWrite(motorR_rev, 0);
}

void driveForward(float l, float r) {
  motorSet(motorL_fwd, motorL_rev, l, true,  invertLeft);
  motorSet(motorR_fwd, motorR_rev, r, true,  invertRight);
}

void driveBackward(float l, float r) {
  motorSet(motorL_fwd, motorL_rev, l, false, invertLeft);
  motorSet(motorR_fwd, motorR_rev, r, false, invertRight);
}

void turnLeft(float spd) {
  motorSet(motorL_fwd, motorL_rev, spd, false, invertLeft);
  motorSet(motorR_fwd, motorR_rev, spd, true,  invertRight);
}

void pivotRight(float l, float r) {
  motorSet(motorL_fwd, motorL_rev, l, true,  invertLeft);
  motorSet(motorR_fwd, motorR_rev, r, false, invertRight);
}

void pivotLeft(float l, float r) {
  motorSet(motorL_fwd, motorL_rev, l, true, invertLeft);
  motorSet(motorR_fwd, motorR_rev, r, true, invertRight);
}

// ==================== Sonar ====================

int readDistanceCm() {
  if (state != RUN_FOLLOW && state != WAIT_FLAG) return 999;
  unsigned long now = millis();
  if (now - lastSonarPingMs < SONAR_INTERVAL_MS) return lastDistanceCm;
  lastSonarPingMs = now;
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long us = pulseIn(echoPin, HIGH, 6000);
  lastDistanceCm = (us == 0) ? 999 : (int)((us * 0.034f) / 2.0f);
  return lastDistanceCm;
}

bool flagPresent() {
  // Temporarily allow sonar read during WAIT_FLAG
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long us = pulseIn(echoPin, HIGH, 6000);
  if (us == 0) return false;
  int d = (int)((us * 0.034f) / 2.0f);
  return (d >= FLAG_MIN_CM && d <= FLAG_MAX_CM);
}

// ==================== Line ====================

LineState readLine() {
  for (int i = 0; i < 8; i++) sensorValues[i] = analogRead(sensorPins[i]);
  bool right  = sensorValues[0] > BLACK || sensorValues[1] > BLACK || sensorValues[2] > BLACK;
  bool center = sensorValues[3] > BLACK || sensorValues[4] > BLACK;
  bool left   = sensorValues[5] > BLACK || sensorValues[6] > BLACK || sensorValues[7] > BLACK;
  if (left && center && right) return LINE_T;
  if (!left && !center && !right) return LINE_END;
  if (center) return LINE_CENTER;
  if (left)   return LINE_LEFT;
  if (right)  return LINE_RIGHT;
  return LINE_NONE;
}

bool lineSeenNow() {
  for (int i = 0; i < 8; i++) if (sensorValues[i] > BLACK) return true;
  return false;
}

void handleLine(LineState line) {
  if (line == LINE_LEFT) {
    motorSet(motorL_fwd, motorL_rev, followLeftTurnL,  false, invertLeft);
    motorSet(motorR_fwd, motorR_rev, followLeftTurnR,  true,  invertRight);
  } else if (line == LINE_RIGHT) {
    motorSet(motorL_fwd, motorL_rev, followRightTurnL, true,  invertLeft);
    motorSet(motorR_fwd, motorR_rev, followRightTurnR, false, invertRight);
  } else {
    driveForward(followCenterL, followCenterR);
  }
}

// ==================== Marker ====================

void updateMarkerStable() {
  unsigned long now = millis();
  int blackCount = 0;
  for (int i = 0; i < 8; i++) if (sensorValues[i] > BLACK) blackCount++;
  if (blackCount >= markerBlackCount) {
    if (markerSince == 0) markerSince = now;
    markerStable = (now - markerSince >= (unsigned long)markerStableMs);
  } else {
    markerSince  = 0;
    markerStable = false;
  }
}

// ==================== Gripper ====================

void gripperUpdate() {
  unsigned long now = millis();
  if (now - lastGripCmd < GRIPPER_HZ_MS) return;
  lastGripCmd = now;
  gripper.write(gripperHoldOpen ? gripOpen : gripClose);
}

// ==================== State Transitions ====================

void enterState(State s) {
  state      = s;
  stateStart = millis();
  switch (s) {
    case START_OPEN_GRIPPER:
      gripperHoldOpen = true;
      gripper.write(gripOpen);
      break;
    case RUN_FOLLOW:
      dropArmed       = false;
      runFollowStart  = millis();
      nonBlackSince   = 0;
      gripperHoldOpen = false;
      break;
    case DROP_BACKUP_BEFORE_OPEN:
      dropReverseStart = millis();
      gripperHoldOpen  = false;
      break;
    case DROP_OPEN_WHILE_BACKING:
      gripperHoldOpen = true;
      gripper.write(gripOpen);
      break;
    case DONE:
      if (!finishSent) {
        finishSent = true;
        Serial.write((uint8_t)POSFin);
        Serial.write((uint8_t)POSFin);
      }
      break;
    default: break;
  }
  setLedsForState(s);
}

// ==================== Setup ====================

void setup() {
  Serial.begin(9600);

  pixels.begin();
  pixels.setBrightness(40);
  COL_WHITE  = pixels.Color(245, 245, 245);
  COL_BLUE   = pixels.Color(0,   0,   240);
  COL_GREEN  = pixels.Color(0,   240, 0  );
  COL_YELLOW = pixels.Color(245, 170, 0  );
  COL_RED    = pixels.Color(245, 0,   0  );
  COL_PURPLE = pixels.Color(160, 0,   245);
  ledsAll(COL_WHITE);

  pinMode(motorL_fwd, OUTPUT); pinMode(motorL_rev, OUTPUT);
  pinMode(motorR_fwd, OUTPUT); pinMode(motorR_rev, OUTPUT);
  pinMode(trigPin,    OUTPUT); pinMode(echoPin,    INPUT);
  for (int i = 0; i < 8; i++) pinMode(sensorPins[i], INPUT);

  gripper.attach(gripperPin);
  stopMotors();

  state        = STARTUP;
  startupStep  = 0;
  startupTimer = millis();
  gripper.write(gripOpen);  // step 0: open
}

// ==================== Loop ====================

void loop() {
  unsigned long now = millis();

  // ── STARTUP: open → wait → close → wait → go to WAIT_FLAG ────────────
  if (state == STARTUP) {
    if (startupStep == 0 && now - startupTimer > 600) {
      gripper.write(gripClose);
      startupTimer = now;
      startupStep  = 1;
    } else if (startupStep == 1 && now - startupTimer > 600) {
      enterState(WAIT_FLAG);
    }
    return;
  }

  // ── WAIT_FLAG: wait until flag object is no longer detected ───────────
  if (state == WAIT_FLAG) {
    stopMotors();
    ledsAll(COL_BLUE);
    if (!flagPresent()) {
      enterState(START_OPEN_GRIPPER);
    }
    return;
  }

  // ── START_OPEN_GRIPPER: open gripper, wait for it to open ─────────────
  if (state == START_OPEN_GRIPPER) {
    if (now - stateStart >= 400) {
      enterState(START_FORWARD);
    }
    return;
  }

  // ── START_FORWARD: blind forward, snap to follow if line found ─────────
  if (state == START_FORWARD) {
    driveForward(startL, startR);
    LineState line = readLine();
    if (line != LINE_NONE && line != LINE_END) {
      stopMotors(); enterState(RUN_FOLLOW); return;
    }
    if (now - stateStart >= startBlindMs) {
      stopMotors(); enterState(START_TURN_LEFT);
    }
    return;
  }

  // ── START_TURN_LEFT: blind pivot left ─────────────────────────────────
  if (state == START_TURN_LEFT) {
    turnLeft(turnSpd);
    if (now - stateStart >= turnLeftMs) {
      stopMotors(); enterState(START_FORWARD_AFTER_TURN);
    }
    return;
  }

  // ── START_FORWARD_AFTER_TURN: forward until line or timeout ───────────
  if (state == START_FORWARD_AFTER_TURN) {
    driveForward(afterTurnL, afterTurnR);
    readLine();
    if (lineSeenNow()) {
      stopMotors(); enterState(RUN_FOLLOW); return;
    }
    if (now - stateStart >= afterTurnForwardMs) {
      stopMotors(); enterState(RUN_FOLLOW);
    }
    return;
  }

  // ── RUN_FOLLOW: line following + obstacle + drop detection ────────────
  if (state == RUN_FOLLOW) {
    LineState line = readLine();
    handleLine(line);
    setLedsForLine(line);
    gripperUpdate();
    updateMarkerStable();

    int d = readDistanceCm();
    if (d <= obstacleCm) {
      stopMotors(); enterState(AVOID_TURN_LEFT); return;
    }

    if (!dropArmed && now - runFollowStart >= dropMinMs) {
      if (!markerStable) {
        if (nonBlackSince == 0) nonBlackSince = now;
        if (now - nonBlackSince >= nonBlackArmMs) dropArmed = true;
      } else { nonBlackSince = 0; }
    }

    if (dropArmed && markerStable) {
      stopMotors(); enterState(DROP_BACKUP_BEFORE_OPEN);
    }
    return;
  }

  // ── OBSTACLE AVOIDANCE ────────────────────────────────────────────────
  if (state == AVOID_TURN_LEFT) {
    pivotRight(avoidTurnRightL, avoidTurnRightR);
    if (now - stateStart >= avoidTurnLeftMs) { stopMotors(); enterState(AVOID_FORWARD_1); }
    return;
  }
  if (state == AVOID_FORWARD_1) {
    driveForward(avoidFwdL, avoidFwdR);
    if (now - stateStart >= avoidForward1Ms) { stopMotors(); enterState(AVOID_CURVE_RIGHT); }
    return;
  }
  if (state == AVOID_CURVE_RIGHT) {
    pivotLeft(avoidTurnLeftL, avoidTurnLeftR);
    readLine();
    if (lineSeenNow())                         { enterState(RUN_FOLLOW); return; }
    if (now - stateStart >= avoidCurveRightMs) { stopMotors(); enterState(AVOID_FORWARD_2); }
    return;
  }
  if (state == AVOID_FORWARD_2) {
    driveForward(avoidFwdL, avoidFwdR);
    if (now - stateStart >= avoidForward2Ms) { stopMotors(); enterState(AVOID_TURN_BACK); }
    return;
  }
  if (state == AVOID_TURN_BACK) {
    motorSet(motorL_fwd, motorL_rev, avoidTurnBackL, true,  invertLeft);
    motorSet(motorR_fwd, motorR_rev, avoidTurnBackR, false, invertRight);
    readLine();
    if (lineSeenNow())                        { enterState(RUN_FOLLOW); return; }
    if (now - stateStart >= avoidTurnBackMs)  { stopMotors(); enterState(AVOID_SEEK_LINE); }
    return;
  }
  if (state == AVOID_SEEK_LINE) {
    driveForward(afterTurnL, afterTurnR);
    readLine();
    if (lineSeenNow() || now - stateStart >= avoidSeekMs) enterState(RUN_FOLLOW);
    return;
  }

  // ── DROP SEQUENCE ─────────────────────────────────────────────────────
  if (state == DROP_BACKUP_BEFORE_OPEN) {
    driveBackward(backL, backR);
    if (now - dropReverseStart >= releaseAfterReverseMs) enterState(DROP_OPEN_WHILE_BACKING);
    return;
  }
  if (state == DROP_OPEN_WHILE_BACKING) {
    driveBackward(backL, backR);
    if (now - dropReverseStart >= releaseAfterReverseMs + dropHoldMs) enterState(DROP_BACKUP_AFTER_OPEN);
    return;
  }
  if (state == DROP_BACKUP_AFTER_OPEN) {
    driveBackward(backL, backR);
    if (now - dropReverseStart >= backupMs) { stopMotors(); enterState(DONE); }
    return;
  }
  if (state == DONE) {
    stopMotors();
    return;
  }
}
