// Arduino Code for Straight Cut Machine

#include <AccelStepper.h>

// --- Encoder 1 (KY-1: X & Y together) ---
#define ENC1_CLK 2
#define ENC1_DT 3
#define ENC1_SW 26

// --- Encoder 2 (KY-2: Y correction only) ---
#define ENC2_CLK 19
#define ENC2_DT 18
#define ENC2_SW 30

// --- X / Y Stepper Drivers ---
#define X_STEP 8
#define X_DIR  9
#define X_EN  10
#define Y_STEP 11
#define Y_DIR  12
#define Y_EN   13

// --- Proximity sensor pins ---
#define X_MIN_PIN 35
#define X_MAX_PIN 37
#define Y_MIN_PIN 39
#define Y_MAX_PIN 41
const bool proxActiveLow = true;

// --- Speed mode indicator relays ---
#define RELAY1 45
#define RELAY2 47
const bool relayActiveLow = true;

// --- Mechanics ---
const long stepsPerRev = 800;
const float leadMm = 5.0;
const long stepsPerMm = stepsPerRev / leadMm;

// --- Encoder increments ---
const float incXMm = 0.5;
const long incXSteps = incXMm * stepsPerMm;

const float incYMm = 0.5;
const long incYSteps = incYMm * stepsPerMm;

// --- Encoder variables ---
volatile int8_t enc1Delta = 0;
volatile int8_t enc2Delta = 0;
const unsigned long minIsrIntervalUs = 0UL;

// --- Stepper objects ---
AccelStepper stepX(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper stepY(AccelStepper::DRIVER, Y_STEP, Y_DIR);
long targetX = 0;
long targetY = 0;

// --- Proximity cached states ---
bool xMinTrig = false;
bool xMaxTrig = false;
bool yMinTrig = false;
bool yMaxTrig = false;

// --- Lock state ---
bool motorsLocked = false;
unsigned long bothPressedSince = 0;
bool toggledOnThisPress = false;
const unsigned long bothPressDebounceMs = 50;

// === Helpers ===
inline bool proxTriggeredRaw(int pin) {
  int v = digitalRead(pin);
  return proxActiveLow ? (v == LOW) : (v == HIGH);
}

inline uint8_t relayActiveLevel() {
  return relayActiveLow ? LOW : HIGH;
}

inline uint8_t relayInactiveLevel() {
  return relayActiveLow ? HIGH : LOW;
}

// === Encoder ISRs ===
void enc1Isr() {
  static uint8_t oldState = 3;
  static int8_t encVal = 0;

  oldState <<= 2;
  if (digitalRead(ENC1_CLK)) oldState |= 0x02;
  if (digitalRead(ENC1_DT))  oldState |= 0x01;

  static const int8_t states[] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };

  encVal += states[oldState & 0x0F];

  if (encVal > 3) { enc1Delta++; encVal = 0; }
  if (encVal < -3) { enc1Delta--; encVal = 0; }

  oldState &= 0x0F;
}

void enc2Isr() {
  static uint8_t oldState = 3;
  static int8_t encVal = 0;

  oldState <<= 2;
  if (digitalRead(ENC2_CLK)) oldState |= 0x02;
  if (digitalRead(ENC2_DT))  oldState |= 0x01;

  static const int8_t states[] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };

  encVal += states[oldState & 0x0F];

  if (encVal > 3) { enc2Delta++; encVal = 0; }
  if (encVal < -3) { enc2Delta--; encVal = 0; }

  oldState &= 0x0F;
}


// === Homing ===
void homeAxes() {
  Serial.println("Starting homing...");

  stepX.setMaxSpeed(300);
  stepY.setMaxSpeed(300);
  stepX.setAcceleration(500);
  stepY.setAcceleration(500);

  stepX.moveTo(-100000);
  stepY.moveTo(-100000);

  bool xHomed = false;
  bool yHomed = false;

  while (!xHomed || !yHomed) {
    if (!xHomed) {
      if (proxTriggeredRaw(X_MIN_PIN)) {
        stepX.stop();
        stepX.setCurrentPosition(0);
        xHomed = true;
        Serial.println("X homed");
      } else {
        stepX.run();
      }
    }

    if (!yHomed) {
      if (proxTriggeredRaw(Y_MIN_PIN)) {
        stepY.stop();
        stepY.setCurrentPosition(0);
        yHomed = true;
        Serial.println("Y homed");
      } else {
        stepY.run();
      }
    }
  }

  delay(200);

  targetX = stepsPerMm;
  targetY = stepsPerMm;
  stepX.moveTo(targetX);
  stepY.moveTo(targetY);

  while (stepX.distanceToGo() || stepY.distanceToGo()) {
    stepX.run();
    stepY.run();
  }

  Serial.println("Homing complete. Ready.");
}

// === UI / Control Helpers ===
void setModeRelays() {
  digitalWrite(RELAY1, motorsLocked ? relayActiveLevel() : relayInactiveLevel());
  digitalWrite(RELAY2, motorsLocked ? relayInactiveLevel() : relayActiveLevel());
}

void applyMotorEnablePins() {
  if (motorsLocked) {
    stepX.enableOutputs();
    stepY.enableOutputs();
  } else {
    stepX.disableOutputs();
    stepY.disableOutputs();
  }
}

void handleSimultaneousButtonToggle() {
  bool sw1 = digitalRead(ENC1_SW) == LOW;
  bool sw2 = digitalRead(ENC2_SW) == LOW;

  if (sw1 && sw2) {
    if (!toggledOnThisPress) {
      if (bothPressedSince == 0) bothPressedSince = millis();
      if (millis() - bothPressedSince >= bothPressDebounceMs) {
        motorsLocked = !motorsLocked;
        setModeRelays();
        toggledOnThisPress = true;
      }
    }
  } else {
    bothPressedSince = 0;
    toggledOnThisPress = false;
  }
}

void readEncoderDeltas(int8_t &d1, int8_t &d2) {
  noInterrupts();
  int8_t t1 = enc1Delta;
  int8_t t2 = enc2Delta;
  enc1Delta = 0;
  enc2Delta = 0;
  interrupts();

  d1 = t1;
  d2 = t2;

  applyMotorEnablePins();
}

void updateProximityStates() {
  xMinTrig = proxTriggeredRaw(X_MIN_PIN);
  xMaxTrig = proxTriggeredRaw(X_MAX_PIN);
  yMinTrig = proxTriggeredRaw(Y_MIN_PIN);
  yMaxTrig = proxTriggeredRaw(Y_MAX_PIN);
}

void applyEncoderDeltas(int8_t d1, int8_t d2) {
  if (motorsLocked) return;

  if (d1) {
    targetX += d1 * incXSteps;
    targetY += d1 * incXSteps;
  }

  if (d2) {
    targetY += d2 * incYSteps;
  }
}

void enforceLimitSwitches() {
  long posX = stepX.currentPosition();
  long posY = stepY.currentPosition();

  if (xMaxTrig && targetX > posX) targetX = posX;
  if (xMinTrig && targetX < posX) targetX = posX;
  if (yMaxTrig && targetY > posY) targetY = posY;
  if (yMinTrig && targetY < posY) targetY = posY;
}

void applySafetyLimits() {
  const long safeMin = -1000000L;
  const long safeMax =  1000000L;

  targetX = constrain(targetX, safeMin, safeMax);
  targetY = constrain(targetY, safeMin, safeMax);
}

void updateSteppers() {
  stepX.moveTo(targetX);
  stepY.moveTo(targetY);
  stepX.run();
  stepY.run();
}

// === Arduino Setup / Loop ===
void setup() {
  Serial.begin(115200);

  pinMode(ENC1_CLK, INPUT_PULLUP);
  pinMode(ENC1_DT, INPUT_PULLUP);
  pinMode(ENC1_SW, INPUT_PULLUP);
  pinMode(ENC2_CLK, INPUT_PULLUP);
  pinMode(ENC2_DT, INPUT_PULLUP);
  pinMode(ENC2_SW, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_CLK), enc1Isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_DT),  enc1Isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CLK), enc2Isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_DT),  enc2Isr, CHANGE);

  pinMode(X_EN, OUTPUT);
  pinMode(Y_EN, OUTPUT);

  digitalWrite(X_EN, LOW);
  digitalWrite(Y_EN, LOW);

  stepX.setEnablePin(X_EN);
  stepY.setEnablePin(Y_EN);

  stepX.setPinsInverted(false, false, true);
  stepY.setPinsInverted(true, false, true);

  stepX.setMaxSpeed(1000);
  stepY.setMaxSpeed(1000);
  stepX.setAcceleration(1500);
  stepY.setAcceleration(1500);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  digitalWrite(RELAY1, relayInactiveLevel());
  digitalWrite(RELAY2, relayInactiveLevel());

  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(X_MAX_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MAX_PIN, INPUT_PULLUP);

  homeAxes();
}

void loop() {
  int8_t d1 = 0;
  int8_t d2 = 0;
  readEncoderDeltas(d1, d2);

  // Update sensors & UI
  updateProximityStates();

  // Handle simultaneous press toggle (locks/unlocks motors & updates relays)
  handleSimultaneousButtonToggle();

  // Apply encoder inputs to targets
  applyEncoderDeltas(d1, d2);

  // Enforce safety / limits
  enforceLimitSwitches();
  applySafetyLimits();

  // Move steppers
  updateSteppers();
}
