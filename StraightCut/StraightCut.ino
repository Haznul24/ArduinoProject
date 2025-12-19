#include <AccelStepper.h>

// --- Encoder 1 (KY-1: X & Y together) ---
#define enc1Clk 2
#define enc1Dt  3
#define enc1Sw  26

// --- Encoder 2 (KY-2: Y correction only) ---
#define enc2Clk 19
#define enc2Dt  18
#define enc2Sw  30

// --- X / Y Stepper Drivers ---
#define xStepPin 8
#define xDirPin  9
#define xEnPin   10
#define yStepPin 11
#define yDirPin  12
#define yEnPin   13

// --- Proximity sensors ---
#define xMinPin 35
#define xMaxPin 37
#define yMinPin 39
#define yMaxPin 41
const bool proxActiveLow = true;

// --- Relay outputs ---
#define relayGreen 45
#define relayRed   47
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

// --- Encoder state ---
volatile int8_t enc1Delta = 0;
volatile int8_t enc2Delta = 0;

// --- Steppers ---
AccelStepper stepX(AccelStepper::DRIVER, xStepPin, xDirPin);
AccelStepper stepY(AccelStepper::DRIVER, yStepPin, yDirPin);

long targetX = 0;
long targetY = 0;

// --- Proximity cache ---
bool xMinTriggered = false;
bool xMaxTriggered = false;
bool yMinTriggered = false;
bool yMaxTriggered = false;

// --- Lock state ---
bool motorsLocked = false;
unsigned long bothPressedSince = 0;
bool toggledOnThisPress = false;
const unsigned long bothPressDebounceMs = 50;

// ---------- Helpers ----------
inline bool proxTriggered(int pin) {
  return proxActiveLow ? digitalRead(pin) == LOW : digitalRead(pin) == HIGH;
}

inline uint8_t relayOn()  { return relayActiveLow ? LOW : HIGH; }
inline uint8_t relayOff() { return relayActiveLow ? HIGH : LOW; }

// ---------- Encoder ISRs ----------
void enc1Isr() {
  static uint8_t oldState = 3;
  static int8_t encVal = 0;

  oldState <<= 2;
  if (digitalRead(enc1Clk)) oldState |= 0x02;
  if (digitalRead(enc1Dt))  oldState |= 0x01;

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
  if (digitalRead(enc2Clk)) oldState |= 0x02;
  if (digitalRead(enc2Dt))  oldState |= 0x01;

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

// ---------- Homing ----------
void homeAxes() {
  stepX.setMaxSpeed(300);
  stepY.setMaxSpeed(300);

  stepX.moveTo(-100000);
  stepY.moveTo(-100000);

  while (!xMinTriggered || !yMinTriggered) {
    if (!xMinTriggered && proxTriggered(xMinPin)) {
      stepX.stop();
      xMinTriggered = true;
      stepX.setCurrentPosition(0);
    } else stepX.run();

    if (!yMinTriggered && proxTriggered(yMinPin)) {
      stepY.stop();
      yMinTriggered = true;
      stepY.setCurrentPosition(0);
    } else stepY.run();
  }

  targetX = stepsPerMm;
  targetY = stepsPerMm;
  stepX.moveTo(targetX);
  stepY.moveTo(targetY);

  while (stepX.distanceToGo() || stepY.distanceToGo()) {
    stepX.run();
    stepY.run();
  }
}

// ---------- UI / Control ----------
void updateRelays() {
  digitalWrite(relayGreen, motorsLocked ? relayOn() : relayOff());
  digitalWrite(relayRed,   motorsLocked ? relayOff() : relayOn());
}

void toggleMotorLock() {
  motorsLocked = !motorsLocked;
  updateRelays();
}

void readEncoders(int8_t &d1, int8_t &d2) {
  noInterrupts();
  d1 = enc1Delta; enc1Delta = 0;
  d2 = enc2Delta; enc2Delta = 0;
  interrupts();
}

// ---------- Motion ----------
void applyEncoderMovement(int8_t d1, int8_t d2) {
  if (motorsLocked) return;

  if (d1) {
    targetX += d1 * incXSteps;
    targetY += d1 * incXSteps;
  }
  if (d2) {
    targetY += d2 * incYSteps;
  }
}

void updateSteppers() {
  stepX.moveTo(targetX);
  stepY.moveTo(targetY);
  stepX.run();
  stepY.run();
}

// ---------- Setup / Loop ----------
void setup() {
  Serial.begin(115200);

  pinMode(enc1Clk, INPUT_PULLUP);
  pinMode(enc1Dt, INPUT_PULLUP);
  pinMode(enc1Sw, INPUT_PULLUP);

  pinMode(enc2Clk, INPUT_PULLUP);
  pinMode(enc2Dt, INPUT_PULLUP);
  pinMode(enc2Sw, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enc1Clk), enc1Isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc1Dt),  enc1Isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2Clk), enc2Isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc2Dt),  enc2Isr, CHANGE);

  pinMode(relayGreen, OUTPUT);
  pinMode(relayRed, OUTPUT);

  homeAxes();
}

void loop() {
  int8_t d1, d2;
  readEncoders(d1, d2);
  applyEncoderMovement(d1, d2);
  updateSteppers();
}
