// Arduino code for Lathe Machine

#include <AccelStepper.h>

// --- Stepper driver pins ---
#define STEP_PUL_PIN 8
#define STEP_DIR_PIN 9
#define STEP_ENA_PIN 10

// --- Control buttons ---
#define BTN_FORWARD_PIN 29
#define BTN_REVERSE_PIN 31

// --- X limit pins ---
#define X_MIN_PIN 35
#define X_MAX_PIN 39
const bool proxActiveLow = true;

// --- Stepper ---
AccelStepper stepper(AccelStepper::DRIVER, STEP_PUL_PIN, STEP_DIR_PIN);
const float maxSpeed = 5000.0f;
const float acceleration = 50.0f;
const float runSpeed = 2500.0f;

// --- Mechanics ---
const long stepsPerRev = 800;
const float leadMm = 5.0f;
const long stepsPerMm = (long)(stepsPerRev / leadMm);

// --- Button debounce ---
const unsigned long debounceMs = 20;
unsigned long lastChangeBtnForward = 0;
unsigned long lastChangeBtnReverse = 0;

bool rawBtnForward = false;
bool rawBtnReverse = false;
bool stableBtnForward = false;
bool stableBtnReverse = false;
bool lastRawBtnForward = false;
bool lastRawBtnReverse = false;

// --- Limit states ---
bool xMinTriggered = false;
bool xMaxTriggered = false;

// --- Motion state ---
//  1 = +X, -1 = -X, 0 = stop
int directionState = 0;

// --- Homing timeout ---
const unsigned long homingTimeoutMs = 30000;

// === Helpers ===
inline bool proxTriggered(int pin) {
  return proxActiveLow ? (digitalRead(pin) == LOW)
                       : (digitalRead(pin) == HIGH);
}

// === Homing (to X_MIN) ===
void homeToMin() {
  Serial.println("Homing: X_MIN");

  stepper.enableOutputs();
  stepper.setMaxSpeed(300);
  stepper.setAcceleration(500);

  stepper.moveTo(-100000L);
  unsigned long startTime = millis();

  while (true) {
    if (millis() - startTime > homingTimeoutMs) {
      Serial.println("Homing timeout");
      stepper.stop();
      stepper.disableOutputs();
      return;
    }

    if (proxTriggered(X_MIN_PIN)) {
      stepper.stop();
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }
      break;
    }

    stepper.run();
  }

  stepper.setCurrentPosition(0);

  // Move +1mm away from switch
  long oneMmSteps = max(stepsPerMm, 1L);
  stepper.moveTo(oneMmSteps);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }

  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
}

// === Setup ===
void setup() {
  Serial.begin(9600);
  Serial.println("Single Stepper with Limits");

  pinMode(STEP_ENA_PIN, OUTPUT);
  stepper.setEnablePin(STEP_ENA_PIN);
  stepper.disableOutputs();

  pinMode(BTN_FORWARD_PIN, INPUT_PULLUP);
  pinMode(BTN_REVERSE_PIN, INPUT_PULLUP);

  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(X_MAX_PIN, INPUT_PULLUP);

  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);

  homeToMin();
}

// === Read and debounce buttons ===
void readButtons() {
  rawBtnForward = (digitalRead(BTN_FORWARD_PIN) == LOW);
  rawBtnReverse = (digitalRead(BTN_REVERSE_PIN) == LOW);

  unsigned long now = millis();

  if (rawBtnForward != lastRawBtnForward) {
    lastChangeBtnForward = now;
    lastRawBtnForward = rawBtnForward;
  } else if (now - lastChangeBtnForward >= debounceMs) {
    stableBtnForward = rawBtnForward;
  }

  if (rawBtnReverse != lastRawBtnReverse) {
    lastChangeBtnReverse = now;
    lastRawBtnReverse = rawBtnReverse;
  } else if (now - lastChangeBtnReverse >= debounceMs) {
    stableBtnReverse = rawBtnReverse;
  }
}

// === Update limit switch states ===
void updateLimitStates() {
  xMinTriggered = proxTriggered(X_MIN_PIN);
  xMaxTriggered = proxTriggered(X_MAX_PIN);
}

// === Decide motion direction ===
void updateDirection() {
  if (stableBtnForward && !stableBtnReverse) {
    directionState = 1;
    stepper.enableOutputs();
  }
  else if (!stableBtnForward && stableBtnReverse) {
    directionState = -1;
    stepper.enableOutputs();
  }
  else {
    directionState = 0;
  }
}

// === Block motion into active limits ===
void enforceLimits() {
  if (directionState == 1 && xMaxTriggered) {
    directionState = 0;
    stepper.disableOutputs();
  }
  else if (directionState == -1 && xMinTriggered) {
    directionState = 0;
    stepper.disableOutputs();
  }
}

// === Run stepper ===
void runStepper() {
  if (directionState == 1) {
    stepper.setSpeed(runSpeed);
    stepper.runSpeed();
  }
  else if (directionState == -1) {
    stepper.setSpeed(-runSpeed);
    stepper.runSpeed();
  }
  else {
    stepper.disableOutputs();
  }
}

// === Main loop ===
void loop() {
  // Read buttons
  readButtons();

  // Enforce limitrs
  updateLimitStates();
  enforceLimits();

  // Update direction motor
  updateDirection();
  
  // Move steppers
  runStepper();
}
