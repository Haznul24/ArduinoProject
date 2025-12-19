#include <AccelStepper.h>

const int stepPul = 8;
const int stepDir = 9;
const int stepEna = 10;

AccelStepper stepper(1, stepPul, stepDir);
const float maxSpeed = 5000;
const float acceleration = 50;
const float speed = 2500;

const int btn1 = 29;
const int btn2 = 31;

// --- X limit pins ---
const int X_MIN_PIN = 35; // stops negative direction (min)
const int X_MAX_PIN = 39; // stops positive direction (max)
const bool PROX_ACTIVE_LOW = true; // set false if your switches are active HIGH

// Helper: interpret raw pin according to active polarity
inline bool proxTriggeredRaw(int pin) {
  int v = digitalRead(pin);
  return PROX_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// Cached limit states
bool xMinTrig = false;
bool xMaxTrig = false;

// Mechanics for homing & step/mm
const long  STEPS_PER_REV = 800;     // microsteps per revolution
const float LEAD_MM       = 5.0;     // ballscrew 1605
const long  STEPS_PER_MM  = (long)(STEPS_PER_REV / LEAD_MM); // 160 steps per mm approx

int lastState = 0;  

const unsigned long DEBOUNCE_MS = 20UL;
unsigned long lastChangeB1 = 0;
unsigned long lastChangeB2 = 0;

// Raw & stable states
bool rawB1 = false, rawB2 = false;
bool stableB1 = false, stableB2 = false;

// last read values to detect edges for debounce timing
bool lastRawB1 = false, lastRawB2 = false;

int dirState = 0;     // 0=stop, -1=ccw, 1=cw

// homing timeout (ms)
const unsigned long HOMING_TIMEOUT_MS = 30000UL; // 30 seconds

void doHomingToMin() {
  Serial.println("Homing: searching for X_MIN...");

  // Ensure driver enabled and set safe homing speeds
  stepper.enableOutputs();
  stepper.setMaxSpeed(300);      // slow speed for homing
  stepper.setAcceleration(500);  // moderate accel for homing

  // Move negative a large amount
  stepper.moveTo(-100000L);

  unsigned long start = millis();
  bool homed = false;

  // run until min switch triggered or timeout
  while (!homed) {
    // check timeout
    if ((millis() - start) > HOMING_TIMEOUT_MS) {
      Serial.println("Homing TIMEOUT! Stop homing and continue.");
      stepper.stop();
      // leave position as-is (it may be far); disable outputs to be safe
      stepper.disableOutputs();
      return;
    }

    // read current state of min switch
    if (proxTriggeredRaw(X_MIN_PIN)) {
      // reached the min switch
      stepper.stop();                  // request stop
      // run until fully stopped (distanceToGo == 0)
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }
      Serial.println("X axis homed (MIN).");
      homed = true;
      break;
    } else {
      // continue moving
      stepper.run();
    }
  }

  // Set current position to zero at the switch
  stepper.setCurrentPosition(0);
  Serial.println("Position set to 0 at X_MIN.");

  // Move +1 mm away from the switch
  long oneMM = STEPS_PER_MM;
  if (oneMM <= 0) oneMM = 1; // safety
  Serial.println("Moving +1mm from home.");
  stepper.moveTo(oneMM);

  // run until arrived
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  Serial.println("Homing complete. Ready.");

  // restore normal motion params
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
  // leave outputs enabled so buttons can drive immediately
}

void setup() {
  Serial.begin(9600);
  Serial.println("=== Single Stepper with Limits & Homing ===");

  pinMode(stepEna, OUTPUT);
  stepper.setEnablePin(stepEna);
  stepper.disableOutputs(); // start disabled

  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);

  // limit switch pins
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(X_MAX_PIN, INPUT_PULLUP);

  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);

  dirState = 0;

  // Perform homing at startup
  doHomingToMin();

  // after homing ensure driver parameters are set for runtime
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
}

void loop() {
  readButtons();
  updateProximityStates();   // read limits
  action();
  enforceLimitSwitches();    // block motion into triggered endstops
  runStepper();
}

void readButtons() {
  // read raw (pressed = LOW)
  rawB1 = (digitalRead(btn1) == LOW);
  rawB2 = (digitalRead(btn2) == LOW);

  unsigned long now = millis();

  // B1 debounce
  if (rawB1 != lastRawB1) {
    lastChangeB1 = now;            // bounce started, reset timer
    lastRawB1 = rawB1;
  } else {
    if ((now - lastChangeB1) >= DEBOUNCE_MS) {
      // stable
      stableB1 = rawB1;
    }
  }

  // B2 debounce
  if (rawB2 != lastRawB2) {
    lastChangeB2 = now;
    lastRawB2 = rawB2;
  } else {
    if ((now - lastChangeB2) >= DEBOUNCE_MS) {
      stableB2 = rawB2;
    }
  }
}

void updateProximityStates() {
  xMinTrig = proxTriggeredRaw(X_MIN_PIN);
  xMaxTrig = proxTriggeredRaw(X_MAX_PIN);
  // debug:
  // if (xMinTrig) Serial.println("X_MIN active");
  // if (xMaxTrig) Serial.println("X_MAX active");
}

void action(){
  if (stableB1 && !stableB2){
    // request CW (positive)
    if (dirState != 1){
      dirState = 1;
      stepper.enableOutputs();
    }
  }
  else if (!stableB1 && stableB2){
    // request CCW (negative)
    if (dirState != -1){
      dirState = -1;
      stepper.enableOutputs();
    }
  }
  else{
    dirState = 0;
  }
}

void enforceLimitSwitches() {
  // If a limit is triggered, block motion INTO that limit
  if (dirState == 1 && xMaxTrig) {
    // trying to move CW (+) but max is triggered -> stop
    Serial.println("X_MAX triggered: blocking + movement");
    dirState = 0;
    stepper.disableOutputs();
  }
  else if (dirState == -1 && xMinTrig) {
    // trying to move CCW (-) but min is triggered -> stop
    Serial.println("X_MIN triggered: blocking - movement");
    dirState = 0;
    stepper.disableOutputs();
  }
  // otherwise leave dirState alone (movement allowed)
}

void runStepper(){
  if (dirState == 1){
    stepper.setSpeed(speed);
    stepper.runSpeed();
  }
  else if (dirState == -1){
    stepper.setSpeed(-speed);
    stepper.runSpeed();
  }
  else{
    stepper.disableOutputs(); 
  }
}
