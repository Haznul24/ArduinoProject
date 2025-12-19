// Arduino Code for Straight Cut Machine, included two rotary encoder, two stepper motor,
// four limit sensor (proximity), and two relay for LED control.

#include <AccelStepper.h>

// --- Encoder 1 (KY-1: X & Y together) ---
#define ENC1_CLK 2      // MUST be interrupt pin on Mega
#define ENC1_DT 3
#define ENC1_SW 26

// --- Encoder 2 (KY-2: Y correction only) ---
#define ENC2_CLK 19      // MUST be interrupt pin on Mega
#define ENC2_DT 18
#define ENC2_SW 30

// --- X / Y Stepper Drivers ---
#define X_STEP 8
#define X_DIR  9
#define X_EN  10
#define Y_STEP 11
#define Y_DIR  12
#define Y_EN   13

// --- Proximity sensor pins (Arduino Mega digital pins) ---
#define X_MIN_PIN 35   // stops negative direction (min)
#define X_MAX_PIN 37   // stops positive direction (max)
#define Y_MIN_PIN 39
#define Y_MAX_PIN 41
const bool PROX_ACTIVE_LOW = true;

// Helper: interpret raw pin according to active polarity
inline bool proxTriggeredRaw(int pin) {
  int v = digitalRead(pin);
  return PROX_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

// --- Speed mode indicator LEDs (relays) ---
#define RELAY1 45   // Green
#define RELAY2 47   // Red
const bool RELAY_ACTIVE_LOW = true;

inline uint8_t relayActiveLevel()   { return RELAY_ACTIVE_LOW ? LOW : HIGH; }
inline uint8_t relayInactiveLevel() { return RELAY_ACTIVE_LOW ? HIGH : LOW; }

// --- Mechanics ---
const long  STEPS_PER_REV = 800;     // microsteps per revolution
const float LEAD_MM       = 5.0;     // ballscrew 1605
const long  STEPS_PER_MM  = STEPS_PER_REV / LEAD_MM; // 160

// --- KY1 correction ---
const float INC_X_MM = 0.5;
const long  INC_X_STEPS = INC_X_MM * STEPS_PER_MM; // 80 steps per click

// --- KY2 correction ---
const float INC_Y_MM = 0.5;
const long  INC_Y_STEPS = INC_Y_MM * STEPS_PER_MM; // 80 steps per click

// --- Encoder variables ---
volatile int8_t enc1_delta = 0;
volatile int8_t enc2_delta = 0;
const unsigned long MIN_ISR_INTERVAL_US = 0UL;

// --- Stepper objects ---
AccelStepper stepX(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper stepY(AccelStepper::DRIVER, Y_STEP, Y_DIR);
long tgtX = 0, tgtY = 0;

// --- Proximity cached states (updated each loop) ---
bool xMinTrig = false, xMaxTrig = false, yMinTrig = false, yMaxTrig = false;

// --- Lock state ---
bool motorsLocked = false; // false = unlocked (motors can move), true = locked
// Debounce/toggle helpers for simultaneous-press detection:
static unsigned long bothPressedSince = 0;
static bool toggledOnThisPress = false;
const unsigned long BOTH_PRESS_DEBOUNCE_MS = 50; // stable press duration

// === ISR ===
void enc1ISR() {
  // static variables persist across calls and are local to the ISR scope
  static uint8_t old_AB = 3;             // previous state bits (A=bit1, B=bit0), init 0b11 (pullups)
  static int8_t encval = 0;              // small accumulator of micro-steps
  static unsigned long lastIsrMicros = 0;

  unsigned long now = micros();
  if (MIN_ISR_INTERVAL_US && (now - lastIsrMicros < MIN_ISR_INTERVAL_US)) return;
  lastIsrMicros = now;

  // Build 4-bit index: (prev << 2) | curr
  old_AB <<= 2;
  if (digitalRead(ENC1_CLK)) old_AB |= 0x02; // bit1 = A (CLK)
  if (digitalRead(ENC1_DT))  old_AB |= 0x01; // bit0 = B (DT)

  // Lookup table maps transition to +1, -1, or 0
  // index = old_AB & 0x0F
  static const int8_t enc_states[] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };

  encval += enc_states[ old_AB & 0x0F ];

  // if we've accumulated 4 consistent micro-steps -> a full detent
  if (encval > 3) {
    enc1_delta++;    // one detent forward
    encval = 0;
  } else if (encval < -3) {
    enc1_delta--;    // one detent backward
    encval = 0;
  }

  // Keep only lower bits of old_AB (so it doesn't grow forever)
  old_AB &= 0x0F;
}

void enc2ISR() {
  static uint8_t old_AB = 3;
  static int8_t encval = 0;
  static unsigned long lastIsrMicros = 0;

  unsigned long now = micros();
  if (MIN_ISR_INTERVAL_US && (now - lastIsrMicros < MIN_ISR_INTERVAL_US)) return;
  lastIsrMicros = now;

  old_AB <<= 2;
  if (digitalRead(ENC2_CLK)) old_AB |= 0x02; // A
  if (digitalRead(ENC2_DT))  old_AB |= 0x01; // B

  static const int8_t enc_states[] = {
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
  };

  encval += enc_states[ old_AB & 0x0F ];

  if (encval > 3) {
    enc2_delta++;
    encval = 0;
  } else if (encval < -3) {
    enc2_delta--;
    encval = 0;
  }

  old_AB &= 0x0F;
}

void homeAxes() {
  Serial.println("Starting homing...");

  // Set slow speed for homing
  stepX.setMaxSpeed(300);
  stepY.setMaxSpeed(300);
  stepX.setAcceleration(500);
  stepY.setAcceleration(500);

  // Move negative
  stepX.moveTo(-100000);  // large negative target
  stepY.moveTo(-100000);

  bool xHomed = false;
  bool yHomed = false;

  while (!xHomed || !yHomed) {
    if (!xHomed) {
      if (proxTriggeredRaw(X_MIN_PIN)) {
        stepX.stop();  // Stop the stepper
        xHomed = true;
        Serial.println("X axis homed.");
      } else {
        stepX.run();
      }
    }

    if (!yHomed) {
      if (proxTriggeredRaw(Y_MIN_PIN)) {
        stepY.stop();
        yHomed = true;
        Serial.println("Y axis homed.");
      } else {
        stepY.run();
      }
    }
  }

  // Reset position to 0,0
  stepX.setCurrentPosition(0);
  stepY.setCurrentPosition(0);
  tgtX = 0;
  tgtY = 0;

  delay(200); // brief pause

  // Move 1mm away from home
  tgtX = STEPS_PER_MM;
  tgtY = STEPS_PER_MM;
  stepX.moveTo(tgtX);
  stepY.moveTo(tgtY);

  Serial.println("Homing complete. Moving to +1mm position.");
  while (stepX.distanceToGo() != 0 || stepY.distanceToGo() != 0) {
    stepX.run();
    stepY.run();
  }

  Serial.println("Ready.");
}



void setup() {
  Serial.begin(115200);
  Serial.println("=== Straight Cut Machine ===");

  // Encoder setup
  pinMode(ENC1_CLK, INPUT_PULLUP);
  pinMode(ENC1_DT, INPUT_PULLUP);
  pinMode(ENC1_SW, INPUT_PULLUP);

  pinMode(ENC2_CLK, INPUT_PULLUP);
  pinMode(ENC2_DT, INPUT_PULLUP);
  pinMode(ENC2_SW, INPUT_PULLUP);

  // Attach interrupts on BOTH signals for each encoder (CHANGE)
  attachInterrupt(digitalPinToInterrupt(ENC1_CLK), enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_DT),  enc1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_CLK), enc2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_DT),  enc2ISR, CHANGE);

  // Stepper pins
  pinMode(X_STEP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(X_EN, OUTPUT);

  pinMode(Y_STEP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);
  pinMode(Y_EN, OUTPUT);

  // Enable drivers
  digitalWrite(X_EN, LOW);
  digitalWrite(Y_EN, LOW);

  stepX.setEnablePin(X_EN);
  stepY.setEnablePin(Y_EN);

  stepX.setPinsInverted(false, false, true);
  stepY.setPinsInverted(true, false, true);

  stepX.setAcceleration(1500);
  stepY.setAcceleration(1500);
  stepX.setMaxSpeed(1000);
  stepY.setMaxSpeed(1000);

  // Relays
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  digitalWrite(RELAY1, relayInactiveLevel());
  digitalWrite(RELAY2, relayInactiveLevel());

  // Proximity sensors
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(X_MAX_PIN, INPUT_PULLUP);
  pinMode(Y_MIN_PIN, INPUT_PULLUP);
  pinMode(Y_MAX_PIN, INPUT_PULLUP);

  homeAxes();

  Serial.println("KY1 = X & Y move together");
  Serial.println("KY2 = Y");
 
}

// === LED helper ===
void setModeRelays() {
  if (motorsLocked) {
    digitalWrite(RELAY1, relayActiveLevel());   // green ON
    digitalWrite(RELAY2, relayInactiveLevel()); // red OFF
  } else {
    digitalWrite(RELAY1, relayInactiveLevel()); // green OFF
    digitalWrite(RELAY2, relayActiveLevel());   // red ON
  }
}

void applyMotorEnablePins() {
  if (motorsLocked) {
    stepX.enableOutputs(); // disable driver
    stepY.enableOutputs();
  } else {
    stepX.disableOutputs();  // enable driver
    stepY.disableOutputs();
  }
}

// Detect simultaneous button press of both encoders and toggle lock state
void handleSimultaneousButtonToggle() {
  bool sw1 = (digitalRead(ENC1_SW) == LOW); // pressed = LOW (INPUT_PULLUP)
  bool sw2 = (digitalRead(ENC2_SW) == LOW); // pressed = LOW

  bool nowBoth = sw1 && sw2;

  if (nowBoth) {
    if (!toggledOnThisPress) {
      // start timing when both became pressed
      if (bothPressedSince == 0) bothPressedSince = millis();
      // if stable for debounce time, toggle lock once
      if (millis() - bothPressedSince >= BOTH_PRESS_DEBOUNCE_MS) {
        motorsLocked = !motorsLocked;
        Serial.print("Motors ");
        Serial.println(motorsLocked ? "LOCKED" : "UNLOCKED");
        // update hardware outputs
        setModeRelays();
        toggledOnThisPress = true; // avoid re-toggling while buttons held
      }
    }
  } else {
    // reset so the next simultaneous press can toggle again
    bothPressedSince = 0;
    toggledOnThisPress = false;
  }
}

// Read and clear encoder deltas
void readEncoderDeltas(int8_t &d1, int8_t &d2) {
  noInterrupts();
  long t1 = enc1_delta;
  long t2 = enc2_delta;
  enc1_delta = 0;
  enc2_delta = 0;
  interrupts();

  // clamp to int8 for the rest of the program (very unlikely to overflow between reads)
  if (t1 > 127) t1 = 127;
  if (t1 < -128) t1 = -128;
  if (t2 > 127) t2 = 127;
  if (t2 < -128) t2 = -128;

  d1 = (int8_t)t1;
  d2 = (int8_t)t2;

  applyMotorEnablePins();
}

// Update cached proximity sensor states
void updateProximityStates() {
  xMinTrig = proxTriggeredRaw(X_MIN_PIN);
  xMaxTrig = proxTriggeredRaw(X_MAX_PIN);
  yMinTrig = proxTriggeredRaw(Y_MIN_PIN);
  yMaxTrig = proxTriggeredRaw(Y_MAX_PIN);
}


// Apply encoder deltas to target positions
void applyEncoderDeltas(int8_t d1, int8_t d2) {
  if (!motorsLocked) {
    // KY-1 → X & Y together
    if (d1) {
      tgtX += (long)d1 * INC_X_STEPS;
      tgtY += (long)d1 * INC_X_STEPS;
      Serial.print("KY1 → tgtX: ");
      Serial.print(tgtX);
      Serial.print("  tgtY: ");
      Serial.println(tgtY);
    }

    // KY-2 → Y only (fine adjustment)
    if (d2) {
      tgtY += (long)d2 * INC_Y_STEPS;
      Serial.print("KY2 → tgtY: ");
      Serial.println(tgtY);
    }
  }
  else {
    return;
  }
}

// Enforce limit-switch logic (blocks movement into triggers)
void enforceLimitSwitches() {
  long posX = stepX.currentPosition();
  long posY = stepY.currentPosition();

  // X axis
  if (xMaxTrig && tgtX > posX) {
    tgtX = posX;
    Serial.println("X_MAX triggered: blocking +X movement");
  }
  if (xMinTrig && tgtX < posX) {
    tgtX = posX;
    Serial.println("X_MIN triggered: blocking -X movement");
  }

  // Y axis
  if (yMaxTrig && tgtY > posY) {
    tgtY = posY;
    Serial.println("Y_MAX triggered: blocking +Y movement");
  }
  if (yMinTrig && tgtY < posY) {
    tgtY = posY;
    Serial.println("Y_MIN triggered: blocking -Y movement");
  }
}

// Apply global safety limits to avoid numeric overflow
void applySafetyLimits() {
  const long SAFE_MIN = -1000000L;
  const long SAFE_MAX =  1000000L;
  if (tgtX < SAFE_MIN) tgtX = SAFE_MIN;
  if (tgtX > SAFE_MAX) tgtX = SAFE_MAX;
  if (tgtY < SAFE_MIN) tgtY = SAFE_MIN;
  if (tgtY > SAFE_MAX) tgtY = SAFE_MAX;
}

// Update steppers to move toward target positions
void updateSteppers() {
  stepX.moveTo(tgtX);
  stepY.moveTo(tgtY);
  stepX.run();
  stepY.run();
}

void loop() {
  // Read encoder deltas
  int8_t d1 = 0, d2 = 0;
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