// Shining Machine code, without keypad
// Input parameter using serial

#include <AccelStepper.h>

/* ====== HARDWARE PIN CONFIGURATION ====== */
// Stepper pins (change to match your wiring)
const uint8_t TABLE_STEP_PIN = 8;
const uint8_t TABLE_DIR_PIN  = 9;
const uint8_t TABLE_ENA_PIN  = 10;
const uint8_t SEED_STEP_PIN  = 11;
const uint8_t SEED_DIR_PIN   = 12;
const uint8_t SEED_ENA_PIN   = 13;

// DC motor
const uint8_t DC_PWM_PIN = 7;   // PWM to MOSFET gate

// Solenoid eject (optional)
const uint8_t EJECT_PIN = 24;
const uint8_t RelayMotor = 25;

// Start / Stop buttons (active LOW with INPUT_PULLUP)
const uint8_t START_PIN = 52;
const uint8_t STOP_PIN  = 53;

// Potentiometer (optional) for D-mode live tuning
const uint8_t POT_PIN = A0;

/* ====== TABLE HOMING SWITCH ====== */
// Use digital pin 37 (as you specified)
const uint8_t TABLE_HOME_PIN = 37;
// true = switch reads LOW when active (common with INPUT_PULLUP wiring)
const bool HOME_ACTIVE_LOW = true;

inline bool tableHomeTriggered() {
  int v = digitalRead(TABLE_HOME_PIN);
  return HOME_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

/* ====== MECHANICAL CONFIG (user-provided) ====== */
const uint32_t STEPPER_PULSES_PER_REV = 800UL; // 800 pulses/rev (driver pulses per motor revolution)
const uint32_t stepsPerRev = STEPPER_PULSES_PER_REV; // alias

// 45° index = 1/8 revolution
const uint32_t TABLE_STEPS_PER_INDEX = stepsPerRev / 8UL; // should be integer (800/8 = 100)

/* ====== DEFAULT PARAMETERS (editable via Serial) ====== */
unsigned long intervalSeconds = 5; // A: seconds between table index (default 2 minutes)
int seedRPM = 240;     // B: seed stepper RPM (0..1500)
int tableMoveRPM = 120; // C: table move RPM (controls move speed)
int dcPercent = 80;    // D: DC motor 0..100%

/* ====== INTERNAL / TIMING ====== */
AccelStepper tableStepper(AccelStepper::DRIVER, TABLE_STEP_PIN, TABLE_DIR_PIN);
AccelStepper seedStepper(AccelStepper::DRIVER, SEED_STEP_PIN, SEED_DIR_PIN);

enum SystemState { SYS_IDLE, SYS_RUNNING, SYS_STOPPED };
SystemState sysState = SYS_IDLE;

unsigned long lastIndexMillis = 0;
uint32_t indexCounter = 0;
const unsigned long ejectPulseMs = 200UL; // solenoid pulse duration

/* ====== Serial UI state (reused pattern) ====== */
enum UIState { STATE_IDLE_UI, STATE_INPUT_UI };
UIState uiState = STATE_IDLE_UI;
char currentParam = 0;
String inputBuffer = "";
unsigned long lastKeyMillis = 0;
const unsigned long inputTimeoutMs = 30000UL;
const unsigned long potUpdateIntervalMs = 200UL;
unsigned long lastPotPrint = 0;

/* ====== Button debounce ====== */
unsigned long lastStartDebounce = 0;
unsigned long lastStopDebounce = 0;
const unsigned long btnDebounceMs = 60UL;


void homeTable(){
  Serial.println("Start Homing.");

  tableStepper.setAcceleration(200);
  tableStepper.setMaxSpeed(400);
  tableStepper.moveTo(-100000);
  tableStepper.disableOutputs();

  bool tableHomed = false;

  while (!tableHomed){
    if(!tableHomed){
      if (tableHomeTriggered()){
        tableStepper.stop();
        tableHomed = true;
        Serial.println("Table Homed.");
        tableStepper.enableOutputs();
      }
      else{
        tableStepper.run();
      }
    }
  }
}
/* ====== UTILITIES ====== */
float rpmToStepsPerSec(int rpm) {
  // steps/sec = rpm * stepsPerRev / 60
  return ((float)rpm * (float)stepsPerRev) / 60.0f;
}

uint8_t percentToPwm(int pct) {
  if (pct <= 0) return 0;
  if (pct >= 100) return 255;
  return (uint8_t) map(pct, 0, 100, 0, 255);
}

/* ====== MACHINE ACTIONS ====== */
void startMachine() {
  if (sysState == SYS_RUNNING) return;
  Serial.println(F("Machine START"));

  // Configure continuous seed speed
  float seedStepsPerSec = rpmToStepsPerSec(seedRPM);
  seedStepper.setMaxSpeed(seedStepsPerSec * 1.5f);
  seedStepper.setSpeed(seedStepsPerSec); // runSpeed uses steps/sec set here

  // Configure table move max speed and acceleration
  float tableStepsPerSec = rpmToStepsPerSec(tableMoveRPM);
  tableStepper.setMaxSpeed(tableStepsPerSec * 2.0f);
  tableStepper.setAcceleration(2000); // tune as needed

  // Ensure outputs enabled
  seedStepper.disableOutputs();
  tableStepper.disableOutputs();

  // DC motor on
  analogWrite(DC_PWM_PIN, percentToPwm(dcPercent));
  digitalWrite(RelayMotor, LOW);

  lastIndexMillis = millis();
  sysState = SYS_RUNNING;
}

void stopMachine() {
  if (sysState != SYS_RUNNING) return;
  Serial.println(F("Machine STOP"));

  // Stop steppers and DC
  seedStepper.setSpeed(0);
  seedStepper.enableOutputs(); // disable driver outputs
  tableStepper.enableOutputs();
  analogWrite(DC_PWM_PIN, 0);
  digitalWrite(RelayMotor, HIGH);

  sysState = SYS_STOPPED;
  if ((indexCounter % 2) == 0){
    indexCounter = 0;
  }
  else {
    indexCounter = 1;
  }
}

void issueTableIndex() {
  // pulse eject every 2 indexes (every 90°)
  if ((indexCounter % 2) == 0 && indexCounter != 0) {
    digitalWrite(EJECT_PIN, LOW);
    delay(ejectPulseMs); // brief blocking pulse (short)
    digitalWrite(EJECT_PIN, HIGH);
    Serial.println(F("Eject pulse"));
    delay(500);
  }

  indexCounter++;
  Serial.print(F("Index #: "));
  Serial.println(indexCounter);

  // relative positive move (one 45° index)
  tableStepper.move((long)TABLE_STEPS_PER_INDEX);
}

/* ====== BUTTONS ====== */
bool readStartButtonPressed() {
  bool pressed = (digitalRead(START_PIN) == LOW);
  if (pressed && (millis() - lastStartDebounce > btnDebounceMs)) {
    lastStartDebounce = millis();
    return true;
  }
  return false;
}

bool readStopButtonPressed() {
  bool pressed = (digitalRead(STOP_PIN) == LOW);
  if (pressed && (millis() - lastStopDebounce > btnDebounceMs)) {
    lastStopDebounce = millis();
    return true;
  }
  return false;
}

/* ====== SERIAL UI ====== */
void printHeader() {
  Serial.println(F("\n--- Shining Machine (Serial UI) ---"));
  Serial.print(F("A: Interval (s) = ")); Serial.println(intervalSeconds);
  Serial.print(F("B: Seed RPM     = ")); Serial.println(seedRPM);
  Serial.print(F("C: Table moveRPM= ")); Serial.println(tableMoveRPM);
  Serial.print(F("D: DC %         = ")); Serial.println(dcPercent);
  Serial.println(F("Commands: A/B/C/D to edit, digits, '*' backspace/cancel, '#' save"));
  Serial.println(F("S = Start, X = Stop, ? = show menu"));
  Serial.println();
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') continue;
    if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
    lastKeyMillis = millis();

    if (uiState == STATE_IDLE_UI) {
      if (c == 'A' || c == 'B' || c == 'C' || c == 'D') {
        currentParam = c;
        inputBuffer = "";
        uiState = STATE_INPUT_UI;
        Serial.print(F("Editing param ")); Serial.println(currentParam);
        if (currentParam == 'D') {
          int pot = analogRead(POT_PIN);
          int pct = map(pot, 0, 1023, 0, 100);
          Serial.print(F("(live pot) ")); Serial.print(pct); Serial.println(F(" %"));
        }
      } else if (c == 'S') {
        startMachine();
      } else if (c == 'X') {
        stopMachine();
      } else if (c == '?') {
        printHeader();
      }
    } else { // STATE_INPUT_UI
      if (currentParam == 'D') {
        // D-mode: accept digits, '*' cancel/backspace, '#' save (if buffer empty '#' will use pot)
        if (c >= '0' && c <= '9') {
          if (inputBuffer.length() < 3) inputBuffer += c;
          Serial.print(c);
        } else if (c == '*') {
          if (inputBuffer.length() > 0) inputBuffer.remove(inputBuffer.length() - 1);
          else { uiState = STATE_IDLE_UI; Serial.println(F("\nEdit cancelled")); printHeader(); }
        } else if (c == '#') {
          int savedPct;
          if (inputBuffer.length() == 0) {
            int pot = analogRead(POT_PIN);
            savedPct = map(pot, 0, 1023, 0, 100);
          } else savedPct = inputBuffer.toInt();
          if (savedPct < 0) savedPct = 0;
          if (savedPct > 100) savedPct = 100;
          dcPercent = savedPct;
          Serial.print(F("\nSaved D = ")); Serial.print(dcPercent); Serial.println(F(" %"));
          if (sysState == SYS_RUNNING) analogWrite(DC_PWM_PIN, percentToPwm(dcPercent));
          uiState = STATE_IDLE_UI;
          printHeader();
        }
      } else {
        // A/B/C numeric input modes
        if (c >= '0' && c <= '9') {
          int maxDigits = (currentParam == 'A') ? 6 : 4;
          if (inputBuffer.length() < maxDigits) inputBuffer += c;
          Serial.print(c);
        } else if (c == '*') {
          if (inputBuffer.length() > 0) inputBuffer.remove(inputBuffer.length() - 1);
          else { uiState = STATE_IDLE_UI; Serial.println(F("\nEdit cancelled")); printHeader(); }
        } else if (c == '#') {
          if (inputBuffer.length() == 0) {
            uiState = STATE_IDLE_UI; Serial.println(F("\nEdit cancelled")); printHeader();
          } else {
            long v = inputBuffer.toInt();
            if (currentParam == 'A') {
              if (v < 1) v = 1;
              intervalSeconds = (unsigned long)v;
            } else if (currentParam == 'B') {
              if (v < 0) v = 0; if (v > 1500) v = 1500;
              seedRPM = (int)v;
              // if running, update live speed
              if (sysState == SYS_RUNNING) seedStepper.setSpeed(rpmToStepsPerSec(seedRPM));
            } else if (currentParam == 'C') {
              if (v < 0) v = 0; if (v > 1500) v = 1500;
              tableMoveRPM = (int)v;
              if (sysState == SYS_RUNNING) {
                float tableStepsPerSec = rpmToStepsPerSec(tableMoveRPM);
                tableStepper.setMaxSpeed(tableStepsPerSec * 2.0f);
              }
            }
            Serial.print(F("\nSaved ")); Serial.print(currentParam); Serial.println();
            uiState = STATE_IDLE_UI;
            printHeader();
          }
        }
      }
    }
  }
}

/* ====== SETUP ====== */
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  delay(20);

    // Pins
  pinMode(DC_PWM_PIN, OUTPUT);
  analogWrite(DC_PWM_PIN, 0);

  pinMode(EJECT_PIN, OUTPUT);
  digitalWrite(EJECT_PIN, HIGH);

  pinMode(RelayMotor, OUTPUT);
  digitalWrite(RelayMotor, HIGH);

  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(STOP_PIN, INPUT_PULLUP);

  pinMode(TABLE_STEP_PIN, OUTPUT);
  pinMode(TABLE_DIR_PIN, OUTPUT);
  pinMode(TABLE_ENA_PIN, OUTPUT);
  pinMode(SEED_STEP_PIN, OUTPUT);
  pinMode(SEED_DIR_PIN, OUTPUT);
  pinMode(SEED_ENA_PIN, OUTPUT);

  digitalWrite(TABLE_ENA_PIN, LOW);
  digitalWrite(SEED_ENA_PIN, LOW);

  // AccelStepper configuration - follow Straight-Cut style
  tableStepper.setEnablePin(TABLE_ENA_PIN);
  seedStepper.setEnablePin(SEED_ENA_PIN);

  // Inversion flags: adjust if your drivers require inversion
  // Following the Straight-Cut example style: (step inverted? dir inverted? enable inverted?)
  tableStepper.setPinsInverted(false, false, true); // (stepInv, dirInv, enableInv)
  seedStepper.setPinsInverted(false, false, true);

  // Initial acceleration / max speed (safe defaults)
  tableStepper.setAcceleration(2000);
  seedStepper.setAcceleration(2000);
  tableStepper.setMaxSpeed(10000);
  seedStepper.setMaxSpeed(10000);

  // Disable outputs initially (will enable on start)
  tableStepper.enableOutputs();
  seedStepper.enableOutputs();

  pinMode(POT_PIN, INPUT);

  pinMode(TABLE_HOME_PIN, INPUT_PULLUP);

  homeTable();

  printHeader();
}

/* ====== LOOP ====== */
void loop() {
  // Poll hardware Start/Stop buttons
  if (readStartButtonPressed()) startMachine();
  if (readStopButtonPressed()) stopMachine();

  // Handle serial UI input
  handleSerialInput();

  // If in D edit mode, show live pot periodically
  if (uiState == STATE_INPUT_UI && currentParam == 'D') {
    unsigned long now = millis();
    if (now - lastPotPrint >= potUpdateIntervalMs) {
      lastPotPrint = now;
      int pot = analogRead(POT_PIN);
      int pct = map(pot, 0, 1023, 0, 100);
      Serial.print(F("(live pot) ")); Serial.print(pct); Serial.println(F(" %"));
    }
  }

  // Main running behavior
  if (sysState == SYS_RUNNING) {
    // Ensure steppers enabled
    seedStepper.disableOutputs();
    tableStepper.disableOutputs();

    // Continuous seed rotation using runSpeed()
    
    seedStepper.runSpeed();

    // If table is moving to reach its relative target, let it run
    if (tableStepper.distanceToGo() != 0) {
      tableStepper.run();
    } else {
      // Not moving: check index timer
      if ((millis() - lastIndexMillis) >= (unsigned long)intervalSeconds * 1000UL) {
        lastIndexMillis = millis();
        issueTableIndex();
      }
    }
  } else {
    // not running: ensure outputs off
    seedStepper.enableOutputs();
    tableStepper.enableOutputs();
    analogWrite(DC_PWM_PIN, 0);
  }

}
