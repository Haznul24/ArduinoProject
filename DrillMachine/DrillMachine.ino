// Arduino code for Drill Machine
// Three Stepper driver, two push button, four proximity sensor, three relay

#include <AccelStepper.h>

// Stepper Motor
#define STEP_X_PIN 13
#define DIR_X_PIN  12
#define EN_X_PIN   11

#define STEP_Y_PIN 10
#define DIR_Y_PIN  9
#define EN_Y_PIN   8

#define STEP_Z_PIN 7
#define DIR_Z_PIN  6
#define EN_Z_PIN   5

AccelStepper stepX(AccelStepper::DRIVER, STEP_X_PIN, DIR_X_PIN);
AccelStepper stepY(AccelStepper::DRIVER, STEP_Y_PIN, DIR_Y_PIN);
AccelStepper stepZ(AccelStepper::DRIVER, STEP_Z_PIN, DIR_Z_PIN);

const float MAX_SPEED   = 4000;
const float ACCEL       = 500;
const long homePos      = 80;

// Proximity Sensor
#define SENSOR_X_PIN         31
#define SENSOR_Y_PIN         33
#define SENSOR_Z_MIN_PIN     35
#define SENSOR_Z_MAX_PIN     37
const bool proxActiveLow = true;

// Relay
#define DRILL_RELAY          43
#define GRN_LED_RELAY        45
#define RED_LED_RELAY        47
const bool relayActiveLow = true;

// Push Button Pin
#define ESTOP_PIN            23     // Emergency Stop
#define PROCESS_BTN_PIN      25     // Start process drilling

bool lastProcessBtnState = HIGH;   // INPUT_PULLUP default
unsigned long lastProcessBtnTime = 0;
const unsigned long PROCESS_BTN_DEBOUNCE_MS = 50;

// State tracking
bool homed = false; 
volatile bool emergencyActive = false;
bool lastEstopPinState = LOW;

// Utility helpers
inline bool proxTriggered(int pin) {
  return proxActiveLow ? digitalRead(pin) == LOW
                       : digitalRead(pin) == HIGH;
}

inline uint8_t relayOn()  { return relayActiveLow ? LOW  : HIGH; }
inline uint8_t relayOff() { return relayActiveLow ? HIGH : LOW; }

void drillMotorOn()  { digitalWrite(DRILL_RELAY, relayOn()); }
void drillMotorOff() { digitalWrite(DRILL_RELAY, relayOff()); }

void greenLedOn()  { digitalWrite(GRN_LED_RELAY, relayOn()); }
void greenLedOff() { digitalWrite(GRN_LED_RELAY, relayOff()); }

void redLedOn()    { digitalWrite(RED_LED_RELAY, relayOn()); }
void redLedOff()   { digitalWrite(RED_LED_RELAY, relayOff()); }

// Process Stage
enum ProcessStage {
  proIdle,        // Idle stage
  proAlign,       // Align seed to drill position
  proDrill,       // Drilling process
  proReturn       // Return seed to original position
};

ProcessStage processStage = proIdle;

void emergencyStop() {
  emergencyActive = true;
  homed = false;

  stepX.setCurrentPosition(stepX.currentPosition());
  stepY.setCurrentPosition(stepY.currentPosition());
  stepZ.setCurrentPosition(stepZ.currentPosition());

  // Disable drivers (optional but recommended)
  stepX.disableOutputs();
  stepY.disableOutputs();
  stepZ.disableOutputs();

  // Turn OFF drill immediately
  drillMotorOff();

  processStage = proIdle;

  Serial.println(F("EMERGENCY_STOP"));
}

void emergencyReset() {
  emergencyActive = false;

  stepX.enableOutputs();
  stepY.enableOutputs();
  stepZ.enableOutputs();

  Serial.println(F("EMERGENCY_RESET"));
}

void checkEmergencyButton() {
  bool current = digitalRead(ESTOP_PIN);

  // NORMAL → EMERGENCY (pressed → released)
  if (lastEstopPinState == LOW && current == HIGH) {
    emergencyStop();
  }

  // EMERGENCY → NORMAL (released → pressed)
  if (lastEstopPinState == HIGH && current == LOW) {
    emergencyReset();
  }

  lastEstopPinState = current;
}

void checkProcessButton() {
  bool current = digitalRead(PROCESS_BTN_PIN);
  unsigned long now = millis();

  // Detect HIGH -> LOW (button pressed)
  if (lastProcessBtnState == HIGH && current == LOW) {
    if (now - lastProcessBtnTime > PROCESS_BTN_DEBOUNCE_MS) {

      // Optional safety checks
      if (!emergencyActive && homed && processStage == proIdle) {
        Serial.println(F("PB_ARDUINO_PROCESS"));
      }

      lastProcessBtnTime = now;
    }
  }

  lastProcessBtnState = current;
}

void reportStatus() {
  Serial.print(F("POS_X:"));Serial.println(stepX.currentPosition());
  Serial.print(F("POS_Y:"));Serial.println(stepY.currentPosition());
  Serial.print(F("SENSOR_X:")); Serial.println(proxTriggered(SENSOR_X_PIN));
  Serial.print(F("SENSOR_Y:")); Serial.println(proxTriggered(SENSOR_Y_PIN));
  Serial.print(F("SENSOR_Z_MIN:")); Serial.println(proxTriggered(SENSOR_Z_MIN_PIN));
  Serial.print(F("SENSOR_Z_MAX:")); Serial.println(proxTriggered(SENSOR_Z_MAX_PIN));

  Serial.print(F("STATUS:"));
}

void homeAxes() {
  homed = false;
  Serial.println(F("Starting homing..."));

  // Homing speed & accel
  stepX.setMaxSpeed(800);
  stepY.setMaxSpeed(800);
  stepZ.setMaxSpeed(800);

  stepX.setAcceleration(500);
  stepY.setAcceleration(500);
  stepZ.setAcceleration(500);

  // Move all axes toward negative direction
  stepX.moveTo(-100000);
  stepY.moveTo(-100000);
  stepZ.moveTo(-100000);

  bool xHomed = false;
  bool yHomed = false;
  bool zHomed = false;

  while (!xHomed || !yHomed || !zHomed) {

    // ---- X axis ----
    if (!xHomed) {
      if (proxTriggered(SENSOR_X_PIN)) {
        stepX.stop();
        stepX.setCurrentPosition(0);
        xHomed = true;
        Serial.println("X homed");
      } else {
        stepX.run();
      }
    }

    // ---- Y axis ----
    if (!yHomed) {
      if (proxTriggered(SENSOR_Y_PIN)) {
        stepY.stop();
        stepY.setCurrentPosition(0);
        yHomed = true;
        Serial.println("Y homed");
      } else {
        stepY.run();
      }
    }

    // ---- Z axis ----
    if (!zHomed) {
      if (proxTriggered(SENSOR_Z_MIN_PIN)) {
        stepZ.stop();
        stepZ.setCurrentPosition(0);
        zHomed = true;
        Serial.println("Z homed");
      } else {
        stepZ.run();
      }
    }
  }

  delay(200);

  // Move all axes to safe position after homing
  stepX.moveTo(homePos);
  stepY.moveTo(homePos);
  stepZ.moveTo(homePos);

  while (stepX.distanceToGo() ||
        stepY.distanceToGo() ||
        stepZ.distanceToGo()) {
    stepX.run();
    stepY.run();
    stepZ.run();
  }
  stepX.setCurrentPosition(0);
  stepY.setCurrentPosition(0);
  stepZ.setCurrentPosition(0);

  homed = true;
  Serial.println(F("Homing complete. Ready."));
}

void drillSeed() {
  Serial.println(F("Start Drilling..."));
  bool zDrill = true;
  stepZ.setMaxSpeed(800);
  stepZ.setAcceleration(400);

  drillMotorOn();
  delay(200);

  stepZ.moveTo(100000);
  
  while (zDrill){
    if (proxTriggered(SENSOR_Z_MAX_PIN)) {
        stepZ.setCurrentPosition(stepZ.currentPosition());
        zDrill = false;
      } else {
        stepZ.run();
      }
  }

  stepZ.moveTo(0);
  while(stepZ.distanceToGo() != 0){
    stepZ.run();
  }

  drillMotorOff();
  Serial.println(F("Drilling complete, Z returned."));
}

void handleCommand(const String &cmd) {
  if (cmd.length() == 0) return;

  int a = cmd.indexOf(':');
  int b = cmd.indexOf(':', a + 1);

  String opcode = (a == -1) ? cmd : cmd.substring(0, a);
  long p1 = (a != -1) ? cmd.substring(a + 1, b).toInt() : 0;    // Position
  float p2 = (b != -1) ? cmd.substring(b + 1).toFloat() : 0;    // Speed

  if (!homed && (opcode != "HOME_ALL")){
    Serial.println(F("ERROR:NOT_HOMED"));
    return;
  }

  if (opcode == "MOVE_X") {
    stepX.setMaxSpeed(p2);
    stepX.moveTo(stepX.currentPosition() + p1);
    Serial.println(F("OK:MOVE_X"));
  }
  else if (opcode == "MOVE_Y") {
    stepY.setMaxSpeed(p2);
    stepY.moveTo(stepY.currentPosition() + p1);
    Serial.println(F("OK:MOVE_Y"));
  }

  else if (opcode == "PROC_MOVE_X") {
    stepX.setMaxSpeed(p2);
    stepX.moveTo(p1);
    Serial.println(F("OK:PROC_MOVE_X"));
    processStage = proAlign;
  }
  else if (opcode == "PROC_MOVE_Y") {
    stepY.setMaxSpeed(p2);
    stepY.moveTo(p1);
    Serial.println(F("OK:PROC_MOVE_Y"));
    processStage = proAlign;
  }
  else if (opcode == "HOME_ALL") {
    homeAxes();
    Serial.println(F("OK:HOME_ALL"));
  }
  else if (opcode == "STOP") {
    emergencyStop();
    Serial.println(F("OK:STOP"));
  }
  else if (opcode == "GET_STATUS") {
    reportStatus();
    Serial.println(F("OK:STATUS"));
  }
  else {
    Serial.println(F("ERROR:UNKNOWN_CMD"));
  }
}

void setup() {
  Serial.begin(115200);

  // Configure pins
  stepX.setEnablePin(EN_X_PIN);   //not inverted
  stepY.setEnablePin(EN_Y_PIN);
  stepZ.setEnablePin(EN_Z_PIN);

  stepX.setMaxSpeed(MAX_SPEED);  
  stepY.setMaxSpeed(MAX_SPEED); 
  stepZ.setMaxSpeed(MAX_SPEED);
  stepX.setAcceleration(ACCEL);
  stepY.setAcceleration(ACCEL);
  stepZ.setAcceleration(ACCEL);

  pinMode(SENSOR_X_PIN, INPUT_PULLUP);
  pinMode(SENSOR_Y_PIN, INPUT_PULLUP);
  pinMode(SENSOR_Z_MIN_PIN, INPUT_PULLUP);
  pinMode(SENSOR_Z_MAX_PIN, INPUT_PULLUP);

  pinMode(DRILL_RELAY, OUTPUT);
  pinMode(GRN_LED_RELAY, OUTPUT);
  pinMode(RED_LED_RELAY, OUTPUT);
  drillMotorOff();
  greenLedOff();
  redLedOff();

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  pinMode(PROCESS_BTN_PIN, INPUT_PULLUP);
}

void loop() {
  checkEmergencyButton();
  checkProcessButton();

  if (emergencyActive) {
    greenLedOff();
    redLedOn();
    return;
  }

  stepX.run();
  stepY.run();
  stepZ.run();

  switch (processStage) {
    case proAlign:
      redLedOn();
      greenLedOff();
      if(stepX.distanceToGo() == 0 &&
         stepY.distanceToGo() == 0) {
          processStage = proDrill;
          Serial.println(F("Seed align with drill"));
         }
      break;

    case proDrill:
      drillSeed();
      processStage = proReturn;
      break;

    case proReturn:
      stepX.moveTo(0);
      stepY.moveTo(0);

      if(stepX.distanceToGo() == 0 &&
         stepY.distanceToGo() == 0) {
          processStage = proIdle;
          Serial.println(F("OK:PROCESS_DONE"));
         }
      break;

      case proIdle:
        greenLedOn();
        redLedOff();
        break;
      default:
        greenLedOff();
        redLedOff();
        break;
  }
  
  static String buffer = "";
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      handleCommand(buffer);
      buffer = "";
    } else {
      buffer += c;
    }
  }
}

