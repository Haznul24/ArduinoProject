#include <Keypad.h>

#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x20, 16, 2);


// Keypad Setup
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {9, 8, 7, 6}; // R1..R4
byte colPins[COLS] = {5, 4, 3, 2}; // C1..C4

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// ----------------- Parameters (defaults) -----------------
unsigned long intervalSeconds = 180; // default: 180 s (3 minutes)
int stepperRPM = 200;           // default: 200 RPM
int servoRPM = 300;             // default: 300 RPM
int dcPercent = 50;                // default: 50%

// ----------------- Hardware pins -----------------
const uint8_t DC_POT_PIN = A0;      // potentiometer wiper
const uint8_t DC_PWM_PIN = 10;      // PWM to MOSFET/transistor (must be PWM-capable)

// ----------------- UI state -----------------
enum UIState { STATE_IDLE, STATE_INPUT };
UIState uiState = STATE_IDLE;

char currentParam = 0;    // 'A','B','C','D' when editing
String inputBuffer = "";  // digits being entered
unsigned long lastKeyMillis = 0;
const unsigned long inputTimeoutMs = 30000UL; // cancel editing after 30s of inactivity

// For D-mode update throttling
unsigned long lastPotUpdate = 0;
const unsigned long potUpdateIntervalMs = 120; // update display & pwm every 120

// Keypad Read Function
void readKeypad(){
  char key = keypad.getKey();
  if (key) {
    Serial.println(key);
  }
}
// ----------------- Helper functions -----------------
void showWelcome() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Shining Machine");
  lcd.setCursor(0,1);
  lcd.print("Press A to start");
}

void showMenu() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("A:");
  lcd.print(intervalSeconds);
  lcd.print("s ");
  lcd.print("B:");
  lcd.print(stepperRPM);
  lcd.print(" RPM");
  lcd.setCursor(0,1);
  lcd.print("C:");
  lcd.print(servoRPM);
  lcd.print("RPM");
  lcd.print(" D:");
  lcd.print(dcPercent);
  lcd.print("%");
}

void showParamPrompt(char param) {
  lcd.clear();
  switch (param) {
    case 'A':
      lcd.setCursor(0,0); lcd.print("A: Interval (s)");
      lcd.setCursor(0,1); lcd.print(inputBuffer);
      break;
    case 'B':
      lcd.setCursor(0,0); lcd.print("B: Stepper RPM");
      lcd.setCursor(0,1); lcd.print(inputBuffer);
      break;
    case 'C':
      lcd.setCursor(0,0); lcd.print("C: Servo RPM");
      lcd.setCursor(0,1); lcd.print(inputBuffer);
      break;
    case 'D':
      lcd.setCursor(0,0); lcd.print("D: DC Motor %");
      lcd.setCursor(0,1); lcd.print(inputBuffer);
      break;
    default:
      showWelcome();
  }
}

void showSaved(char param) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Saved ");
  lcd.print(param);
  lcd.setCursor(0,1);
  switch (param) {
    case 'A': lcd.print(intervalSeconds); lcd.print(" s"); break;
    case 'B': lcd.print(stepperRPM); lcd.print(" RPM"); break;
    case 'C': lcd.print(servoRPM); lcd.print(" RPM"); break;
    case 'D': lcd.print(dcPercent); lcd.print(" %"); break;
  }
}
void cancelInput() {
  inputBuffer = "";
  currentParam = 0;
  uiState = STATE_IDLE;
  showMenu();
}

// ----------------- Setup -----------------
void setup() {
  pinMode(DC_PWM_PIN, OUTPUT);
  analogWrite(DC_PWM_PIN, 0); // ensure off
  // Initialize LCD
  lcd.init();        // initialize the lcd (for I2C)
  lcd.backlight();   // turn on backlight
  showWelcome();

  // Small startup delay and then show current menu
  delay(1000);
  showMenu();

  // (Optional) initialize serial for debugging
  Serial.begin(115200);
  Serial.println("Parameter UI started");
}

// ----------------- Utility -----------------
// Map 0..100% to PWM 0..255
uint8_t percentToPwm(int pct) {
  if (pct <= 0) return 0;
  if (pct >= 100) return 255;
  return (uint8_t) map(pct, 0, 100, 0, 255);
}

// Read pot and return 0..100 rounded
int readPotPercent() {
  int a = analogRead(DC_POT_PIN); // 0..1023
  int pct = map(a, 0, 1023, 0, 100);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

// Update live DC motor from pot (call periodically)
void updateDcLive() {
  int pct = readPotPercent();
  uint8_t pwm = percentToPwm(pct);
  analogWrite(DC_PWM_PIN, pwm);
  Serial.println(pwm);

  // show live value on LCD (line 2 right side)
  // we avoid clearing whole LCD too often; write formatted value
  lcd.setCursor(0,1);
  lcd.print("DC:");
  if (pct < 10) lcd.print(" ");
  if (pct < 100) lcd.print(" ");
  lcd.print(pct);
  lcd.print(" %   "); // trailing spaces to clear old chars
}


// ----------------- Main loop -----------------
void loop() {
  char key = keypad.getKey();

  // If in D-mode input, update the pot-driven PWM regularly even if no key is pressed
  if (uiState == STATE_INPUT && currentParam == 'D') {
    unsigned long now = millis();
    if (now - lastPotUpdate >= potUpdateIntervalMs) {
      lastPotUpdate = now;
      updateDcLive();
    }
  }

  if (key) {
    lastKeyMillis = millis(); // reset inactivity timer

    // If not currently entering a parameter, A..D start editing
    if (uiState == STATE_IDLE) {
      if (key == 'A' || key == 'B' || key == 'C' || key == 'D') {
        currentParam = key;
        inputBuffer = ""; // start fresh for A/B/C
        uiState = STATE_INPUT;
        showParamPrompt(currentParam);

        // If entering D-mode, immediately start live update so motor runs
        if (currentParam == 'D') {
          // Force an immediate pot update and display
          updateDcLive();
        }
      } else {
        // other keys while idle: ignore
      }
    } 
    else if (uiState == STATE_INPUT) {
      // Special handling for D-mode: use pot; only accept '#' to save or '*' to cancel
      if (currentParam == 'D') {
        if (key == '#') {
          // Save current pot value
          int pct = readPotPercent();
          dcPercent = pct;
          // stop motor after saving
          analogWrite(DC_PWM_PIN, 0);
          showSaved('D');
          delay(800);
          // return to menu
          currentParam = 0;
          uiState = STATE_IDLE;
          inputBuffer = "";
          showMenu();
        } else if (key == '*') {
          // cancel: turn off motor and return to menu
          analogWrite(DC_PWM_PIN, 0);
          cancelInput();
        } else {
          // ignore other keys in D-mode
        }
      } 
      else {
        // Normal numeric input mode for A/B/C
        if (key >= '0' && key <= '9') {
          // Limit length (A: up to 6 digits; B/C RPM up to 4 digits)
          int maxDigits = (currentParam == 'A') ? 6 : 4;
          if (inputBuffer.length() < maxDigits) {
            inputBuffer += key;
          }
          showParamPrompt(currentParam);
        } 
        else if (key == '*') {
          if (inputBuffer.length() > 0) {
            inputBuffer.remove(inputBuffer.length() - 1);
            showParamPrompt(currentParam);
          } else {
            // cancel editing
            cancelInput();
          }
        } 
        else if (key == '#') {
          if (inputBuffer.length() == 0) {
            cancelInput();
          } else {
            long v = inputBuffer.toInt();
            // Clamp values
            if (currentParam == 'A') {
              if (v < 1) v = 1;
              intervalSeconds = (unsigned long)v;
            } else if (currentParam == 'B') {
              if (v < 0) v = 0;
              if (v > 1500) v = 1500;
              stepperRPM = (int)v;
            } else if (currentParam == 'C') {
              if (v < 0) v = 0;
              if (v > 1500) v = 1500;
              servoRPM = (int)v;
            }
            showSaved(currentParam);
            delay(800);
            currentParam = 0;
            inputBuffer = "";
            uiState = STATE_IDLE;
            showMenu();
          }
        }
      } // end non-D
    } // end STATE_INPUT
  } // end if key

  // Handle input timeout (abort editing if idle for too long)
  if (uiState == STATE_INPUT) {
    if (millis() - lastKeyMillis > inputTimeoutMs) {
      // If we were in D-mode, stop motor
      if (currentParam == 'D') analogWrite(DC_PWM_PIN, 0);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Input timeout");
      delay(700);
      cancelInput();
    }
  }

  // short non-blocking delay
  delay(6);
}