/**
 * ============================================================================
 * PRODUCTION-GRADE ARDUINO NANO PID LINE FOLLOWER ROBOT (LFR)
 * ============================================================================
 * Architecture : Arduino Nano (ATmega328P, 16MHz, 5V Logic)
 * Motor Driver : TB6612FNG Dual H-Bridge (STBY hardware-tied to 5V)
 * Sensor Array : 8-Channel TCRT5000 / RSL 08 IR Reflectance Array (A0 - A7)
 * Control Loop : High-Speed Proportional-Derivative (PD) with Weighted Centroid
 * 
 * Pin Connections (as defined in connections.txt):
 * ----------------------------------------------------------------------------
 * Left Motor  : BIN1 -> D2 (Dir 1), BIN2 -> D4 (Dir 2), PWMB -> D5 (PWM)
 * Right Motor : AIN1 -> D7 (Dir 1), AIN2 -> D8 (Dir 2), PWMA -> D6 (PWM)
 * Standby     : TB6612FNG STBY tied directly to 5V (Hardware Rail)
 * Button 1    : D11 (INPUT_PULLUP, Active-LOW) -> Start / Stop PID Toggle
 * Button 2    : D12 (INPUT_PULLUP, Active-LOW) -> Calibration Trigger & Step Advance
 * Status LED  : D13 (Built-in Nano LED)
 * Sensors     : A0 to A7 -> 8-Channel Analog IR Reflectance Inputs
 * ----------------------------------------------------------------------------
 * 
 * Calibration Sequence:
 * 1. Hold Button 2 (>1.5s) from IDLE -> 3 LED Blinks -> State: CALIB_WHITE
 * 2. Place on White surface -> Short press Button 2 -> Measures White -> 2 LED Blinks -> State: CALIB_BLACK
 * 3. Place on Black line/sheet -> Short press Button 2 -> Measures Black -> 2 LED Blinks -> State: CALIB_CENTER
 * 4. Place Centered over line -> Short press Button 2 -> Measures Center -> 2 LED Blinks -> State: MOTOR_TEST
 * 5. Motor Test: Forward 500ms -> Backward 500ms -> Stop -> Auto-save EEPROM (Magic: 0x5A) -> 4 Fast Blinks -> IDLE
 * ============================================================================
 */

#include <Arduino.h>
#include <EEPROM.h>

// ---------------------------------------------------------------------------
// HARDWARE PIN ASSIGNMENTS (Strictly matching connections.txt)
// ---------------------------------------------------------------------------
// Left Motor (TB6612FNG Channel B)
const uint8_t PIN_BIN1 = 2;   // Left Motor Direction Pin 1
const uint8_t PIN_BIN2 = 4;   // Left Motor Direction Pin 2
const uint8_t PIN_PWMB = 5;   // Left Motor Speed PWM (Timer 0, ~976 Hz)

// Right Motor (TB6612FNG Channel A)
const uint8_t PIN_PWMA = 6;   // Right Motor Speed PWM (Timer 0, ~976 Hz)
const uint8_t PIN_AIN1 = 7;   // Right Motor Direction Pin 1
const uint8_t PIN_AIN2 = 8;   // Right Motor Direction Pin 2

// User Interface Push Buttons & Indicators
const uint8_t PIN_BTN_START  = 11; // Button 1: PID Run / Stop toggle (Active-LOW to GND)
const uint8_t PIN_BTN_CALIB  = 12; // Button 2: Calibration trigger & advance (Active-LOW to GND)
const uint8_t PIN_LED_STATUS = 13; // Built-in Status LED

// 8-Channel TCRT5000 / RSL 08 IR Sensor Array Analog Inputs
// Physical Alignment: A7 is on physical LEFT, A0 is on physical RIGHT.
// Array index 0 = A7 (Far Left, Coordinate 0) ... Index 7 = A0 (Far Right, Coordinate 7000)
const uint8_t SENSOR_PINS[8] = { A7, A6, A5, A4, A3, A2, A1, A0 };

// ---------------------------------------------------------------------------
// EEPROM CALIBRATION STORAGE STRUCTURE
// ---------------------------------------------------------------------------
#define EEPROM_MAGIC_BYTE 0x5A
#define EEPROM_START_ADDR 0

struct CalibrationData {
  uint8_t  magic;           // Offset 0:  Magic identifier byte (0x5A)
  uint16_t sensorMin[8];    // Offset 1:  White baseline ADC readings (16 bytes, offsets 1-16)
  uint16_t sensorMax[8];    // Offset 17: Black baseline ADC readings (16 bytes, offsets 17-32)
  float    centerSetpoint;  // Offset 33: Geometric/calibrated center setpoint (4 bytes, offsets 33-36)
}; // Total size: 37 bytes

CalibrationData calData;

// ---------------------------------------------------------------------------
// ROBOT OPERATIONAL STATE MACHINE
// ---------------------------------------------------------------------------
enum RobotState {
  IDLE,          // Motors off, waiting for user command (Button 1: Run, Button 2 hold: Calib)
  RUNNING_PID,   // Closed-loop PID line tracking active
  CALIB_WHITE,   // Calibration Step 1: Sampling white background baseline
  CALIB_BLACK,   // Calibration Step 2: Sampling black track/line baseline
  CALIB_CENTER,  // Calibration Step 3: Sampling centered position setpoint
  MOTOR_TEST     // Calibration Step 4: Motor directional verification & EEPROM persistence
};

RobotState robotState = IDLE;

// ---------------------------------------------------------------------------
// PID CONTROLLER PARAMETERS & GAINS
// ---------------------------------------------------------------------------
// Kp: Proportional gain
// Controls immediate steering correction proportional to tracking error.
// Higher values increase agility on tight curves, but excessive values cause wobbling.
float Kp = 0.080f;

// Ki: Integral gain
// CRITICAL NOTE: Set to 0.0 by default. In high-speed line followers, integral accumulation
// creates phase lag and severe integral windup during transitions (e.g., S-curves, chicanes),
// leading to violent oscillations and track departure. If steady-state offset is observed along
// long straightaways, Ki can be tuned to a very small non-zero value (e.g., 0.0001 - 0.001)
// paired with anti-windup clamping.
const float Ki = 0.0f;

// Kd: Derivative gain
// Dampens steering oscillations by counteracting the rate of error change.
// Crucial for high-speed stability and preventing overshoot when re-centering.
float Kd = 1.250f;

// Speed Configuration for 2S LiPo (7.4V) & 600 RPM N20 Motors (PWM range: 0 - 255)
int baseSpeed     = 140;  // Cruising speed on straight track sections (optimized for 2S 600 RPM)
int maxSpeed      = 220;  // Maximum PWM ceiling to prevent motor driver saturation
int minSpeed      = -80;  // Negative allows reverse rotation for tight pivot turns
int recoverySpeed = 110;  // Controlled rotational speed during line-lost recovery

// ---------------------------------------------------------------------------
// SENSOR PROCESSING & PID RUNTIME VARIABLES
// ---------------------------------------------------------------------------
int   rawSensors[8];         // Raw 10-bit ADC values (0 - 1023)
int   normalizedSensors[8];  // Calibrated reflectance values (0: White, 1000: Black)
float lastError    = 0.0f;   // Previous iteration error for derivative calculation
float integral     = 0.0f;   // Accumulated integral error
float lastCentroid = 3500.0f;// Last valid calculated centroid position
bool  lineLost     = false;  // Flag indicating no sensor detects the line

// ---------------------------------------------------------------------------
// NON-BLOCKING BUTTON DEBOUNCE VARIABLES
// ---------------------------------------------------------------------------
unsigned long lastBtn1DebounceTime   = 0;
bool          lastBtn1Reading        = HIGH;
bool          btn1State              = HIGH;

unsigned long lastBtn2DebounceTime   = 0;
bool          lastBtn2Reading        = HIGH;
bool          btn2State              = HIGH;
unsigned long btn2PressStartTime     = 0;
bool          btn2LongPressHandled   = false;

const unsigned long DEBOUNCE_DELAY_MS = 50;
const unsigned long LONG_PRESS_MS     = 1500;

// ---------------------------------------------------------------------------
// FORWARD FUNCTION DECLARATIONS
// ---------------------------------------------------------------------------
void setup();
void loop();
void handleButtons();
void enterCalibration();
void calibrateWhite();
void calibrateBlack();
void calibrateCenter();
void executeMotorTest();
void readSensors();
float computeCentroid();
void computePID();
void setMotors(int leftSpeed, int rightSpeed);
void saveEEPROM();
bool loadEEPROM();
void blinkLED(int times, int onDuration, int offDuration);

// ---------------------------------------------------------------------------
// ARDUINO SETUP
// ---------------------------------------------------------------------------
void setup() {
  // Initialize Serial Telemetry & Debug Stream (9600 Baud per connections.txt)
  Serial.begin(9600);
  Serial.println(F("=================================================="));
  Serial.println(F("   ARDUINO NANO PID LINE FOLLOWER ROBOT (LFR)     "));
  Serial.println(F("=================================================="));

  // Motor Driver GPIO Initialization
  pinMode(PIN_BIN1, OUTPUT);
  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_PWMB, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);

  // Guarantee motors are stopped on boot
  setMotors(0, 0);

  // UI Components Initialization
  pinMode(PIN_BTN_START, INPUT_PULLUP);
  pinMode(PIN_BTN_CALIB, INPUT_PULLUP);
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_STATUS, LOW);

  // Sensor Array Analog Inputs
  for (int i = 0; i < 8; i++) {
    pinMode(SENSOR_PINS[i], INPUT);
  }

  // Restore calibration profile from EEPROM or apply defaults
  loadEEPROM();

  Serial.println(F("[SYSTEM] Initialization complete. Robot in IDLE state."));
  Serial.println(F("[SYSTEM] Press Button 1 (D11) to START/STOP PID line following."));
  Serial.println(F("[SYSTEM] Hold Button 2 (D12) >1.5s to enter CALIBRATION mode."));
}

// ---------------------------------------------------------------------------
// MAIN EXECUTION LOOP
// ---------------------------------------------------------------------------
void loop() {
  // Process UI Pushbuttons
  handleButtons();

  // State Machine Dispatcher
  switch (robotState) {
    case IDLE:
      setMotors(0, 0);
      digitalWrite(PIN_LED_STATUS, LOW);
      break;

    case RUNNING_PID:
      readSensors();
      computePID();
      break;

    case CALIB_WHITE:
    case CALIB_BLACK:
    case CALIB_CENTER:
      // Stationary states waiting for Button 2 confirmation
      setMotors(0, 0);
      break;

    case MOTOR_TEST:
      executeMotorTest();
      break;
  }
}

// ---------------------------------------------------------------------------
// BUTTON HANDLING & USER INTERACTION (Non-blocking)
// ---------------------------------------------------------------------------
void handleButtons() {
  unsigned long currentMillis = millis();

  // --- BUTTON 1: START / STOP PID TOGGLE (D11) ---
  bool reading1 = digitalRead(PIN_BTN_START);
  if (reading1 != lastBtn1Reading) {
    lastBtn1DebounceTime = currentMillis;
  }

  if ((currentMillis - lastBtn1DebounceTime) > DEBOUNCE_DELAY_MS) {
    if (reading1 != btn1State) {
      btn1State = reading1;
      if (btn1State == LOW) { // Button 1 Pressed (Active-LOW Falling Edge)
        if (robotState == IDLE) {
          Serial.println(F("[SYSTEM] Button 1 pressed: Starting PID Line Following."));
          blinkLED(3, 100, 100); // Visual launch countdown
          lastError = 0.0f;
          integral = 0.0f;
          lastCentroid = calData.centerSetpoint;
          robotState = RUNNING_PID;
        } else if (robotState == RUNNING_PID) {
          Serial.println(F("[SYSTEM] Button 1 pressed: Stopping robot."));
          robotState = IDLE;
          setMotors(0, 0);
          digitalWrite(PIN_LED_STATUS, LOW);
        } else {
          // If in any calibration state, Button 1 aborts to IDLE safely
          Serial.println(F("[SYSTEM] Calibration aborted by user. Returning to IDLE."));
          robotState = IDLE;
          setMotors(0, 0);
          digitalWrite(PIN_LED_STATUS, LOW);
        }
      }
    }
  }
  lastBtn1Reading = reading1;

  // --- BUTTON 2: CALIBRATION TRIGGER & ADVANCE / EMERGENCY STOP (D12) ---
  bool reading2 = digitalRead(PIN_BTN_CALIB);
  if (reading2 != lastBtn2Reading) {
    lastBtn2DebounceTime = currentMillis;
  }

  if ((currentMillis - lastBtn2DebounceTime) > DEBOUNCE_DELAY_MS) {
    if (reading2 != btn2State) {
      btn2State = reading2;

      if (btn2State == LOW) {
        // Button 2 Pressed Down
        btn2PressStartTime = currentMillis;
        btn2LongPressHandled = false;

        // In RUNNING_PID, pressing Button 2 acts as immediate Emergency Stop
        if (robotState == RUNNING_PID) {
          Serial.println(F("[SYSTEM] Emergency Stop triggered via Button 2!"));
          robotState = IDLE;
          setMotors(0, 0);
          digitalWrite(PIN_LED_STATUS, LOW);
        }
      } else {
        // Button 2 Released
        // In calibration states, any button release advances the step.
        // Long-press guard only applies to IDLE entry.
        if (!btn2LongPressHandled) {
          if (robotState == CALIB_WHITE) {
            calibrateWhite();
          } else if (robotState == CALIB_BLACK) {
            calibrateBlack();
          } else if (robotState == CALIB_CENTER) {
            calibrateCenter();
          }
        }
      }
    }
  }
  lastBtn2Reading = reading2;

  // Long-press detection for Button 2 while in IDLE state (>1.5s)
  if (btn2State == LOW && !btn2LongPressHandled) {
    if (robotState == IDLE && (currentMillis - btn2PressStartTime >= LONG_PRESS_MS)) {
      btn2LongPressHandled = true;
      enterCalibration();
    }
  }
}

// ---------------------------------------------------------------------------
// CALIBRATION WORKFLOW FUNCTIONS
// ---------------------------------------------------------------------------
/**
 * @brief Enters calibration mode after >1.5s long press on Button 2.
 * Blinks status LED 3 times and prompts user for white baseline.
 */
void enterCalibration() {
  Serial.println(F("\n>>> ENTERING CALIBRATION MODE <<<"));
  setMotors(0, 0);
  blinkLED(3, 200, 200); // 3 blinks indicates calibration entry
  Serial.println(F("[CALIB STEP 1/3] Place robot on pure WHITE surface."));
  Serial.println(F("                 Press Button 2 (short press) to capture white baseline."));
  robotState = CALIB_WHITE;
}

/**
 * @brief Measures white baseline ADC values across all 8 sensors.
 * Blinks LED 2 times and advances to CALIB_BLACK.
 */
void calibrateWhite() {
  Serial.println(F("[CALIB] Sampling White Baseline (50 iterations)..."));
  long sums[8] = {0};
  const int SAMPLES = 50;

  for (int s = 0; s < SAMPLES; s++) {
    for (int i = 0; i < 8; i++) {
      sums[i] += analogRead(SENSOR_PINS[i]);
    }
    delay(3);
  }

  for (int i = 0; i < 8; i++) {
    calData.sensorMin[i] = (uint16_t)(sums[i] / SAMPLES);
    Serial.print(F(" S")); Serial.print(i); Serial.print(F(" Min(White): "));
    Serial.print(calData.sensorMin[i]);
  }
  Serial.println();

  blinkLED(2, 200, 200); // 2 blinks confirmation
  Serial.println(F("[CALIB STEP 2/3] Place robot on BLACK line/sheet."));
  Serial.println(F("                 Press Button 2 (short press) to capture black baseline."));
  robotState = CALIB_BLACK;
}

/**
 * @brief Measures black baseline ADC values across all 8 sensors.
 * Blinks LED 2 times and advances to CALIB_CENTER.
 */
void calibrateBlack() {
  Serial.println(F("[CALIB] Sampling Black Baseline (50 iterations)..."));
  long sums[8] = {0};
  const int SAMPLES = 50;

  for (int s = 0; s < SAMPLES; s++) {
    for (int i = 0; i < 8; i++) {
      sums[i] += analogRead(SENSOR_PINS[i]);
    }
    delay(3);
  }

  for (int i = 0; i < 8; i++) {
    calData.sensorMax[i] = (uint16_t)(sums[i] / SAMPLES);
    Serial.print(F(" S")); Serial.print(i); Serial.print(F(" Max(Black): "));
    Serial.print(calData.sensorMax[i]);
  }
  Serial.println();

  blinkLED(2, 200, 200); // 2 blinks confirmation
  Serial.println(F("[CALIB STEP 3/3] Align robot dead-center over the line."));
  Serial.println(F("                 Press Button 2 (short press) to capture center setpoint."));
  robotState = CALIB_CENTER;
}

/**
 * @brief Measures center position setpoint with robot aligned on the line.
 * Blinks LED 2 times and advances to MOTOR_TEST.
 */
void calibrateCenter() {
  Serial.println(F("[CALIB] Measuring Center Position Setpoint..."));
  float sumCentroid = 0.0f;
  int validSamples = 0;
  const int SAMPLES = 50;

  for (int s = 0; s < SAMPLES; s++) {
    readSensors();
    float c = computeCentroid();
    if (!lineLost) {
      sumCentroid += c;
      validSamples++;
    }
    delay(3);
  }

  if (validSamples > 0) {
    calData.centerSetpoint = sumCentroid / (float)validSamples;
  } else {
    // Fallback to theoretical geometric center of 8 sensors (range 0 to 7000)
    calData.centerSetpoint = 3500.0f;
    Serial.println(F("[CALIB] Warning: Line not strongly seen. Assigned default center 3500.0."));
  }

  Serial.print(F("[CALIB] Calibrated Center Setpoint: "));
  Serial.println(calData.centerSetpoint);

  blinkLED(2, 200, 200); // 2 blinks confirmation
  Serial.println(F("[CALIB] Starting Motor Verification Test..."));
  robotState = MOTOR_TEST;
}

/**
 * @brief Verifies motor wiring and direction by moving forward 500ms,
 * backward 500ms, stopping, saving EEPROM, and blinking LED 4 times fast.
 */
void executeMotorTest() {
  delay(600); // 600ms safety pause so user can release robot after center step blinks
  Serial.println(F("[MOTOR TEST] Forward verification (500ms @ PWM 120)..."));
  setMotors(120, 120);
  delay(500);

  // Brief active dynamic brake to eliminate forward inertia and back-EMF spike
  setMotors(0, 0);
  delay(100);

  Serial.println(F("[MOTOR TEST] Backward verification (500ms @ PWM -120)..."));
  setMotors(-120, -120);
  delay(500);

  Serial.println(F("[MOTOR TEST] Stopping motors."));
  setMotors(0, 0);
  delay(200);

  // Commit calibrated parameters to non-volatile EEPROM
  saveEEPROM();

  // 4 Fast blinks indicate complete calibration cycle
  blinkLED(4, 100, 100);

  Serial.println(F(">>> CALIBRATION & VERIFICATION COMPLETE! <<<\n"));
  robotState = IDLE;
}

// ---------------------------------------------------------------------------
// SENSOR READING & NORMALIZATION
// ---------------------------------------------------------------------------
/**
 * @brief Reads all 8 analog IR sensors and normalizes each between 0 (White)
 * and 1000 (Black) using the stored EEPROM calibration baselines.
 */
void readSensors() {
  for (int i = 0; i < 8; i++) {
    rawSensors[i] = analogRead(SENSOR_PINS[i]);

    long span = (long)calData.sensorMax[i] - (long)calData.sensorMin[i];
    if (abs(span) < 20) {
      span = (span >= 0) ? 20 : -20; // Guard against division by zero
    }

    // Map ADC reading to 0 - 1000 range
    long val = ((long)rawSensors[i] - (long)calData.sensorMin[i]) * 1000L / span;
    normalizedSensors[i] = (int)constrain(val, 0L, 1000L);
  }
}

// ---------------------------------------------------------------------------
// WEIGHTED CENTROID CALCULATION & LINE-LOST DETECTION
// ---------------------------------------------------------------------------
/**
 * @brief Calculates weighted center-of-gravity (centroid) across 8 sensors.
 * Sensor physical weight coordinates:
 * S0: 0, S1: 1000, S2: 2000, S3: 3000, S4: 4000, S5: 5000, S6: 6000, S7: 7000
 * @return Calculated position (0.0 to 7000.0)
 */
float computeCentroid() {
  unsigned long weightedSum = 0;
  unsigned long totalWeight = 0;

  for (int i = 0; i < 8; i++) {
    weightedSum += (unsigned long)normalizedSensors[i] * (i * 1000UL);
    totalWeight += (unsigned long)normalizedSensors[i];
  }

  // Threshold check: If sum of all normalized sensors is very low, line is lost
  if (totalWeight < 200) {
    lineLost = true;
    return lastCentroid; // Retain last known position
  }

  lineLost = false;
  float centroid = (float)weightedSum / (float)totalWeight;
  lastCentroid = centroid;
  return centroid;
}

// ---------------------------------------------------------------------------
// PID CONTROLLER & FAIL-SAFE STEERING
// ---------------------------------------------------------------------------
/**
 * @brief Computes Proportional-Derivative (PD) control correction and sets
 * differential drive motor speeds. Implements line-lost recovery.
 */
void computePID() {
  float position = computeCentroid();
  float error = position - calData.centerSetpoint;

  // --- LINE-LOST FAIL-SAFE RECOVERY ---
  if (lineLost) {
    digitalWrite(PIN_LED_STATUS, HIGH); // Light up LED during line search

    // Steer aggressively towards the side where line was last detected
    if (lastError > 0.0f) {
      // Line was to the right -> Pivot turn right
      setMotors(recoverySpeed, -recoverySpeed / 2);
    } else {
      // Line was to the left -> Pivot turn left
      setMotors(-recoverySpeed / 2, recoverySpeed);
    }
    return;
  }

  digitalWrite(PIN_LED_STATUS, LOW);

  // Proportional Term
  float pTerm = Kp * error;

  // Integral Term (Ki is 0.0 by default, anti-windup clamping implemented)
  integral += error;
  integral = constrain(integral, -10000.0f, 10000.0f);
  float iTerm = Ki * integral;

  // Derivative Term (Counteracts rate of change of error)
  float dTerm = Kd * (error - lastError);
  lastError = error;

  // Composite PID Steering Correction
  float correction = pTerm + iTerm + dTerm;

  // Differential Motor Mixing:
  // Positive error/correction -> Line is on Right -> Left Motor Speeds Up, Right Motor Slows Down
  // Negative error/correction -> Line is on Left -> Left Motor Slows Down, Right Motor Speeds Up
  int leftMotorSpeed  = baseSpeed + (int)correction;
  int rightMotorSpeed = baseSpeed - (int)correction;

  // Enforce hardware speed boundaries
  leftMotorSpeed  = constrain(leftMotorSpeed, minSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);

  setMotors(leftMotorSpeed, rightMotorSpeed);
}

// ---------------------------------------------------------------------------
// MOTOR DRIVER HARDWARE INTERFACE (TB6612FNG)
// ---------------------------------------------------------------------------
/**
 * @brief Controls Left and Right motor directions and PWM duty cycles.
 * @param leftSpeed  Left motor speed (-255 to 255)
 * @param rightSpeed Right motor speed (-255 to 255)
 */
void setMotors(int leftSpeed, int rightSpeed) {
  // --- LEFT MOTOR (BIN1: D2, BIN2: D4, PWMB: D5) ---
  if (leftSpeed > 0) {
    digitalWrite(PIN_BIN1, HIGH);
    digitalWrite(PIN_BIN2, LOW);
    analogWrite(PIN_PWMB, constrain(leftSpeed, 0, 255));
  } else if (leftSpeed < 0) {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, HIGH);
    analogWrite(PIN_PWMB, constrain(-leftSpeed, 0, 255));
  } else {
    digitalWrite(PIN_BIN1, LOW);
    digitalWrite(PIN_BIN2, LOW);
    analogWrite(PIN_PWMB, 0);
  }

  // --- RIGHT MOTOR (AIN1: D7, AIN2: D8, PWMA: D6) ---
  if (rightSpeed > 0) {
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, constrain(rightSpeed, 0, 255));
  } else if (rightSpeed < 0) {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, HIGH);
    analogWrite(PIN_PWMA, constrain(-rightSpeed, 0, 255));
  } else {
    digitalWrite(PIN_AIN1, LOW);
    digitalWrite(PIN_AIN2, LOW);
    analogWrite(PIN_PWMA, 0);
  }
}

// ---------------------------------------------------------------------------
// EEPROM STORAGE MANAGEMENT
// ---------------------------------------------------------------------------
/**
 * @brief Saves calibration data structure to ATmega328P internal EEPROM.
 */
void saveEEPROM() {
  calData.magic = EEPROM_MAGIC_BYTE;
  EEPROM.put(EEPROM_START_ADDR, calData);
  Serial.println(F("[EEPROM] Calibration parameters committed to EEPROM."));
}

/**
 * @brief Reads calibration data from internal EEPROM.
 * Verifies magic byte (0x5A); falls back to defaults if not found.
 * @return true if valid EEPROM data loaded, false if defaults applied.
 */
bool loadEEPROM() {
  EEPROM.get(EEPROM_START_ADDR, calData);

  if (calData.magic == EEPROM_MAGIC_BYTE && !isnan(calData.centerSetpoint) && calData.centerSetpoint >= 0.0f && calData.centerSetpoint <= 7000.0f) {
    Serial.println(F("[EEPROM] Calibration profile verified and loaded from EEPROM."));
    Serial.print(F("[EEPROM] Center Setpoint: "));
    Serial.println(calData.centerSetpoint);
    return true;
  } else {
    Serial.println(F("[EEPROM] No saved calibration detected (Magic mismatch). Applying defaults."));
    calData.magic = EEPROM_MAGIC_BYTE;
    for (int i = 0; i < 8; i++) {
      calData.sensorMin[i] = 100; // Typical White floor
      calData.sensorMax[i] = 900; // Typical Black ceiling
    }
    calData.centerSetpoint = 3500.0f; // Nominal geometric center
    return false;
  }
}

// ---------------------------------------------------------------------------
// STATUS LED UTILITY
// ---------------------------------------------------------------------------
/**
 * @brief Blinks on-board status LED (D13) a specified number of times.
 * @param times       Number of blink cycles
 * @param onDuration  LED ON duration in milliseconds
 * @param offDuration LED OFF duration in milliseconds
 */
void blinkLED(int times, int onDuration, int offDuration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_LED_STATUS, HIGH);
    delay(onDuration);
    digitalWrite(PIN_LED_STATUS, LOW);
    if (i < times - 1) {
      delay(offDuration);
    }
  }
}
