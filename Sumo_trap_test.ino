#include <Arduino.h>
/************************************************************
 * Robust SUMO edge-avoid game with RANDOM choice on equal edge
 * and OUT-OF-RING SAFETY (both sensors white too long)
 *
 * - 2x QTR-1RC on pins 2 & 3 (blocking read, very short)
 * Board assumed: Arduino Uno-style (AVR)
 ************************************************************/

/********************* DEBUG CONFIG **************************/

#define DEBUG_ENABLED 0

#if DEBUG_ENABLED
  #define DEBUG_BEGIN(baud)        Serial.begin(baud)
  #define DEBUG_PRINT(...)         Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTLN(...)       Serial.println(__VA_ARGS__)
#else
  #define DEBUG_BEGIN(baud)
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTLN(...)
#endif

/********************* GLOBAL CONSTANTS ************************/

// -------- Debug --------
const unsigned long DEBUG_BAUDRATE            = 115200;
const unsigned long DEBUG_PRINT_INTERVAL_MS   = 200;   // how often to print sensor values

// -------- Button / LED --------
#define BTN_PIN 4            // start/stop button pin
#define LED     11           // game status LED pin
const uint8_t BUTTON_ACTIVE_LEVEL             = LOW;   // using INPUT_PULLUP
const unsigned long BUTTON_DEBOUNCE_MS        = 50;    // debounce time

// Random seed pin (any floating analog pin)
const uint8_t RNG_SEED_PIN                    = A0;

// -------- QTR-1RC sensors --------
const uint8_t QTR_LEFT_PIN                    = 2;
const uint8_t QTR_RIGHT_PIN                   = 3;

// QTR timing (microseconds)
const unsigned long QTR_CHARGE_TIME_US        = 10;    // time to charge RC
const unsigned long QTR_TIMEOUT_US            = 3000;  // max discharge time

// Threshold for white edge detection (tune on your ring)
// smaller value = whiter surface (short discharge time)
const uint16_t QTR_EDGE_WHITE_THRESHOLD_US    = 500;   // base "white" threshold

// When both see white, how different must they be to decide a “whiter” side
const uint16_t QTR_BOTH_DECISION_DEADZONE_US  = 80;    // if abs(L-R) < this -> treat as equal

// -------- Motor pins & config --------
#define LEFT_MOTOR_IN1_PIN   7
#define LEFT_MOTOR_IN2_PIN   8
#define LEFT_MOTOR_PWM_PIN   5

#define RIGHT_MOTOR_IN1_PIN  9
#define RIGHT_MOTOR_IN2_PIN  10
#define RIGHT_MOTOR_PWM_PIN  6

const int16_t MOTOR_MAX_SPEED                 = 255;
const int16_t MOTOR_SPEED_FORWARD             = 80;
const int16_t MOTOR_SPEED_BACKWARD            = 200;
const int16_t MOTOR_SPEED_TURN                = 180;

// invert both because of wiring
bool invertLeftMotorDirection                 = true;
bool invertRightMotorDirection                = true;

// -------- Game timings (all non-blocking) --------
const unsigned long EDGE_STOP_DURATION_MS     = 120;   // small visible pause at edge

// Backing up: keep going until both black, but clamp duration
const unsigned long EDGE_BACK_MIN_MS          = 150;   // always back at least this much
const unsigned long EDGE_BACK_MAX_MS          = 400;   // but not longer than this

// Turning: re-check sensors while turning
const unsigned long EDGE_TURN_MIN_MS          = 160;   // minimum spin time
const unsigned long EDGE_TURN_MAX_MS          = 450;   // safety clamp

const unsigned long LOOP_PERIOD_MS            = 5;     // main loop min period

// -------- Out-of-ring safety --------
// If BOTH sensors stay white continuously longer than this, assume we left the ring
const unsigned long OUT_OF_RING_WHITE_TIMEOUT_MS = 800;  // ms, tune if needed

/********************* GLOBAL STATE ****************************/

// Debug / monitoring
unsigned long lastDebugPrintTimeMs            = 0;
unsigned long lastLoopTimeMs                  = 0;

// Current motor command (for debugging / logging)
int16_t currentLeftMotorSpeedCommand          = 0;
int16_t currentRightMotorSpeedCommand         = 0;

// Button state (debounced)
bool buttonStableState                        = HIGH;  // due to INPUT_PULLUP
bool buttonLastReadState                      = HIGH;
unsigned long buttonLastChangeTimeMs          = 0;
bool buttonPressEventPending                  = false; // becomes true on clean press

// Robot state machine
enum RobotState {
  STATE_WAIT_START,
  STATE_DRIVE_FORWARD,
  STATE_EDGE_STOP,
  STATE_EDGE_BACK,
  STATE_EDGE_TURN
};

RobotState currentState                       = STATE_WAIT_START;

// What edge caused the escape?
enum EdgeEvent {
  EDGE_NONE,
  EDGE_LEFT,
  EDGE_RIGHT,
  EDGE_BOTH
};

EdgeEvent lastEdgeEvent                       = EDGE_NONE;

// Effective direction decision after analyzing EDGE_BOTH
EdgeEvent effectiveEdgeForTurn                = EDGE_NONE;

// Time when we entered the current state
unsigned long stateStartTimeMs                = 0;

// Last QTR readings (for debug & logic)
uint16_t qtrLeftValueUs                       = 0;
uint16_t qtrRightValueUs                      = 0;
bool qtrLeftIsWhite                           = false;
bool qtrRightIsWhite                          = false;

// QTR values at the moment the edge was detected
uint16_t edgeLeftValueUsAtDetection           = 0;
uint16_t edgeRightValueUsAtDetection          = 0;

// Out-of-ring watchdog
bool bothWhiteOngoing                         = false;
unsigned long bothWhiteStartMs                = 0;

/********************* QTR READING *****************************/

// Fast, bounded RC timing read for one QTR-1RC
uint16_t readQTR_RC_Us(uint8_t pin) {
  // 1) Charge the capacitor
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  unsigned long chargeStart = micros();
  while ((micros() - chargeStart) < QTR_CHARGE_TIME_US) {
    // tight microsecond wait
  }

  // 2) Set as input and measure discharge time
  pinMode(pin, INPUT);
  unsigned long dischargeStart = micros();

  while (digitalRead(pin) == HIGH) {
    unsigned long elapsed = micros() - dischargeStart;
    if (elapsed >= QTR_TIMEOUT_US) {
      return static_cast<uint16_t>(QTR_TIMEOUT_US);
    }
  }

  return static_cast<uint16_t>(micros() - dischargeStart);
}

void updateQTRReadings() {
  qtrLeftValueUs  = readQTR_RC_Us(QTR_LEFT_PIN);
  qtrRightValueUs = readQTR_RC_Us(QTR_RIGHT_PIN);

  qtrLeftIsWhite  = (qtrLeftValueUs  < QTR_EDGE_WHITE_THRESHOLD_US);
  qtrRightIsWhite = (qtrRightValueUs < QTR_EDGE_WHITE_THRESHOLD_US);
}

/********************* MOTOR CONTROL ***************************/

void setMotorSpeeds(int16_t leftMotorSpeed, int16_t rightMotorSpeed) {
  if (invertLeftMotorDirection) {
    leftMotorSpeed = -leftMotorSpeed;
  }
  if (invertRightMotorDirection) {
    rightMotorSpeed = -rightMotorSpeed;
  }

  leftMotorSpeed  = constrain(leftMotorSpeed,  -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
  rightMotorSpeed = constrain(rightMotorSpeed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

  currentLeftMotorSpeedCommand  = leftMotorSpeed;
  currentRightMotorSpeedCommand = rightMotorSpeed;

  // LEFT MOTOR
  if (leftMotorSpeed > 0) {
    digitalWrite(LEFT_MOTOR_IN1_PIN, HIGH);
    digitalWrite(LEFT_MOTOR_IN2_PIN, LOW);
    analogWrite(LEFT_MOTOR_PWM_PIN, leftMotorSpeed);
  } else if (leftMotorSpeed < 0) {
    digitalWrite(LEFT_MOTOR_IN1_PIN, LOW);
    digitalWrite(LEFT_MOTOR_IN2_PIN, HIGH);
    analogWrite(LEFT_MOTOR_PWM_PIN, -leftMotorSpeed);
  } else {
    analogWrite(LEFT_MOTOR_PWM_PIN, 0);
  }

  // RIGHT MOTOR
  if (rightMotorSpeed > 0) {
    digitalWrite(RIGHT_MOTOR_IN1_PIN, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2_PIN, LOW);
    analogWrite(RIGHT_MOTOR_PWM_PIN, rightMotorSpeed);
  } else if (rightMotorSpeed < 0) {
    digitalWrite(RIGHT_MOTOR_IN1_PIN, LOW);
    digitalWrite(RIGHT_MOTOR_IN2_PIN, HIGH);
    analogWrite(RIGHT_MOTOR_PWM_PIN, -rightMotorSpeed);
  } else {
    analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
  }
}

void stopMotors() {
  setMotorSpeeds(0, 0);
}

/********************* BUTTON HANDLING *************************/

void updateButtonState(unsigned long nowMs) {
  bool rawState = digitalRead(BTN_PIN);

  if (rawState != buttonLastReadState) {
    buttonLastReadState     = rawState;
    buttonLastChangeTimeMs  = nowMs;
  }

  if ((nowMs - buttonLastChangeTimeMs) >= BUTTON_DEBOUNCE_MS &&
      rawState != buttonStableState) {
    buttonStableState = rawState;

    if (buttonStableState == BUTTON_ACTIVE_LEVEL) {
      buttonPressEventPending = true;
      DEBUG_PRINTLN(F("Button press detected (debounced)"));
    }
  }
}

/********************* EDGE DECISION LOGIC *********************/

EdgeEvent decideEffectiveEdgeDirection() {
  if (lastEdgeEvent == EDGE_LEFT) {
    return EDGE_LEFT;
  }
  if (lastEdgeEvent == EDGE_RIGHT) {
    return EDGE_RIGHT;
  }

  // If both sensors saw white, analyze which one is "whiter"
  if (lastEdgeEvent == EDGE_BOTH) {
    int16_t diff = (int16_t)edgeLeftValueUsAtDetection -
                   (int16_t)edgeRightValueUsAtDetection;

    if (abs(diff) <= (int16_t)QTR_BOTH_DECISION_DEADZONE_US) {
      // Very similar → RANDOM safe choice: left or right
      DEBUG_PRINTLN(F("EDGE_BOTH ~equal → RANDOM TURN"));
      int decide = random(1001) % 2;   // 0 or 1
      if (decide == 0) {
        DEBUG_PRINTLN(F("  RANDOM choice → LEFT"));
        return EDGE_LEFT;   // treat as left edge → turn right later
      } else {
        DEBUG_PRINTLN(F("  RANDOM choice → RIGHT"));
        return EDGE_RIGHT;  // treat as right edge → turn left later
      }
    }

    if (diff < 0) {
      // Left discharge time is smaller → left is whiter → edge more on left → turn right
      DEBUG_PRINTLN(F("EDGE_BOTH → left whiter → turn RIGHT"));
      return EDGE_LEFT;
    } else {
      // Right is whiter → edge more on right → turn left
      DEBUG_PRINTLN(F("EDGE_BOTH → right whiter → turn LEFT"));
      return EDGE_RIGHT;
    }
  }

  // Fallback: if sensors currently see something, use that as hint
  if (qtrLeftIsWhite && !qtrRightIsWhite) return EDGE_LEFT;
  if (!qtrLeftIsWhite && qtrRightIsWhite) return EDGE_RIGHT;

  return EDGE_NONE;
}

/********************* OUT-OF-RING WATCHDOG ********************/

// Returns true if it forced a stop & state change
bool handleOutOfRingTimeout(unsigned long nowMs) {
  bool bothWhite = qtrLeftIsWhite && qtrRightIsWhite;

  if (bothWhite) {
    if (!bothWhiteOngoing) {
      bothWhiteOngoing   = true;
      bothWhiteStartMs   = nowMs;
    } else {
      unsigned long elapsed = nowMs - bothWhiteStartMs;
      if (elapsed >= OUT_OF_RING_WHITE_TIMEOUT_MS) {
        DEBUG_PRINT(F("SAFETY: both sensors white too long ("));
        DEBUG_PRINT(elapsed);
        DEBUG_PRINTLN(F(" ms) → STOP & WAIT_START"));

        stopMotors();
        digitalWrite(LED, LOW);

        lastEdgeEvent         = EDGE_NONE;
        effectiveEdgeForTurn  = EDGE_NONE;
        bothWhiteOngoing      = false;

        // Go to WAIT_START; user must press button again
        currentState = STATE_WAIT_START;
        stateStartTimeMs = nowMs;
        return true;
      }
    }
  } else {
    bothWhiteOngoing = false;
  }

  return false;
}

/********************* STATE MACHINE ***************************/

void enterState(RobotState newState) {
  currentState     = newState;
  stateStartTimeMs = millis();

  DEBUG_PRINT(F("STATE -> "));
  switch (currentState) {
    case STATE_WAIT_START:    DEBUG_PRINTLN(F("WAIT_START"));    break;
    case STATE_DRIVE_FORWARD: DEBUG_PRINTLN(F("DRIVE_FORWARD")); break;
    case STATE_EDGE_STOP:     DEBUG_PRINTLN(F("EDGE_STOP"));      break;
    case STATE_EDGE_BACK:     DEBUG_PRINTLN(F("EDGE_BACK"));      break;
    case STATE_EDGE_TURN:     DEBUG_PRINTLN(F("EDGE_TURN"));      break;
    default:                  DEBUG_PRINTLN(F("UNKNOWN"));        break;
  }
}

// Handle global start/stop toggle; returns true if it changed state
bool handleButtonToggle() {
  if (!buttonPressEventPending) {
    return false;
  }
  buttonPressEventPending = false;

  if (currentState == STATE_WAIT_START) {
    digitalWrite(LED, HIGH);           // LED ON when game runs
    enterState(STATE_DRIVE_FORWARD);
  } else {
    stopMotors();
    digitalWrite(LED, LOW);            // LED OFF when stopped
    enterState(STATE_WAIT_START);
  }

  return true;
}

void updateStateMachine(unsigned long nowMs) {
  // First: check if button wants to toggle start/stop
  if (handleButtonToggle()) {
    return;
  }

  // Global safety: if we stayed on white too long → hard stop
  if (handleOutOfRingTimeout(nowMs)) {
    return;
  }

  switch (currentState) {

    case STATE_WAIT_START: {
      stopMotors();
      break;
    }

    case STATE_DRIVE_FORWARD: {
      // If an edge is seen, immediately go to escape sequence
      if (qtrLeftIsWhite || qtrRightIsWhite) {
        edgeLeftValueUsAtDetection  = qtrLeftValueUs;
        edgeRightValueUsAtDetection = qtrRightValueUs;

        if (qtrLeftIsWhite && !qtrRightIsWhite) {
          lastEdgeEvent = EDGE_LEFT;
        } else if (!qtrLeftIsWhite && qtrRightIsWhite) {
          lastEdgeEvent = EDGE_RIGHT;
        } else {
          lastEdgeEvent = EDGE_BOTH;
        }

        DEBUG_PRINT(F("EDGE detected in FORWARD: "));
        if (lastEdgeEvent == EDGE_LEFT)  DEBUG_PRINTLN(F("LEFT"));
        if (lastEdgeEvent == EDGE_RIGHT) DEBUG_PRINTLN(F("RIGHT"));
        if (lastEdgeEvent == EDGE_BOTH) {
          DEBUG_PRINT(F("BOTH (L="));
          DEBUG_PRINT(edgeLeftValueUsAtDetection);
          DEBUG_PRINT(F("us, R="));
          DEBUG_PRINT(edgeRightValueUsAtDetection);
          DEBUG_PRINTLN(F("us)"));
        }

        stopMotors();
        enterState(STATE_EDGE_STOP);
        break;
      }

      setMotorSpeeds(MOTOR_SPEED_FORWARD, MOTOR_SPEED_FORWARD);
      break;
    }

    case STATE_EDGE_STOP: {
      stopMotors();
      if ((nowMs - stateStartTimeMs) >= EDGE_STOP_DURATION_MS) {
        enterState(STATE_EDGE_BACK);
      }
      break;
    }

    case STATE_EDGE_BACK: {
      setMotorSpeeds(-MOTOR_SPEED_BACKWARD, -MOTOR_SPEED_BACKWARD);

      unsigned long elapsed = nowMs - stateStartTimeMs;
      bool anyWhite         = (qtrLeftIsWhite || qtrRightIsWhite);

      if (elapsed >= EDGE_BACK_MIN_MS) {
        if (!anyWhite || (elapsed >= EDGE_BACK_MAX_MS)) {
          DEBUG_PRINT(F("BACK done: elapsed="));
          DEBUG_PRINT(elapsed);
          DEBUG_PRINT(F("ms, anyWhite="));
          DEBUG_PRINTLN(anyWhite ? F("YES") : F("NO"));
          enterState(STATE_EDGE_TURN);
        }
      }
      break;
    }

    case STATE_EDGE_TURN: {
      if (effectiveEdgeForTurn == EDGE_NONE) {
        effectiveEdgeForTurn = decideEffectiveEdgeDirection();
      }

      int16_t leftSpeed  = 0;
      int16_t rightSpeed = 0;

      if (effectiveEdgeForTurn == EDGE_LEFT) {
        leftSpeed  = MOTOR_SPEED_TURN;
        rightSpeed = -MOTOR_SPEED_TURN;
      } else if (effectiveEdgeForTurn == EDGE_RIGHT) {
        leftSpeed  = -MOTOR_SPEED_TURN;
        rightSpeed = MOTOR_SPEED_TURN;
      } else {
        leftSpeed  = -MOTOR_SPEED_TURN;
        rightSpeed = MOTOR_SPEED_TURN;
      }

      setMotorSpeeds(leftSpeed, rightSpeed);

      unsigned long elapsed = nowMs - stateStartTimeMs;

      // Safety logic during turn
      if (qtrLeftIsWhite || qtrRightIsWhite) {
        DEBUG_PRINTLN(F("TURN: sensor saw WHITE again → BACK"));
        effectiveEdgeForTurn = EDGE_NONE;
        enterState(STATE_EDGE_BACK);
        break;
      }

      if (elapsed >= EDGE_TURN_MIN_MS) {
        if (elapsed >= EDGE_TURN_MAX_MS) {
          DEBUG_PRINTLN(F("TURN: timeout → FORWARD"));
          effectiveEdgeForTurn = EDGE_NONE;
          enterState(STATE_DRIVE_FORWARD);
        } else {
          if (!qtrLeftIsWhite && !qtrRightIsWhite) {
            DEBUG_PRINTLN(F("TURN: both black & min time → FORWARD"));
            effectiveEdgeForTurn = EDGE_NONE;
            enterState(STATE_DRIVE_FORWARD);
          }
        }
      }
      break;
    }

    default:
      enterState(STATE_WAIT_START);
      break;
  }
}

/********************* DEBUG OUTPUT ****************************/

void maybePrintDebug(unsigned long nowMs) {
  if ((nowMs - lastDebugPrintTimeMs) < DEBUG_PRINT_INTERVAL_MS) {
    return;
  }
  lastDebugPrintTimeMs = nowMs;

  DEBUG_PRINT(F("QTR L="));
  DEBUG_PRINT(qtrLeftValueUs);
  DEBUG_PRINT(F("us ("));
  DEBUG_PRINT(qtrLeftIsWhite ? F("W") : F("B"));
  DEBUG_PRINT(F(")  R="));
  DEBUG_PRINT(qtrRightValueUs);
  DEBUG_PRINT(F("us ("));
  DEBUG_PRINT(qtrRightIsWhite ? F("W") : F("B"));
  DEBUG_PRINT(F(")  Motors L="));
  DEBUG_PRINT(currentLeftMotorSpeedCommand);
  DEBUG_PRINT(F(" R="));
  DEBUG_PRINTLN(currentRightMotorSpeedCommand);
}

/********************* SETUP / LOOP ****************************/

void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  pinMode(RNG_SEED_PIN, INPUT);

  pinMode(LEFT_MOTOR_IN1_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_IN2_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);

  pinMode(RIGHT_MOTOR_IN1_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);

  stopMotors();
  digitalWrite(LED, LOW);

  DEBUG_BEGIN(DEBUG_BAUDRATE);
  DEBUG_PRINTLN(F("Robust sumo edge-avoid + random + safety: init"));

  // Seed RNG from a floating analog pin
  randomSeed(analogRead(RNG_SEED_PIN));

  unsigned long nowMs = millis();
  buttonStableState       = digitalRead(BTN_PIN);
  buttonLastReadState     = buttonStableState;
  buttonLastChangeTimeMs  = nowMs;

  enterState(STATE_WAIT_START);
}

void loop() {
  const unsigned long nowMs = millis();

  if ((nowMs - lastLoopTimeMs) < LOOP_PERIOD_MS) {
    return;
  }
  lastLoopTimeMs = nowMs;

  updateButtonState(nowMs);
  updateQTRReadings();
  updateStateMachine(nowMs);
  maybePrintDebug(nowMs);
}
