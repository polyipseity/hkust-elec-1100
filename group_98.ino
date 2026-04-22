/*
  ELEC1100 Your Lab#06 & Project Template

  To program the car tracking the white line on a dark mat

  Group No. (number of your project box):  
  Group Member 1 (name & SID):
  Group Member 2 (name & SID):
  
*/

// ========================= TUNABLE SETTINGS (at the top for easy access) =========================

// Mode selection
const int MODE_CONSTANT_PWM = 0;
const int MODE_LINE_TRACK = 1;
const int MODE_MISSION_TASK = 2;
const int MODE_TUNE_TURN_90 = 3;
const int MODE_TUNE_ROTATE_90 = 4;

// Which mode to run. Change ONLY this constant to switch behavior.
const int RUN_MODE = 2; // REMEMBER TO SET TO 2 OR MODE_MISSION_TASK!!!!!!!!!!

const int DEBUG_START_AT_STAGE = 3; // REMEMBER TO SET TO 3!!!!!!!!!!
// Debug stage stop (mission mode only):
// 0 = disabled
// N (3..18) = stop completely once stage N is entered
const int DEBUG_STOP_AT_STAGE = 19; // REMEMBER TO SET TO 19!!!!!!!!!!

// Three effective power levels: stop / half / full
const float POWER_STOP = 0.0; // stop
const float POWER_HALF = 0.55;  // half
const float POWER_FULL = 0.75;  // full
const float POWER_MAX = 1.0;

// Constant test mode power (use any defined POWER_* level)
const float CONSTANT_MODE_POWER = POWER_FULL;

// Right motor compensation multiplier (right is slower, so boost its PWM).
// Right PWM = left PWM * RIGHT_PWM_MULTIPLIER when both are commanded at same power.
const float RIGHT_PWM_MULTIPLIER = 1; // 1.054;

// Auto-computed max safe left PWM from multiplier (floor so right never exceeds 255).
const int MAX_LEFT_PWM = (int)(255.0 / RIGHT_PWM_MULTIPLIER);

// Global timing scale for mission mode (all state durations multiplied by this value)
const float MISSION_TIME_MULTIPLIER = 1.0;

// Mission timing model:
// - Number labels (1..18) are map TIME POINTS (waypoints).
// - currentState means active TIME INTERVAL [currentState -> currentState+1].
// - Stages 1 and 2 are handled by startup arming + bumper toggle, so mission mode starts at stage 3.
// - So duration/condition arrays are interval-based (indexed by currentState).
// Relative mission interval durations in ms (base time before multiplying by MISSION_TIME_MULTIPLIER)
// Each interval advances only after min duration; it is forced to advance at max duration.
// Keep windows narrow: max - min <= 1 s.
const unsigned long INTERVAL_MIN_DURATION_MS[19] = {
  0,
  0,   // 1 (startup-handled)
  0,   // 2 (startup-handled)
  1200,  // 3
  2300,  // 4
  5000,  // 5
  1200,  // 6
  950,  // 7 (self-rotation only)
  800,  // 8 (post-self-rotation causes variation)
  9200,   // 9  (retuned after slow->half mapping)
  700,  // 10
  900,   // 11
  1200,   // 12
  2600,   // 13
  3000,   // 14 (retuned after slow->half mapping)
  2600,   // 15
  1400,  // 16
  700,  // 17
  1100   // 18
};

const unsigned long INTERVAL_MAX_DURATION_MS[19] = {
  0,
  0,   // 1 (startup-handled)
  0,  // 2 (startup-handled)
  1700,  // 3
  2800,  // 4
  5700,  // 5
  1800,  // 6
  1100,  // 7 (self-rotation only)
  1400,  // 8 (post-self-rotation causes variation)
  10000,   // 9  (retuned after slow->half mapping)
  900,  // 10
  1100,  // 11
  1500,  // 12
  3100,  // 13
  4000,   // 14 (retuned after slow->half mapping)
  3100,  // 15
  1800,  // 16
  900,  // 17
  1300   // 18
};

// Cumulative time to REACH each stage point (seconds, before global multiplier)
// Stage 1:  0.00 ~ 0.00
// Stage 2:  0.30 ~ 0.50
// Stage 3:  1.00 ~ 1.50
// Stage 4:  3.90 ~ 4.80
// Stage 5:  4.80 ~ 6.00
// Stage 6:  6.80 ~ 8.40
// Stage 7:  8.00 ~ 10.00
// Stage 8:  9.70 ~ 12.10
// Stage 9: 10.90 ~ 13.70
// Stage 10: 11.35 ~ 14.40
// Stage 11: 13.15 ~ 16.60
// Stage 12: 13.85 ~ 17.60
// Stage 13: 14.55 ~ 18.60
// Stage 14: 15.25 ~ 19.60
// Stage 15: 15.80 ~ 20.40
// Stage 16: 16.60 ~ 21.50
// Stage 17: 18.00 ~ 23.30
// Stage 18: 19.20 ~ 24.90
// Mission end (after stage 18 hold): 22.20 ~ 28.30

// Mission actions per state (index 1..18)
// Note: states 1 and 2 are not run in mission mode (handled by startup gate).
const int ACT_STOP = 0;
const int ACT_LINE_TRACK = 1;
const int ACT_SPIN_360_RIGHT = 2;
const int ACT_BACKWARD_FAST = 3;

// Per-state transition condition (checked after min duration; forced by max duration)
const int COND_TIME_ONLY = 0;
const int COND_CENTER_ON_WHITE = 1;
const int COND_JUNCTION_WHITE = 2;
const int COND_BUMPER_ON_WHITE = 3;

// Stage-entry turn hint (used at the beginning of selected stages)
const int TURN_NONE = 0;
const int TURN_LEFT = -1;
const int TURN_RIGHT = 1;

// ========================= FIXED 90-DEGREE ACTIONS (FULLY TUNABLE) =========================
// 1) Turn 90 left  (fixed-radius arc)
// 2) Turn 90 right (fixed-radius arc)
// 3) Rotate 90 left  (self-rotation)
// 4) Rotate 90 right (self-rotation)
const int FIX_ACT_NONE = 0;
const int FIX_ACT_TURN_LEFT_90 = 1;
const int FIX_ACT_TURN_RIGHT_90 = 2;
const int FIX_ACT_ROTATE_LEFT_90 = 3;
const int FIX_ACT_ROTATE_RIGHT_90 = 4;

// Arc-turn (fixed radius) tuning
const float FIX_TURN_LEFT_90_INNER_POWER = POWER_STOP;
const float FIX_TURN_LEFT_90_OUTER_POWER = POWER_FULL;
const unsigned long FIX_TURN_LEFT_90_DURATION_MS = 220;

const float FIX_TURN_RIGHT_90_INNER_POWER = POWER_STOP;
const float FIX_TURN_RIGHT_90_OUTER_POWER = POWER_FULL;
const unsigned long FIX_TURN_RIGHT_90_DURATION_MS = 220;

// Self-rotation 90 tuning
const float FIX_ROTATE_LEFT_90_POWER = POWER_FULL;
const unsigned long FIX_ROTATE_LEFT_90_DURATION_MS = 220;

const float FIX_ROTATE_RIGHT_90_POWER = POWER_FULL;
const unsigned long FIX_ROTATE_RIGHT_90_DURATION_MS = 220;

// Self-rotation centering correction:
// after detection, move straight briefly so wheel-center aligns before rotating
const unsigned long FIX_ROTATE_CENTERING_FORWARD_MS = 50;
const float FIX_ROTATE_CENTERING_FORWARD_POWER = POWER_HALF;

// Tuning mode sequence length: 4 left + 4 right
const int TUNING_REPEAT_COUNT = 4;

// Stage 7 replacement: 4 x self-rotate right 90
const int STAGE7_ROTATE_90_COUNT = 4;

// Startup / debounce timing (tunable)
const unsigned long START_LINE_CONFIRM_MS = 120;
const unsigned long BOOT_SETTLE_DELAY_MS = 300;
const unsigned long START_TRIGGER_DEBOUNCE_MS = 120;
const unsigned long STAGE17_BUMPER_DEBOUNCE_MS = 80;

const int STATE_ACTION[19] = {
  ACT_STOP,
  ACT_STOP,               // 1 (startup-handled)
  ACT_STOP,               // 2 (startup-handled)
  ACT_LINE_TRACK,         // 3
  ACT_LINE_TRACK,         // 4
  ACT_LINE_TRACK,         // 5
  ACT_LINE_TRACK,         // 6
  ACT_SPIN_360_RIGHT,     // 7
  ACT_LINE_TRACK,         // 8
  ACT_LINE_TRACK,         // 9
  ACT_LINE_TRACK,         // 10
  ACT_LINE_TRACK,         // 11
  ACT_LINE_TRACK,         // 12
  ACT_LINE_TRACK,         // 13
  ACT_LINE_TRACK,         // 14
  ACT_LINE_TRACK,         // 15
  ACT_LINE_TRACK,         // 16
  ACT_BACKWARD_FAST,      // 17
  ACT_STOP                // 18
};

// Fixed actions to run at the beginning of selected stages (ACT_LINE_TRACK stages)
const int ENTRY_FIXED_ACTION_TYPE[19] = {
  FIX_ACT_NONE,
  FIX_ACT_NONE,            // 1
  FIX_ACT_NONE,            // 2
  FIX_ACT_NONE,            // 3
  FIX_ACT_TURN_LEFT_90,    // 4
  FIX_ACT_TURN_RIGHT_90,   // 5
  FIX_ACT_TURN_LEFT_90,    // 6
  FIX_ACT_NONE,            // 7 (handled by ACT_SPIN_360_RIGHT replacement)
  FIX_ACT_ROTATE_LEFT_90,  // 8
  FIX_ACT_NONE,            // 9
  FIX_ACT_NONE,            // 10
  FIX_ACT_TURN_LEFT_90,    // 11
  FIX_ACT_TURN_RIGHT_90,   // 12
  FIX_ACT_ROTATE_LEFT_90,  // 13
  FIX_ACT_ROTATE_LEFT_90,  // 14
  FIX_ACT_ROTATE_LEFT_90,  // 15
  FIX_ACT_ROTATE_LEFT_90,  // 16
  FIX_ACT_NONE,            // 17
  FIX_ACT_NONE             // 18
};

const int ENTRY_FIXED_ACTION_COUNT[19] = {
  0,
  0,  // 1
  0,  // 2
  0,  // 3
  2,  // 4  : 2 x turn left 90
  2,  // 5  : 2 x turn right 90
  1,  // 6  : 1 x turn left 90
  0,  // 7
  1,  // 8  : 1 x self-rotate left 90
  0,  // 9
  0,  // 10
  1,  // 11 : 1 x turn left 90
  1,  // 12 : 1 x turn right 90
  1,  // 13 : 1 x self-rotate left 90
  1,  // 14 : 1 x self-rotate left 90
  1,  // 15 : 1 x self-rotate left 90
  1,  // 16 : 1 x self-rotate left 90
  0,  // 17
  0   // 18
};

// Interval transition conditions derived from the map's distraction/junction points.
// Index N corresponds to interval [N -> N+1].
const int INTERVAL_TRANSITION_CONDITION[19] = {
  COND_TIME_ONLY,
  COND_TIME_ONLY,        // 1 (startup-handled)
  COND_TIME_ONLY,        // 2 (startup-handled)
  COND_JUNCTION_WHITE,   // 3 -> arrive 4
  COND_JUNCTION_WHITE,   // 4 -> arrive 5
  COND_JUNCTION_WHITE,   // 5 -> arrive 6
  COND_JUNCTION_WHITE,   // 6 -> arrive 7
  COND_TIME_ONLY,        // 7 (fixed 4x90 self-rotation)
  COND_JUNCTION_WHITE,   // 8 -> arrive 9
  COND_TIME_ONLY,        // 9
  COND_JUNCTION_WHITE,   // 10 -> arrive 11
  COND_JUNCTION_WHITE,   // 11 -> arrive 12
  COND_JUNCTION_WHITE,   // 12 -> arrive 13
  COND_JUNCTION_WHITE,   // 13 -> arrive 14
  COND_JUNCTION_WHITE,   // 14 -> arrive 15
  COND_JUNCTION_WHITE,   // 15 -> arrive 16
  COND_JUNCTION_WHITE,   // 16 -> arrive 17
  COND_TIME_ONLY,        // 17 -> arrive 18 (custom logic in runMissionMode)
  COND_TIME_ONLY         // 18
};

// ========================= PIN DEFINITIONS =========================

// assign meaningful names to those pins that will be used
const int pinL_Sensor = A5;      //pin A5: left tracking sensor
const int pinB_Sensor = A4;      //pin A4: bumper sensor
const int pinR_Sensor = A3;      //pin A3: right tracking sensor
const int pinC_Sensor = A2;      //pin A2: center tracking sensor

const int pinL_PWM = 9;          //pin D9: left motor speed
const int pinL_DIR = 10;         //pin D10: left motor direction

const int pinR_PWM = 11;         //pin D11: right motor speed
const int pinR_DIR = 12;         //pin D12: right motor direction

// ========================= VARIABLES =========================

int leftSensor = 1;    // 1 = dark, 0 = white
int bumperSensor = 1;  // 1 = dark, 0 = white (reserved for start/end marker)
int centerSensor = 1;  // 1 = dark, 0 = white
int rightSensor = 1;   // 1 = dark, 0 = white

int currentState = 0;   // active interval start point: interval [currentState -> currentState+1], starts at 3

// line-lost recovery memory: -1 = last correction to left, 1 = right, 0 = none
int lastTurn = 0;

// start gate: all modes stay stopped until bumper state toggles once from boot state
int bumperBootState = 1;
bool hasStarted = false;
bool startLineArmed = false;
unsigned long startLineSeenSinceMs = 0;

bool missionStateStarted = false;
unsigned long missionStateStartMs = 0;
int lastMissionState = 0;
bool stage17BumperTriggered = false;
bool debugStageStopActive = false;

int tuningStepIndex = 0;
unsigned long tuningStepStartMs = 0;
int tuningModeLatched = -1;

float clampPower(float p)
{
  if (p < 0.0) return 0.0;
  if (p > 1.0) return 1.0;
  return p;
}

int readBinaryStable(int pin)
{
  // majority vote over 3 reads for binary sensors (0/1)
  int s1 = digitalRead(pin);
  // int s2 = digitalRead(pin);
  // int s3 = digitalRead(pin);
  int sum = s1; // s1 + s2 + s3;
  return sum; // (sum >= 2) ? 1 : 0;
}

unsigned long scaledDurationMs(unsigned long baseMs)
{
  return (unsigned long)(baseMs * MISSION_TIME_MULTIPLIER + 0.5);
}

void refreshTrackingSensors()
{
  leftSensor = readBinaryStable(pinL_Sensor);
  centerSensor = readBinaryStable(pinC_Sensor);
  rightSensor = readBinaryStable(pinR_Sensor);
}

bool areAllTrackingSensorsOnWhite()
{
  return (leftSensor == 0 && centerSensor == 0 && rightSensor == 0);
}

void onMissionStateEnter(int state)
{
  if (DEBUG_STOP_AT_STAGE >= 3 && DEBUG_STOP_AT_STAGE <= 18 && state == DEBUG_STOP_AT_STAGE) {
    debugStageStopActive = true;
  }

  if (state == 17) {
    stage17BumperTriggered = false;
  }
}

float getMissionCruisePower(int intervalState)
{
  // Default interval speed is fast.
  // Requested exceptions:
  // - between 5 and 6   => interval state 5 => half
  // - between 9 and 10  => interval state 9 => half
  // - between 14 and 15 => interval state 14 => half
  if (intervalState == 5) return POWER_HALF;
  if (intervalState == 9) return POWER_HALF;
  if (intervalState == 14) return POWER_HALF;
  return POWER_FULL;
}

void setForwardDirection()
{
  digitalWrite(pinL_DIR, HIGH);
  digitalWrite(pinR_DIR, HIGH);
}

void setBackwardDirection()
{
  digitalWrite(pinL_DIR, LOW);
  digitalWrite(pinR_DIR, LOW);
}

void setWheelPower(float leftPower, float rightPower)
{
  // Each power is continuous in [0.0, 1.0].
  // Right side uses multiplier-scaled range.
  float leftCmd = clampPower(leftPower);
  float rightCmd = clampPower(rightPower);

  int left = (int)(leftCmd * MAX_LEFT_PWM + 0.5);
  int right = (int)(rightCmd * MAX_LEFT_PWM * RIGHT_PWM_MULTIPLIER + 0.5);

  if (left < 0) left = 0;
  if (left > MAX_LEFT_PWM) left = MAX_LEFT_PWM;
  if (right < 0) right = 0;
  if (right > 255) right = 255;

  analogWrite(pinL_PWM, left);
  analogWrite(pinR_PWM, right);
}

void setSteerLeftForCruise(float cruisePower)
{
  // Hard steering with bounded inside-wheel power.
  digitalWrite(pinL_DIR, LOW);
  digitalWrite(pinR_DIR, HIGH);
  setWheelPower(cruisePower <= POWER_HALF ? POWER_HALF : POWER_STOP, cruisePower);
}

void setSteerRightForCruise(float cruisePower)
{
  // Hard steering with bounded inside-wheel power.
  digitalWrite(pinL_DIR, HIGH);
  digitalWrite(pinR_DIR, LOW);
  setWheelPower(cruisePower, cruisePower <= POWER_HALF ? POWER_HALF : POWER_STOP);
}

unsigned long getFixedActionDurationMs(int actionType)
{
  if (actionType == FIX_ACT_TURN_LEFT_90) return FIX_TURN_LEFT_90_DURATION_MS;
  if (actionType == FIX_ACT_TURN_RIGHT_90) return FIX_TURN_RIGHT_90_DURATION_MS;
  if (actionType == FIX_ACT_ROTATE_LEFT_90) return FIX_ROTATE_LEFT_90_DURATION_MS;
  if (actionType == FIX_ACT_ROTATE_RIGHT_90) return FIX_ROTATE_RIGHT_90_DURATION_MS;
  return 0;
}

bool isSelfRotateActionType(int actionType)
{
  return (actionType == FIX_ACT_ROTATE_LEFT_90 || actionType == FIX_ACT_ROTATE_RIGHT_90);
}

void applyFixedActionCommand(int actionType)
{
  if (actionType == FIX_ACT_TURN_LEFT_90) {
    // Fixed-radius left arc: both forward, right wheel faster
    setForwardDirection();
    setWheelPower(FIX_TURN_LEFT_90_INNER_POWER, FIX_TURN_LEFT_90_OUTER_POWER);
    lastTurn = -1;
    return;
  }

  if (actionType == FIX_ACT_TURN_RIGHT_90) {
    // Fixed-radius right arc: both forward, left wheel faster
    setForwardDirection();
    setWheelPower(FIX_TURN_RIGHT_90_OUTER_POWER, FIX_TURN_RIGHT_90_INNER_POWER);
    lastTurn = 1;
    return;
  }

  if (actionType == FIX_ACT_ROTATE_LEFT_90) {
    // Self-rotation left 90: left backward, right forward
    digitalWrite(pinL_DIR, LOW);
    digitalWrite(pinR_DIR, HIGH);
    setWheelPower(FIX_ROTATE_LEFT_90_POWER, FIX_ROTATE_LEFT_90_POWER);
    lastTurn = -1;
    return;
  }

  if (actionType == FIX_ACT_ROTATE_RIGHT_90) {
    // Self-rotation right 90: left forward, right backward
    digitalWrite(pinL_DIR, HIGH);
    digitalWrite(pinR_DIR, LOW);
    setWheelPower(FIX_ROTATE_RIGHT_90_POWER, FIX_ROTATE_RIGHT_90_POWER);
    lastTurn = 1;
    return;
  }

  setForwardDirection();
  setWheelPower(POWER_STOP, POWER_STOP);
}

bool runFixedActionSequence(int actionType, int actionCount, unsigned long elapsedInState)
{
  if (actionType == FIX_ACT_NONE || actionCount <= 0) {
    return false;
  }

  unsigned long actionDur = scaledDurationMs(getFixedActionDurationMs(actionType));
  unsigned long preForwardDur = 0;
  if (isSelfRotateActionType(actionType)) {
    preForwardDur = scaledDurationMs(FIX_ROTATE_CENTERING_FORWARD_MS);
  }

  unsigned long totalDur = preForwardDur + actionDur * (unsigned long)actionCount;
  if (elapsedInState >= totalDur) {
    return false;
  }

  // Self-rotation centering correction (once before the sequence)
  if (preForwardDur > 0 && elapsedInState < preForwardDur) {
    setForwardDirection();
    setWheelPower(FIX_ROTATE_CENTERING_FORWARD_POWER, FIX_ROTATE_CENTERING_FORWARD_POWER);
    return true;
  }

  applyFixedActionCommand(actionType);
  return true;
}

void runFixedActionTuningMode(int firstActionType, int secondActionType)
{
  if (tuningModeLatched != RUN_MODE) {
    tuningModeLatched = RUN_MODE;
    tuningStepIndex = 0;
    tuningStepStartMs = millis();
  }

  int currentAction = (tuningStepIndex < TUNING_REPEAT_COUNT) ? firstActionType : secondActionType;
  unsigned long elapsed = millis() - tuningStepStartMs;

  if (!runFixedActionSequence(currentAction, 1, elapsed)) {
    tuningStepIndex = tuningStepIndex + 1;
    if (tuningStepIndex >= TUNING_REPEAT_COUNT * 2) {
      tuningStepIndex = 0;
    }
    tuningStepStartMs = millis();
  }
}

void runLineTrackSimple(float cruisePower)
{
  refreshTrackingSensors();

  int steerTurn = TURN_NONE;
  if (leftSensor == 0 && rightSensor == 1) {
    steerTurn = TURN_LEFT;
  }
  else if (leftSensor == 1 && rightSensor == 0) {
    steerTurn = TURN_RIGHT;
  }
  else if (centerSensor != 0) {
    // line-lost recovery direction when center is dark and no side preference
    if (lastTurn < 0) steerTurn = TURN_LEFT;
    else if (lastTurn > 0) steerTurn = TURN_RIGHT;
  }

  if (steerTurn == TURN_LEFT) {
    setSteerLeftForCruise(cruisePower);
    lastTurn = -1;
    return;
  }

  if (steerTurn == TURN_RIGHT) {
    setSteerRightForCruise(cruisePower);
    lastTurn = 1;
    return;
  }

  setForwardDirection();
  setWheelPower(cruisePower, cruisePower);
}

bool transitionConditionMet(int state)
{
  int cond = INTERVAL_TRANSITION_CONDITION[state];
  if (cond == COND_TIME_ONLY) {
    return true;
  }

  refreshTrackingSensors();
  bumperSensor = readBinaryStable(pinB_Sensor);
  if (cond == COND_CENTER_ON_WHITE) {
    return (centerSensor == 0);
  }
  if (cond == COND_JUNCTION_WHITE) {
    // requested: left and right sensors both white (junction)
    return (leftSensor == 0 && rightSensor == 0);
  }
  if (cond == COND_BUMPER_ON_WHITE) {
    return (bumperSensor == 0);
  }

  return true;
}

void runMissionMode()
{
  if (!missionStateStarted) {
    missionStateStarted = true;
    debugStageStopActive = false;
    currentState = DEBUG_START_AT_STAGE;
    missionStateStartMs = millis();
    lastMissionState = currentState;
    onMissionStateEnter(currentState);
  }

  if (currentState < 3 || currentState > 18) {
    currentState = 3;
    missionStateStartMs = millis();
    lastMissionState = currentState;
    onMissionStateEnter(currentState);
  }

  if (currentState != lastMissionState) {
    onMissionStateEnter(currentState);
    lastMissionState = currentState;
  }

  if (debugStageStopActive) {
    setForwardDirection();
    setWheelPower(POWER_STOP, POWER_STOP);
    return;
  }

  int action = STATE_ACTION[currentState];
  unsigned long elapsedInState = millis() - missionStateStartMs;

  if (action == ACT_STOP) {
    setForwardDirection();
    setWheelPower(POWER_STOP, POWER_STOP);
  }
  else if (action == ACT_LINE_TRACK) {
    float cruise = getMissionCruisePower(currentState);
    int entryActionType = ENTRY_FIXED_ACTION_TYPE[currentState];
    int entryActionCount = ENTRY_FIXED_ACTION_COUNT[currentState];

    // Execute fixed entry action(s) first; then continue normal line tracking.
    if (!runFixedActionSequence(entryActionType, entryActionCount, elapsedInState)) {
      runLineTrackSimple(cruise);
    }
  }
  else if (action == ACT_SPIN_360_RIGHT) {
    // Stage 7: replace 360 spin with 4 x 90-degree self-rotation (right)
    if (!runFixedActionSequence(FIX_ACT_ROTATE_RIGHT_90, STAGE7_ROTATE_90_COUNT, elapsedInState)) {
      setForwardDirection();
      setWheelPower(POWER_STOP, POWER_STOP);
    }
  }
  else if (action == ACT_BACKWARD_FAST) {
    // Stage 17 custom sequence:
    // - before bumper trigger: keep moving forward
    // - after bumper trigger: go backward until stage-18 white line is detected
    if (!stage17BumperTriggered) {
      setForwardDirection();
      runLineTrackSimple(POWER_FULL);
      bumperSensor = readBinaryStable(pinB_Sensor);
      if (bumperSensor == 0) {
        stage17BumperTriggered = true;
        delay(scaledDurationMs(STAGE17_BUMPER_DEBOUNCE_MS));
      }
    } else {
      setBackwardDirection();
      setWheelPower(POWER_FULL, POWER_FULL);
    }
  }
  else {
    setForwardDirection();
    setWheelPower(POWER_STOP, POWER_STOP);
  }

  // State transition logic based on relative duration from last stage change
  if (currentState >= 18) {
    return;
  }

  if (currentState == 17) {
    if (!stage17BumperTriggered) {
      return;
    }

    refreshTrackingSensors();
    if (areAllTrackingSensorsOnWhite()) {
      currentState = 18;
      missionStateStartMs = millis();
    }
    return;
  }

  unsigned long minDur = scaledDurationMs(INTERVAL_MIN_DURATION_MS[currentState]);
  unsigned long maxDur = scaledDurationMs(INTERVAL_MAX_DURATION_MS[currentState]);

  if (elapsedInState < minDur) {
    return;
  }

  bool shouldAdvance = transitionConditionMet(currentState);
  if (!shouldAdvance && elapsedInState >= maxDur) {
    shouldAdvance = true;
  }

  if (shouldAdvance) {
    currentState = currentState + 1;
    if (currentState > 18) {
      currentState = 18;
    }
    missionStateStartMs = millis();
  }
}

// the setup function runs once when you press reset or power the board

void setup ()
{
  // define pins as input and output
  pinMode(pinL_Sensor, INPUT);
  pinMode(pinB_Sensor, INPUT);
  pinMode(pinC_Sensor, INPUT);
  pinMode(pinR_Sensor, INPUT);
  
  pinMode(pinL_DIR, OUTPUT);
  pinMode(pinR_DIR, OUTPUT);
  
  pinMode(pinL_PWM, OUTPUT);
  pinMode(pinR_PWM, OUTPUT);
  
  // initialize output pins
  setForwardDirection();
  setWheelPower(0.0, 0.0);

  // record bumper state at boot; run begins only after this state toggles once
  bumperBootState = readBinaryStable(pinB_Sensor);
  delay(scaledDurationMs(BOOT_SETTLE_DELAY_MS));
}

// the loop function runs over and over again forever

void loop() {
  // startup sequence:
  // 1) place car so all three tracking sensors are on start white line (arm)
  // 2) toggle bumper once to start the task
  if (!hasStarted) {
    setWheelPower(0.0, 0.0);

    // Step 1: arm only when all three tracking sensors stably detect white
    if (!startLineArmed) {
      refreshTrackingSensors();
      if (areAllTrackingSensorsOnWhite()) {
        if (startLineSeenSinceMs == 0) {
          startLineSeenSinceMs = millis();
        }
        if ((millis() - startLineSeenSinceMs) >= scaledDurationMs(START_LINE_CONFIRM_MS)) {
          startLineArmed = true;
          // capture bumper baseline at arming moment
          bumperBootState = readBinaryStable(pinB_Sensor);
        }
      } else {
        startLineSeenSinceMs = 0;
      }
      return;
    }

    // Step 2: after armed, wait for bumper toggle to start
    bumperSensor = readBinaryStable(pinB_Sensor);
    if (bumperSensor != bumperBootState) {
      hasStarted = true;
      delay(scaledDurationMs(START_TRIGGER_DEBOUNCE_MS));  // debounce/settle after start trigger
    }
    return;
  }

  // mode 0: constant PWM test
  if (RUN_MODE == MODE_CONSTANT_PWM) {
    setForwardDirection();
    setWheelPower(CONSTANT_MODE_POWER, CONSTANT_MODE_POWER);
    return;
  }

  // mode 1: simple line tracking
  if (RUN_MODE == MODE_LINE_TRACK) {
    setForwardDirection();
    runLineTrackSimple(POWER_FULL);
    return;
  }

  // mode 2: mission mode for full project task
  if (RUN_MODE == MODE_MISSION_TASK) {
    runMissionMode();
    return;
  }

  // mode 3: tuning mode (turn 90 left x4, then turn 90 right x4, repeat)
  if (RUN_MODE == MODE_TUNE_TURN_90) {
    runFixedActionTuningMode(FIX_ACT_TURN_LEFT_90, FIX_ACT_TURN_RIGHT_90);
    return;
  }

  // mode 4: tuning mode (self-rotate left 90 x4, then self-rotate right 90 x4, repeat)
  if (RUN_MODE == MODE_TUNE_ROTATE_90) {
    runFixedActionTuningMode(FIX_ACT_ROTATE_LEFT_90, FIX_ACT_ROTATE_RIGHT_90);
    return;
  }

  // safety fallback
  setForwardDirection();
  setWheelPower(POWER_STOP, POWER_STOP);
}

