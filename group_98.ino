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

// Which mode to run. Change ONLY this constant to switch behavior.
const int RUN_MODE = MODE_MISSION_TASK; // SET TO 2 OR MODE_MISSION_TASK!!!

const int DEBUG_START_AT_STAGE = 9; // SET TO 3!!!!!
// Debug stage stop (mission mode only):
// Any value outside 3..18 = disabled
// N (3..18) = stop completely once stage N is entered
const int DEBUG_STOP_AT_STAGE = 18; // SET TO 19!!!

// Six effective power levels only: stop / quarter / half low / half / full / max
const float POWER_STOP = 0.0;
const float POWER_QUARTER = 0.35; // quarter
const float POWER_QUARTER_HIGH = 0.4; // quarter high
const float POWER_HALF_LOW = 0.5; // half low
const float POWER_HALF = 0.65;  // half
const float POWER_FULL = 0.8;  // full
const float POWER_MAX = 1.0; // max

// Constant test mode power (choose from POWER_* levels above)
const float CONSTANT_MODE_POWER = POWER_FULL;

// Right motor compensation multiplier (right is slower, so boost its PWM).
// Right PWM = left PWM * RIGHT_PWM_MULTIPLIER when both are commanded at same power.
const float RIGHT_PWM_MULTIPLIER = 1.054;

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
// Keep windows narrow: max - min <= 500 ms.
const unsigned long INTERVAL_MIN_DURATION_MS[19] = {
  0,
  0,   // 1 (startup-handled)
  0,   // 2 (startup-handled)
  1400,  // 3
  2300,  // 4
  4500,  // 5
  1400,  // 6
  950,  // 7 (self-rotation only)
  1200,  // 8 (post-self-rotation causes variation)
  9333,   // 9  (retuned after slow->half mapping)
  0,  // 10
  400,   // 11
  600,   // 12
  1000,   // 13
  3000,   // 14 (retuned after slow->half mapping)
  1000,   // 15
  1500,  // 16
  1500,  // 17
  100   // 18
};

const unsigned long INTERVAL_MAX_DURATION_MS[19] = {
  0,
  0,   // 1 (startup-handled)
  0,  // 2 (startup-handled)
  2000,  // 3
  3200,  // 4
  6000,  // 5
  2000,  // 6
  1050,  // 7 (self-rotation only)
  1800,  // 8 (post-self-rotation causes variation)
  11333,   // 9  (retuned after slow->half mapping)
  0,  // 10
  800,  // 11
  1100,  // 12
  2500,  // 13
  3750,   // 14 (retuned after slow->half mapping)
  2500,  // 15
  2000,  // 16
  2000,  // 17
  400   // 18
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
const int COND_JUNCTION_WHITE = 2;  // use left and right sensors only, ignore center (requested junction condition)
const int COND_BUMPER_ON_WHITE = 3;
const int COND_STAGE9_TO_11_WHITE_PATTERN = 4;    // special, empirically tuned stage-9->11 pattern
const int COND_STAGE14_ROTATE_360_LEFT_WHITE = 5; // stage 14 -> 15: ~360deg rotation and left sensor on white

// Stage-entry turn hint (used at the beginning of selected stages)
const int TURN_NONE = 0;
const int TURN_LEFT = -1;
const int TURN_RIGHT = 1;

// Time for entry turn bias at stage start (before normal line tracking resumes)
const unsigned long ENTRY_TURN_DURATION_SHORT_MS = 100;
const unsigned long ENTRY_TURN_DURATION_MS = 150;
const unsigned long ENTRY_TURN_DURATION_LONG_MS = 240;

// After forced stage-entry turn ends, temporarily force straight driving.
const unsigned long POST_FORCED_TURN_DISTRACT_BLOCK_MS = 100;
const unsigned long ENTRY_SENSOR_STEER_BLOCK_MS = 300;
const unsigned long ENTRY_SENSOR_STEER_BLOCK_LONG_MS = 500;
const unsigned long FAR_RIGHT_CORRECTION_POST_CENTER_MS = 75;

// Approximate heading tracker for stage 9.
// Positive heading = right, negative heading = left.
const unsigned long STAGE9_HEADING_ZERO_DELAY_AFTER_INITIAL_MS = 250;
const unsigned long APPROX_TURN_90_DEG_TIME_MS = 250;  // tuneable: ~time to rotate 90 deg while steering
const float APPROX_HEADING_MIN_DEG = -90.0;
const float APPROX_HEADING_MAX_DEG = 90.0;
const float STAGE9_TURN_DETECT_DEG = 80.0;             // crossing threshold around heading=0 for stage-9 L/R swing counting
const float HEADING_ROUGH_LEFT_THRESHOLD_DEG = 12.0;   // if <= -threshold, treat as still left
const float STAGE14_ROTATE_APPROX_DEG = 330.0;         // stage 14 completion threshold (approx 360)

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
  ACT_STOP,               // 10 (dummy stage, instant pass-through)
  ACT_LINE_TRACK,         // 11
  ACT_LINE_TRACK,         // 12
  ACT_LINE_TRACK,         // 13
  ACT_LINE_TRACK,         // 14
  ACT_LINE_TRACK,         // 15
  ACT_LINE_TRACK,         // 16
  ACT_BACKWARD_FAST,      // 17
  ACT_STOP                // 18
};

// Stage-beginning transition decision (from map):
// 4:L, 5:R, 6:L, 9:L, 11:R, 12:L, 13:L, 14:L, 15:R, 16:L
const int STAGE_ENTRY_TURN[19] = {
  TURN_NONE,
  TURN_NONE,   // 1
  TURN_NONE,   // 2
  TURN_NONE,   // 3
  TURN_LEFT,   // 4
  TURN_RIGHT,  // 5
  TURN_LEFT,   // 6
  TURN_NONE,   // 7 (dedicated 360 action)
  TURN_NONE,   // 8 (line tracking only)
  TURN_LEFT,   // 9 (begin with left turn)
  TURN_NONE,   // 10
  TURN_RIGHT,  // 11
  TURN_LEFT,   // 12
  TURN_LEFT,   // 13
  TURN_LEFT,   // 14
  TURN_RIGHT,  // 15
  TURN_LEFT,   // 16
  TURN_NONE,   // 17 (custom bumper/backward logic)
  TURN_NONE    // 18
};

// Interval transition conditions derived from the map's distraction/junction points.
// For COND_JUNCTION_WHITE, decision uses left+right both white (center ignored).
// Index N corresponds to interval [N -> N+1].
const int INTERVAL_TRANSITION_CONDITION[19] = {
  COND_TIME_ONLY,
  COND_TIME_ONLY,        // 1 (startup-handled)
  COND_TIME_ONLY,        // 2 (startup-handled)
  COND_JUNCTION_WHITE,   // 3 -> arrive 4
  COND_JUNCTION_WHITE,   // 4 -> arrive 5
  COND_JUNCTION_WHITE,   // 5 -> arrive 6
  COND_JUNCTION_WHITE,   // 6 -> arrive 7
  COND_JUNCTION_WHITE,   // 7 (self-rotation)
  COND_JUNCTION_WHITE,   // 8 -> arrive 9
  COND_STAGE9_TO_11_WHITE_PATTERN, // 9 -> (10 dummy) -> 11
  COND_TIME_ONLY,        // 10 -> arrive 11 (dummy stage, immediate)
  COND_JUNCTION_WHITE,   // 11 -> arrive 12
  COND_JUNCTION_WHITE,   // 12 -> arrive 13
  COND_JUNCTION_WHITE,   // 13 -> arrive 14
  COND_STAGE14_ROTATE_360_LEFT_WHITE, // 14 -> arrive 15
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
const int pinFR_Sensor = A1;     //pin A1: far-right tracking sensor (stage 9/14 special use)

const int pinL_PWM = 9;          //pin D9: left motor speed
const int pinL_DIR = 10;         //pin D10: left motor direction

const int pinR_PWM = 11;         //pin D11: right motor speed
const int pinR_DIR = 12;         //pin D12: right motor direction

// ========================= VARIABLES =========================

int leftSensor = 1;    // 1 = dark, 0 = white
int bumperSensor = 1;  // 1 = dark, 0 = white (reserved for start/end marker)
int centerSensor = 1;  // 1 = dark, 0 = white
int rightSensor = 1;   // 1 = dark, 0 = white
int farRightSensor = 1; // 1 = dark, 0 = white (used only in stage 9/14 rule)

bool farRightCorrectionActive = false;
int farRightCorrectionTurn = TURN_NONE;
bool farRightCorrectionCenterSeen = false;
unsigned long farRightCorrectionPostCenterStartMs = 0;

int currentState = 0;   // mission interval index [currentState -> currentState+1]; set to 3 when mission starts

// line-lost recovery memory: -1 = last correction to left, 1 = right, 0 = none
int lastTurn = 0;

// start gate: all modes stay stopped until start-line arming is done, then bumper toggles from baseline
int bumperBootState = 1;
bool hasStarted = false;
bool startLineArmed = false;
unsigned long startLineSeenSinceMs = 0;

// Require stable placement on start white line before accepting bumper toggle
const unsigned long START_LINE_CONFIRM_MS = 120;

bool missionStateStarted = false;
unsigned long missionStateStartMs = 0;
int lastMissionState = 0;
bool stage17BumperTriggered = false;
bool debugStageStopActive = false;

float approxHeadingDeg = 0.0;
bool stage9HeadingZeroed = false;
int stage9HeadingTurnCount = 0; // counts detected swings in sequence: L, R, L, R, L (target = 5)
unsigned long headingLastUpdateMs = 0;
bool headingSteeringBlockActive = false;
int headingSteeringBlockTurn = TURN_NONE; // TURN_LEFT blocks left steering, TURN_RIGHT blocks right steering
bool stage14HeadingTrackingActive = false;
float stage14AccumulatedTurnDeg = 0.0;

float clampPower(float p)
{
  if (p < 0.0) return 0.0;
  if (p > 1.0) return 1.0;
  return p;
}

float clampHeadingDeg(float h)
{
  if (h < APPROX_HEADING_MIN_DEG) return APPROX_HEADING_MIN_DEG;
  if (h > APPROX_HEADING_MAX_DEG) return APPROX_HEADING_MAX_DEG;
  return h;
}

void clearHeadingSteeringBlock()
{
  headingSteeringBlockActive = false;
  headingSteeringBlockTurn = TURN_NONE;
}

void resetStage9HeadingTracking()
{
  approxHeadingDeg = 0.0;
  stage9HeadingZeroed = false;
  stage9HeadingTurnCount = 0;
  headingLastUpdateMs = millis();
  clearHeadingSteeringBlock();
}

void resetStage14HeadingTracking()
{
  stage14HeadingTrackingActive = false;
  stage14AccumulatedTurnDeg = 0.0;
}

float getApproxTurnDeltaDeg(int steeringTurn, unsigned long dtMs)
{
  if ((steeringTurn != TURN_LEFT && steeringTurn != TURN_RIGHT) || dtMs == 0) {
    return 0.0;
  }

  unsigned long turn90Ms = scaledDurationMs(APPROX_TURN_90_DEG_TIME_MS);
  if (turn90Ms == 0) turn90Ms = 1;

  float degPerMs = 90.0 / (float)turn90Ms;
  float delta = degPerMs * (float)dtMs;
  return (steeringTurn == TURN_LEFT) ? -delta : delta;
}

void updateStage9ApproxHeading(int steeringTurn)
{
  if (currentState != 9 || !stage9HeadingZeroed || headingSteeringBlockActive) {
    return;
  }

  unsigned long nowMs = millis();
  if (headingLastUpdateMs == 0) {
    headingLastUpdateMs = nowMs;
    return;
  }

  unsigned long dtMs = nowMs - headingLastUpdateMs;
  headingLastUpdateMs = nowMs;

  float prevHeading = approxHeadingDeg;
  float delta = getApproxTurnDeltaDeg(steeringTurn, dtMs);
  if (delta != 0.0) {
    approxHeadingDeg = clampHeadingDeg(approxHeadingDeg + delta);
  }

  if (stage9HeadingTurnCount < 5) {
    // Alternate threshold crossing relative to heading=0: left, right, left, right, left.
    int expectedTurn = (stage9HeadingTurnCount % 2 == 0) ? TURN_LEFT : TURN_RIGHT;
    bool crossedLeft = (prevHeading > -STAGE9_TURN_DETECT_DEG && approxHeadingDeg <= -STAGE9_TURN_DETECT_DEG);
    bool crossedRight = (prevHeading < STAGE9_TURN_DETECT_DEG && approxHeadingDeg >= STAGE9_TURN_DETECT_DEG);

    if (expectedTurn == TURN_LEFT && crossedLeft) {
      stage9HeadingTurnCount = stage9HeadingTurnCount + 1;
    }
    else if (expectedTurn == TURN_RIGHT && crossedRight) {
      stage9HeadingTurnCount = stage9HeadingTurnCount + 1;
    }

    if (stage9HeadingTurnCount >= 5) {
      headingSteeringBlockActive = true;
      // After final left: if still roughly left, block left; otherwise block right.
      // This avoids further right-turn drifting when heading is not left anymore.
      headingSteeringBlockTurn = (approxHeadingDeg <= -HEADING_ROUGH_LEFT_THRESHOLD_DEG) ? TURN_LEFT : TURN_RIGHT;
    }
  }
}

void updateStage14ApproxHeading(int steeringTurn)
{
  if (currentState != 14 || !stage14HeadingTrackingActive) {
    return;
  }

  unsigned long nowMs = millis();
  if (headingLastUpdateMs == 0) {
    headingLastUpdateMs = nowMs;
    return;
  }

  unsigned long dtMs = nowMs - headingLastUpdateMs;
  headingLastUpdateMs = nowMs;

  float delta = getApproxTurnDeltaDeg(steeringTurn, dtMs);
  if (delta < 0.0) delta = -delta;
  stage14AccumulatedTurnDeg = stage14AccumulatedTurnDeg + delta;
}

int readBinaryStable(int pin)
{
  // majority vote over 3 reads for binary sensors (0/1)
  int s1 = digitalRead(pin);
  int s2 = digitalRead(pin);
  int s3 = digitalRead(pin);
  int sum = s1 + s2 + s3;
  return (sum >= 2) ? 1 : 0;
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
  farRightSensor = readBinaryStable(pinFR_Sensor);
}

bool areAllTrackingSensorsOnWhite()
{
  return (leftSensor == 0 && centerSensor == 0 && rightSensor == 0);
}

void clearFarRightCorrection()
{
  farRightCorrectionActive = false;
  farRightCorrectionTurn = TURN_NONE;
  farRightCorrectionCenterSeen = false;
  farRightCorrectionPostCenterStartMs = 0;
}

void startFarRightCorrection(int turnDir)
{
  farRightCorrectionActive = true;
  farRightCorrectionTurn = turnDir;
  farRightCorrectionCenterSeen = (centerSensor == 0);
  farRightCorrectionPostCenterStartMs = farRightCorrectionCenterSeen ? millis() : 0;
}

void onMissionStateEnter(int state)
{
  if (DEBUG_STOP_AT_STAGE >= 3 && DEBUG_STOP_AT_STAGE <= 18 && state == DEBUG_STOP_AT_STAGE) {
    debugStageStopActive = true;
  }

  if (state == 17) {
    stage17BumperTriggered = false;
  }

  if (state == 9) {
    resetStage14HeadingTracking();
    resetStage9HeadingTracking();
  }
  else if (state == 11) {
    // Keep heading steering block (if armed in stage 9) for all of stage 11.
    resetStage14HeadingTracking();
    headingLastUpdateMs = millis();
  }
  else if (state == 14) {
    approxHeadingDeg = 0.0;
    stage9HeadingZeroed = false;
    stage9HeadingTurnCount = 0;
    clearHeadingSteeringBlock();
    resetStage14HeadingTracking();
    headingLastUpdateMs = 0;
  }
  else if (state != 10) {
    // Stage 10 is a dummy pass-through. Clear heading-related effects once stage 11 is finished.
    approxHeadingDeg = 0.0;
    stage9HeadingZeroed = false;
    stage9HeadingTurnCount = 0;
    headingLastUpdateMs = 0;
    clearHeadingSteeringBlock();
    resetStage14HeadingTracking();
  }

  clearFarRightCorrection();
}

float getMissionCruisePower(int intervalState)
{
  // Default interval speed is fast.
  // Requested exceptions:
  // - between 5 and 6  => interval state 5 => half
  // - between 9 and 10 => interval state 9 => half low
  // - between 14 and 15 => interval state 14 => half
  if (intervalState == 5) return POWER_HALF;
  if (intervalState == 9) return POWER_HALF;
  // if (intervalState == 11) return POWER_HALF;
  // if (intervalState == 12) return POWER_HALF;
  if (intervalState == 14) return POWER_HALF;
  return POWER_FULL;
}

unsigned long getEntryTurnDurationMs(int state)
{
  // Keep 100 ms for stage 11.
  if (state == 11) {
    return ENTRY_TURN_DURATION_SHORT_MS;
  }

  // Keep 150 ms for stages 4, 5, 6, 12.
  if (state == 4 || state == 5 || state == 6 || state == 12) {
    return ENTRY_TURN_DURATION_MS;
  }

  // Remaining stages that need stage-entry turning use longer turning.
  return ENTRY_TURN_DURATION_LONG_MS;
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
  // Each power is a normalized command in [0.0, 1.0].
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

void applyDirectionalCommandWithStability(int leftDir, int rightDir, bool stabilize)
{
  if (stabilize) {
    // Stability trick: precharge both direction lines HIGH, then apply target directions.
    digitalWrite(pinL_DIR, HIGH);
    digitalWrite(pinR_DIR, HIGH);
    digitalWrite(pinL_DIR, leftDir);
    digitalWrite(pinR_DIR, rightDir);
  }
  else {
    // No stability mode: apply target directions twice.
    digitalWrite(pinL_DIR, leftDir);
    digitalWrite(pinR_DIR, rightDir);
    digitalWrite(pinL_DIR, leftDir);
    digitalWrite(pinR_DIR, rightDir);
  }
}

void setSteerLeftForCruise(float cruisePower, bool stabilize)
{
  // Steering rule:
  // - cruise >= POWER_FULL: left wheel backward uses POWER_QUARTER.
  // - cruise >= POWER_HALF_LOW (but < POWER_FULL): left wheel backward uses POWER_MAX.
  // - otherwise: left wheel backward stays stopped.
  float backwardPower = (cruisePower >= POWER_FULL) ? POWER_QUARTER : (cruisePower >= POWER_HALF_LOW) ? POWER_HALF_LOW : POWER_STOP;

  applyDirectionalCommandWithStability(LOW, HIGH, stabilize);
  setWheelPower(backwardPower, cruisePower);
}

void setSteerRightForCruise(float cruisePower, bool stabilize)
{
  // Steering rule:
  // - cruise >= POWER_FULL: right wheel backward uses POWER_QUARTER.
  // - cruise >= POWER_HALF_LOW (but < POWER_FULL): right wheel backward uses POWER_MAX.
  // - otherwise: right wheel backward stays stopped.
  float backwardPower = (cruisePower >= POWER_FULL) ? POWER_QUARTER : (cruisePower >= POWER_HALF_LOW) ? POWER_HALF_LOW : POWER_STOP;

  applyDirectionalCommandWithStability(HIGH, LOW, stabilize);
  setWheelPower(cruisePower, backwardPower);
}

void setEntryTurnLeftAggressive(bool stabilize)
{
  // Aggressive stage-entry pivot for explicit decision points:
  // left wheel backward + right wheel forward
  applyDirectionalCommandWithStability(LOW, HIGH, false);
  setWheelPower(POWER_FULL, POWER_MAX);
}

void setEntryTurnRightAggressive(bool stabilize)
{
  // Aggressive stage-entry pivot for explicit decision points:
  // left wheel forward + right wheel backward
  applyDirectionalCommandWithStability(HIGH, LOW, false);
  setWheelPower(POWER_MAX, POWER_FULL);
}

int runLineTrackSimple(float cruisePower, int forcedTurn, bool forceStraightAfterTurn, bool stabilize, bool blockLeftSensorForSteering, bool blockRightSensorForSteering)
{
  refreshTrackingSensors();

  // Entry steering-bias method: block selected side sensor(s) for steering decisions only.
  int steerLeftSensor = blockLeftSensorForSteering ? 1 : leftSensor;
  int steerRightSensor = blockRightSensorForSteering ? 1 : rightSensor;

  // Optional forced entry-turn for mission decision points.
  // This keeps mode1 and mode2 on the same tracking function.
  if (forcedTurn == TURN_LEFT) {
    clearFarRightCorrection();
    setEntryTurnLeftAggressive(stabilize);
    lastTurn = -1;
    return TURN_LEFT;
  }
  if (forcedTurn == TURN_RIGHT) {
    clearFarRightCorrection();
    setEntryTurnRightAggressive(stabilize);
    lastTurn = 1;
    return TURN_RIGHT;
  }

  // After forced turn ends, keep going straight for a short window.
  if (forceStraightAfterTurn) {
    clearFarRightCorrection();
    setForwardDirection();
    setForwardDirection();
    setWheelPower(cruisePower, cruisePower);
    return TURN_NONE;
  }

  bool inFarRightStages = (currentState == 9 || currentState == 14);
  if (!inFarRightStages) {
    clearFarRightCorrection();
  }

  // Far-right sensor override is used only in stage 9 and 14.
  // Rule:
  // - if far-right sensor sees white, turn right
  // - else if far-right is not white and no main tracking sensor sees white, turn left
  // - otherwise ignore far-right and continue normal tracking logic
  if (inFarRightStages) {
    if (farRightCorrectionActive) {
      if (!farRightCorrectionCenterSeen) {
        if (centerSensor == 0) {
          farRightCorrectionCenterSeen = true;
          farRightCorrectionPostCenterStartMs = millis();
        }
      }
      else {
        if ((millis() - farRightCorrectionPostCenterStartMs) >= FAR_RIGHT_CORRECTION_POST_CENTER_MS) {
          clearFarRightCorrection();
        }
      }

      if (farRightCorrectionActive) {
        if (farRightCorrectionTurn == TURN_RIGHT) {
          setSteerRightForCruise(cruisePower, stabilize);
          lastTurn = 1;
          return TURN_RIGHT;
        }
        else {
          setSteerLeftForCruise(cruisePower, stabilize);
          lastTurn = -1;
          return TURN_LEFT;
        }
      }
    }

    if (farRightSensor == 0) {
      startFarRightCorrection(TURN_RIGHT);
      setSteerRightForCruise(cruisePower, stabilize);
      lastTurn = 1;
      return TURN_RIGHT;
    }
    if (leftSensor == 1 && centerSensor == 1 && rightSensor == 1) {
      startFarRightCorrection(TURN_LEFT);
      setSteerLeftForCruise(cruisePower, stabilize);
      lastTurn = -1;
      return TURN_LEFT;
    }
  }

  // 0 = white line, 1 = dark background
  if (centerSensor == 0) {
    if (steerLeftSensor == 0 && steerRightSensor == 1) {
      setSteerLeftForCruise(cruisePower, stabilize);
      lastTurn = -1;
      return TURN_LEFT;
    }
    else if (steerLeftSensor == 1 && steerRightSensor == 0) {
      setSteerRightForCruise(cruisePower, stabilize);
      lastTurn = 1;
      return TURN_RIGHT;
    }
    else {
      setForwardDirection();
      setForwardDirection();
      setWheelPower(cruisePower, cruisePower);
      // keep lastTurn memory while centered to avoid introducing turn bias
      return TURN_NONE;
    }
  }
  else {
    if (steerLeftSensor == 0 && steerRightSensor == 1) {
      setSteerLeftForCruise(cruisePower, stabilize);
      lastTurn = -1;
      return TURN_LEFT;
    }
    else if (steerLeftSensor == 1 && steerRightSensor == 0) {
      setSteerRightForCruise(cruisePower, stabilize);
      lastTurn = 1;
      return TURN_RIGHT;
    }
    else {
      // search by last known direction using cruise-dependent steering rule
      // if lastTurn is unknown (0), do neutral forward probing first
      if (lastTurn < 0) {
        setSteerLeftForCruise(cruisePower, stabilize);
        return TURN_LEFT;
      }
      else if (lastTurn > 0) {
        setSteerRightForCruise(cruisePower, stabilize);
        return TURN_RIGHT;
      }
      else {
        // startup/unknown case: avoid hard left bias, keep tracking forward
        setForwardDirection();
        setForwardDirection();
        setWheelPower(cruisePower, cruisePower);
        return TURN_NONE;
      }
    }
  }
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
    return (leftSensor == 0 && rightSensor == 0) || (leftSensor == 0 && farRightSensor == 0);
  }
  if (cond == COND_BUMPER_ON_WHITE) {
    return (bumperSensor == 0);
  }
  if (cond == COND_STAGE9_TO_11_WHITE_PATTERN) {
    // Empirical stage-9->11 pattern (kept as requested):
    // (left+right white) OR ((left white OR center dark) AND far-right white)
    return (leftSensor == 0 && rightSensor == 0) || ((leftSensor == 0 || centerSensor == 1) && farRightSensor == 0);
  }
  if (cond == COND_STAGE14_ROTATE_360_LEFT_WHITE) {
    return stage14HeadingTrackingActive && stage14AccumulatedTurnDeg >= STAGE14_ROTATE_APPROX_DEG && leftSensor == 0;
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
    int entryTurn = STAGE_ENTRY_TURN[currentState];
    bool useEntrySensorBlockMethod = (currentState == 11 || currentState == 12);

    unsigned long entryTurnDur = scaledDurationMs(getEntryTurnDurationMs(currentState));
    unsigned long distractBlockDur = scaledDurationMs(POST_FORCED_TURN_DISTRACT_BLOCK_MS);

    if (currentState == 9 && !stage9HeadingZeroed) {
      // Start heading reference only after stage-9 initial actions complete,
      // then wait an extra delay before zeroing heading.
      unsigned long zeroDelayMs = scaledDurationMs(STAGE9_HEADING_ZERO_DELAY_AFTER_INITIAL_MS);
      unsigned long zeroAtMs = entryTurnDur + distractBlockDur + zeroDelayMs;
      if (elapsedInState >= zeroAtMs) {
        approxHeadingDeg = 0.0;
        stage9HeadingZeroed = true;
        headingLastUpdateMs = millis();
      }
    }

    if (currentState == 14 && !stage14HeadingTrackingActive) {
      // Start stage-14 heading tracking right after initial entry actions are done.
      unsigned long startAtMs = entryTurnDur + distractBlockDur;
      if (elapsedInState >= startAtMs) {
        stage14HeadingTrackingActive = true;
        stage14AccumulatedTurnDeg = 0.0;
        headingLastUpdateMs = millis();
      }
    }

    bool blockLeftSensorForSteering = false;
    bool blockRightSensorForSteering = false;
    if (useEntrySensorBlockMethod
    && ((currentState == 11 && elapsedInState < scaledDurationMs(ENTRY_SENSOR_STEER_BLOCK_MS))
    || (currentState == 12 && elapsedInState < scaledDurationMs(ENTRY_SENSOR_STEER_BLOCK_LONG_MS)))) {
      if (entryTurn == TURN_LEFT) {
        // force-left by blocking right sensor from steering logic
        blockRightSensorForSteering = true;
      }
      else if (entryTurn == TURN_RIGHT) {
        // force-right by blocking left sensor from steering logic
        blockLeftSensorForSteering = true;
      }
    }

    if (useEntrySensorBlockMethod) {
      // requested for transitions into 11 and 12: no aggressive forced-turn pulse.
      entryTurn = TURN_NONE;
    }

    if (headingSteeringBlockActive && (currentState == 9 || currentState == 11)) {
      if (headingSteeringBlockTurn == TURN_LEFT) {
        blockLeftSensorForSteering = true;
      }
      else if (headingSteeringBlockTurn == TURN_RIGHT) {
        blockRightSensorForSteering = true;
      }
    }

    bool stabilize = (elapsedInState < entryTurnDur || (currentState != 5 && currentState != 9 && currentState != 14));

    int forcedTurn = TURN_NONE;
    if (entryTurn != TURN_NONE && elapsedInState < entryTurnDur) {
      forcedTurn = entryTurn;
    }

    bool forceStraightAfterTurn = (entryTurn != TURN_NONE && elapsedInState >= entryTurnDur && elapsedInState < (entryTurnDur + distractBlockDur));

    int steeringTurn = runLineTrackSimple(cruise, forcedTurn, forceStraightAfterTurn, stabilize, blockLeftSensorForSteering, blockRightSensorForSteering);
    updateStage9ApproxHeading(steeringTurn);
    updateStage14ApproxHeading(steeringTurn);
  }
  else if (action == ACT_SPIN_360_RIGHT) {
    digitalWrite(pinL_DIR, HIGH);
    digitalWrite(pinR_DIR, LOW);
    setWheelPower(POWER_MAX, POWER_MAX);   // rotation always full
    lastTurn = 1;
  }
  else if (action == ACT_BACKWARD_FAST) {
    // Stage 17 custom sequence:
    // - before bumper trigger: keep line-tracking forward
    // - after bumper trigger: go backward until stage-18 white line is detected
    if (!stage17BumperTriggered) {
      setForwardDirection();
      (void)runLineTrackSimple(POWER_FULL, TURN_NONE, false, true, false, false);
      bumperSensor = readBinaryStable(pinB_Sensor);
      if (bumperSensor == 0) {
        stage17BumperTriggered = true;
        delay(80);
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
  bool allowForcedAdvanceAtMax = (INTERVAL_TRANSITION_CONDITION[currentState] != COND_STAGE14_ROTATE_360_LEFT_WHITE);
  if (!shouldAdvance && elapsedInState >= maxDur && allowForcedAdvanceAtMax) {
    shouldAdvance = true;
  }

  if (shouldAdvance) {
    currentState = currentState + 1;
    if (currentState > 18) {
      currentState = 18;
    }

    // Cleanly skip zero-time dummy states (e.g., stage 10) without executing them.
    while (currentState < 18 &&
           INTERVAL_MIN_DURATION_MS[currentState] == 0 &&
           INTERVAL_MAX_DURATION_MS[currentState] == 0 &&
           INTERVAL_TRANSITION_CONDITION[currentState] == COND_TIME_ONLY &&
           STATE_ACTION[currentState] == ACT_STOP) {
      currentState = currentState + 1;
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
  pinMode(pinFR_Sensor, INPUT);
  
  pinMode(pinL_DIR, OUTPUT);
  pinMode(pinR_DIR, OUTPUT);
  
  pinMode(pinL_PWM, OUTPUT);
  pinMode(pinR_PWM, OUTPUT);
  
  // initialize output pins
  setForwardDirection();
  setWheelPower(0.0, 0.0);

  // record bumper baseline at boot (later compared after start-line arming)
  bumperBootState = readBinaryStable(pinB_Sensor);
  delay(300);
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
        if ((millis() - startLineSeenSinceMs) >= START_LINE_CONFIRM_MS) {
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
      delay(120);  // debounce/settle after start trigger
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
    (void)runLineTrackSimple(POWER_FULL, TURN_NONE, false, true, false, false);
    return;
  }

  // mode 2: mission mode for full project task
  if (RUN_MODE == MODE_MISSION_TASK) {
    runMissionMode();
    return;
  }

  // safety fallback
  setForwardDirection();
  setWheelPower(POWER_STOP, POWER_STOP);
}
