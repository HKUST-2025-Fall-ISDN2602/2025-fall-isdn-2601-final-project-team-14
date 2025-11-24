#include <Servo.h>

// ==================== SERVO OBJECTS ====================
Servo baseServo;      // Z-axis rotation
Servo shoulderServo;  // First arm joint
Servo elbowServo;     // Second arm joint
Servo wristServo;     // Third arm joint
Servo gripperServo;   // Clamp open/close

// ==================== PIN ASSIGNMENTS ====================
// Change these to match your ESP/Arduino wiring
const int BASE_PIN     = D1;   // GPIO5  (example)
const int SHOULDER_PIN = D2;   // GPIO4
const int ELBOW_PIN    = D3;   // GPIO0
const int WRIST_PIN    = D4;   // GPIO2
const int GRIPPER_PIN  = D5;   // GPIO14

// ==================== ANGLE LIMITS (TUNE) ====================
// Adjust all these based on your physical arm limits
const int BASE_MIN     = 10;
const int BASE_MAX     = 170;

const int SHOULDER_MIN = 20;
const int SHOULDER_MAX = 160;

const int ELBOW_MIN    = 10;
const int ELBOW_MAX    = 170;

const int WRIST_MIN    = 10;
const int WRIST_MAX    = 170;

// Gripper open/close angles (tune these for your clamp)
const int GRIPPER_OPEN  = 90;   // "open" clamp angle
const int GRIPPER_CLOSE = 40;   // "closed" clamp angle


// ==================== POSE STRUCT ====================
struct Pose {
  int base;
  int shoulder;
  int elbow;
  int wrist;
  int gripper;
};


// ==================== HELPER: MAX OF 5 INTS ====================
int max5(int a, int b, int c, int d, int e) {
  int m = a;
  if (b > m) m = b;
  if (c > m) m = c;
  if (d > m) m = d;
  if (e > m) m = e;
  return m;
}


// ==================== BASIC POSES ====================

// Safe home pose – tune these first!
Pose homePose = {
  90,   // base
  90,   // shoulder
  90,   // elbow
  90,   // wrist
  GRIPPER_OPEN
};

// ---------- POSES FOR TASK 1 (cube) ----------
// !!! PLACEHOLDER ANGLES – CHANGE AFTER TESTING !!!
Pose t1_aboveObj = { 60, 70, 110, 80, GRIPPER_OPEN };
Pose t1_grabObj  = { 60, 80, 120, 80, GRIPPER_OPEN };
Pose t1_liftObj  = { 60, 60, 100, 80, GRIPPER_CLOSE };
Pose t1_aboveBox = {100, 65, 105, 80, GRIPPER_CLOSE };
Pose t1_release  = {100, 65, 105, 80, GRIPPER_OPEN  };

// ---------- POSES FOR TASK 2 (cylinder) ----------
Pose t2_aboveObj = { 40, 75, 110, 80, GRIPPER_OPEN };
Pose t2_grabObj  = { 40, 85, 120, 80, GRIPPER_OPEN };
Pose t2_liftObj  = { 40, 65, 100, 80, GRIPPER_CLOSE };
Pose t2_aboveBox = {120, 60, 100, 80, GRIPPER_CLOSE };
Pose t2_release  = {120, 60, 100, 80, GRIPPER_OPEN  };

// ---------- POSES FOR TASK 3 (hat) ----------
Pose t3_aboveObj = { 70, 70, 110, 80, GRIPPER_OPEN };
Pose t3_grabObj  = { 70, 80, 120, 80, GRIPPER_OPEN };
Pose t3_liftObj  = { 70, 60, 100, 80, GRIPPER_CLOSE };
Pose t3_aboveBox = {140, 60, 100, 80, GRIPPER_CLOSE };
Pose t3_release  = {140, 60, 100, 80, GRIPPER_OPEN  };

// ---------- POSES FOR TASK 4 (boat) ----------
Pose t4_aboveObj = { 85, 70, 110, 80, GRIPPER_OPEN };
Pose t4_grabObj  = { 85, 80, 120, 80, GRIPPER_OPEN };
Pose t4_liftObj  = { 85, 60, 100, 80, GRIPPER_CLOSE };
Pose t4_aboveBox = {150, 60, 100, 80, GRIPPER_CLOSE };
Pose t4_release  = {150, 60, 100, 80, GRIPPER_OPEN  };


// ==================== GLOBAL STATE ====================
Pose currentPose = homePose;


// ==================== HELPER: CLAMP ANGLES ====================
int clampAngle(int angle, int minVal, int maxVal) {
  if (angle < minVal) return minVal;
  if (angle > maxVal) return maxVal;
  return angle;
}

// ==================== HELPER: WRITE ALL SERVOS ====================
void writePose(const Pose &p) {
  baseServo.write( clampAngle(p.base,     BASE_MIN,     BASE_MAX) );
  shoulderServo.write( clampAngle(p.shoulder, SHOULDER_MIN, SHOULDER_MAX) );
  elbowServo.write( clampAngle(p.elbow,   ELBOW_MIN,    ELBOW_MAX) );
  wristServo.write( clampAngle(p.wrist,   WRIST_MIN,    WRIST_MAX) );
  gripperServo.write(p.gripper); // you can also clamp this if needed
}


// ==================== HELPER: SMOOTH MOVE TO POSE ====================
/*
 * speedDelayMs:
 *   smaller  -> faster motion
 *   larger   -> slower, smoother
 *
 * stepSize (degrees per step):
 *   smaller  -> smoother, slower
 *   larger   -> choppier, faster
 */
void moveToPose(Pose target, int speedDelayMs = 10, int stepSize = 2) {
  Pose start = currentPose;

  int deltaBase     = target.base     - start.base;
  int deltaShoulder = target.shoulder - start.shoulder;
  int deltaElbow    = target.elbow    - start.elbow;
  int deltaWrist    = target.wrist    - start.wrist;
  int deltaGripper  = target.gripper  - start.gripper;

  int maxDelta = max5(
    abs(deltaBase),
    abs(deltaShoulder),
    abs(deltaElbow),
    abs(deltaWrist),
    abs(deltaGripper)
  );

  int steps = maxDelta / stepSize;
  if (steps < 1) steps = 1;

  for (int i = 1; i <= steps; i++) {
    float t = (float)i / (float)steps;  // 0 → 1

    Pose intermediate;
    intermediate.base     = start.base     + deltaBase     * t;
    intermediate.shoulder = start.shoulder + deltaShoulder * t;
    intermediate.elbow    = start.elbow    + deltaElbow    * t;
    intermediate.wrist    = start.wrist    + deltaWrist    * t;
    intermediate.gripper  = start.gripper  + deltaGripper  * t;

    writePose(intermediate);
    delay(speedDelayMs);
  }

  // Snap to exact target at the end
  currentPose = target;
  writePose(currentPose);
}


// ==================== GRIPPER HELPERS ====================
void openGripper(int speedDelayMs = 5) {
  Pose target = currentPose;
  target.gripper = GRIPPER_OPEN;
  moveToPose(target, speedDelayMs, 2);
}

void closeGripper(int speedDelayMs = 5) {
  Pose target = currentPose;
  target.gripper = GRIPPER_CLOSE;
  moveToPose(target, speedDelayMs, 2);
}


// ==================== HIGH-LEVEL MOVES / TASKS ====================

void resetArm() {
  Serial.println("Resetting to home...");
  moveToPose(homePose, 10, 3);  // tune speed here
}


// ---------- TASK 1: e.g. cube ----------
void task1() {
  Serial.println("Running Task 1 (cube)...");

  moveToPose(t1_aboveObj, 10, 3);   // go above object
  moveToPose(t1_grabObj,  15, 2);   // lower slowly
  closeGripper(10);                 // clamp
  moveToPose(t1_liftObj,  10, 2);   // lift
  moveToPose(t1_aboveBox, 10, 3);   // to box
  moveToPose(t1_release,  15, 2);   // lower into box
  openGripper(10);                  // release
  moveToPose(homePose,    10, 3);   // back home

  Serial.println("Task 1 done.");
}


// ---------- TASK 2: cylinder ----------
void task2() {
  Serial.println("Running Task 2 (cylinder)...");

  moveToPose(t2_aboveObj, 10, 3);
  moveToPose(t2_grabObj,  15, 2);
  closeGripper(10);
  moveToPose(t2_liftObj,  10, 2);
  moveToPose(t2_aboveBox, 10, 3);
  moveToPose(t2_release,  15, 2);
  openGripper(10);
  moveToPose(homePose,    10, 3);

  Serial.println("Task 2 done.");
}


// ---------- TASK 3: hat ----------
void task3() {
  Serial.println("Running Task 3 (hat)...");

  moveToPose(t3_aboveObj, 10, 3);
  moveToPose(t3_grabObj,  15, 2);
  closeGripper(10);
  moveToPose(t3_liftObj,  10, 2);
  moveToPose(t3_aboveBox, 10, 3);
  moveToPose(t3_release,  15, 2);
  openGripper(10);
  moveToPose(homePose,    10, 3);

  Serial.println("Task 3 done.");
}


// ---------- TASK 4: boat ----------
void task4() {
  Serial.println("Running Task 4 (boat)...");

  moveToPose(t4_aboveObj, 10, 3);
  moveToPose(t4_grabObj,  15, 2);
  closeGripper(10);
  moveToPose(t4_liftObj,  10, 2);
  moveToPose(t4_aboveBox, 10, 3);
  moveToPose(t4_release,  15, 2);
  openGripper(10);
  moveToPose(homePose,    10, 3);

  Serial.println("Task 4 done.");
}


// ==================== SETUP & LOOP ====================
void setup() {
  Serial.begin(115200);

  baseServo.attach(BASE_PIN);
  shoulderServo.attach(SHOULDER_PIN);
  elbowServo.attach(ELBOW_PIN);
  wristServo.attach(WRIST_PIN);
  gripperServo.attach(GRIPPER_PIN);

  // Move to home pose at startup
  writePose(homePose);
  delay(1000);

  Serial.println("Arm ready!");
  Serial.println("Type command then press ENTER:");
  Serial.println("1 = Task 1 (cube)");
  Serial.println("2 = Task 2 (cylinder)");
  Serial.println("3 = Task 3 (hat)");
  Serial.println("4 = Task 4 (boat)");
  Serial.println("r = reset to home");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    // ignore newline / carriage return
    if (cmd == '\n' || cmd == '\r') {
      return;
    }

    Serial.print("Received command: ");
    Serial.println(cmd);

    switch (cmd) {
      case '1':
        task1();
        break;
      case '2':
        task2();
        break;
      case '3':
        task3();
        break;
      case '4':
        task4();
        break;
      case 'r':
      case 'R':
        resetArm();
        break;
      default:
        Serial.println("Unknown command. Use 1,2,3,4,r.");
        break;
    }

    Serial.println("Ready for next command: 1,2,3,4,r");
  }
}