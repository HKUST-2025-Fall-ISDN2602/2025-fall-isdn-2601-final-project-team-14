#include <Servo.h>

// ==================== SERVO OBJECTS ====================
Servo baseServo;      // Z-axis rotation
Servo shoulderServo;  // First arm joint
Servo elbowServo;     // Second arm joint
Servo wristServo;     // Third arm joint
Servo gripperServo;   // Clamp open/close

// ==================== PIN ASSIGNMENTS ====================
// Change these to match your ESP/Arduino wiring
const int BASE_PIN     = 5;   // GPIO5
const int SHOULDER_PIN = 4;   // GPIO4
const int ELBOW_PIN    = 12;  // GPIO12
const int WRIST_PIN    = 14;  // GPIO14
const int GRIPPER_PIN  = 16;  // GPIO16

// ==================== ANGLE LIMITS (TUNE) ====================
// Adjust all these based on your physical arm limits
const int BASE_MIN     = 0;
const int BASE_MAX     = 180;

const int SHOULDER_MIN = 0;
const int SHOULDER_MAX = 180;

const int ELBOW_MIN    = 0;
const int ELBOW_MAX    = 180;

const int WRIST_MIN    = 0;
const int WRIST_MAX    = 180;

// Gripper open/close angles (tune these for your clamp)
const int GRIPPER_OPEN  = 0;    // "open" clamp angle
const int GRIPPER_CLOSE = 100;  // "closed" clamp angle


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
  180,           // base
  0,             // shoulder
  100,           // elbow
  0,             // wrist
  GRIPPER_OPEN   // gripper
};

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
  baseServo.write(    clampAngle(p.base,     BASE_MIN,     BASE_MAX) );
  shoulderServo.write(clampAngle(p.shoulder, SHOULDER_MIN, SHOULDER_MAX) );
  elbowServo.write(   clampAngle(p.elbow,    ELBOW_MIN,    ELBOW_MAX) );
  wristServo.write(   clampAngle(p.wrist,    WRIST_MIN,    WRIST_MAX) );
  // Clamp gripper too, just in case
  int g = clampAngle(p.gripper, GRIPPER_OPEN, GRIPPER_CLOSE);
  gripperServo.write(g);
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
    intermediate.base     = start.base     + (int)(deltaBase     * t);
    intermediate.shoulder = start.shoulder + (int)(deltaShoulder * t);
    intermediate.elbow    = start.elbow    + (int)(deltaElbow    * t);
    intermediate.wrist    = start.wrist    + (int)(deltaWrist    * t);
    intermediate.gripper  = start.gripper  + (int)(deltaGripper  * t);

    writePose(intermediate);
    delay(speedDelayMs);
  }

  // Snap to exact target at the end
  currentPose = target;
  writePose(currentPose);
}


// ==================== GRIPPER HELPERS (OPTIONAL) ====================
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


// ==================== RESET / HOME ====================
void resetArm() {
  Serial.println("Resetting to home...");
  moveToPose(homePose, 10, 3);  // tune speed here
  Serial.println("At home pose.");
}


// ==================== SERVO DEBUG MODE ====================
// Command format (type in Serial Monitor):
//   d base shoulder elbow wrist gripper
// Example:
//   d 90 80 120 45 10
//
// This directly sets all 5 joints.

void debugMove(String line) {
  line.trim();  // should contain just the 5 numbers

  int b, s, e, w, g;
  int parsed = sscanf(line.c_str(), "%d %d %d %d %d", &b, &s, &e, &w, &g);

  if (parsed != 5) {
    Serial.println("Format error! Use: d base shoulder elbow wrist gripper");
    Serial.println("Example: d 90 120 45 10 20");
    return;
  }

  Pose target = {
    b, // base
    s, // shoulder
    e, // elbow
    w, // wrist
    g  // gripper
  };

  Serial.println("DEBUG MOVE →");
  Serial.print("  Base: ");     Serial.println(b);
  Serial.print("  Shoulder: "); Serial.println(s);
  Serial.print("  Elbow: ");    Serial.println(e);
  Serial.print("  Wrist: ");    Serial.println(w);
  Serial.print("  Gripper: ");  Serial.println(g);

  // slightly faster & smooth
  moveToPose(target, 8, 2);
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
  Serial.println("Commands:");
  Serial.println("  r            = reset to home");
  Serial.println("  d a b c d e  = debug: set all 5 joints");
  Serial.println("    (base shoulder elbow wrist gripper)");
  Serial.println("Example:");
  Serial.println("  d 180 0 100 0 0");
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
      case 'r':
      case 'R':
        resetArm();
        break;

      case 'd':
      case 'D': {
        // read the rest of the line after 'd'
        String params = Serial.readStringUntil('\n');
        debugMove(params);
        break;
      }

      default:
        Serial.println("Unknown command.");
        Serial.println("Use:");
        Serial.println("  r");
        Serial.println("  d base shoulder elbow wrist gripper");
        break;
    }

    Serial.println("Ready for next command.");
  }
}