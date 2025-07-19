#include <Servo.h>

// === CONFIGURATION ===
constexpr int NUM_JOINTS = 6;
constexpr int SERVO_PINS[NUM_JOINTS] = { 3, 5, 6, 9, 10, 11 };  // Base, Shoulder, Elbow, Wrist Roll, Wrist Pitch, Gripper

constexpr float MIN_ANGLE = 0.0f;
constexpr float MAX_ANGLE = 180.0f;
constexpr float MAX_GRIPPER_ANGLE = 75.0f;

constexpr float SERVO_STEP = 1.0f;  // smaller step = smoother, but slower
constexpr int UPDATE_INTERVAL_MS = 4;
unsigned long lastUpdateTime = 0;


// === STRUCTURES ===
struct JointAngles {
  float joint[NUM_JOINTS];
};

// === GLOBALS ===
Servo servos[NUM_JOINTS];
JointAngles target_angles = { { 90.0, 90.0, 90.0, 90.0, 90.0, 0.0 } };
JointAngles measured_angles;

// === FUNCTION DECLARATIONS ===
void initializeServos();
void updateServoPositions();
void updateMeasuredAngles();
void readSerialInput();
bool parseAngles(const String& msg, float* out_vals);
float clamp(float val, float min_val, float max_val);
void printJointStatus();

void setup() {
  Serial.begin(115200);
  initializeServos();
}

void loop() {
  readSerialInput();
  updateMeasuredAngles();
  updateServoPositions();
}

// === INITIALIZATION ===
void initializeServos() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    servos[i].attach(SERVO_PINS[i]);
  }
}

// === CONTROL ===
void updateServoPositions() {
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime < UPDATE_INTERVAL_MS) return;  // pacing
  lastUpdateTime = currentTime;

  for (int i = 0; i < NUM_JOINTS; ++i) {
    float max_limit = (i == 5) ? MAX_GRIPPER_ANGLE : MAX_ANGLE;
    target_angles.joint[i] = clamp(target_angles.joint[i], MIN_ANGLE, max_limit);

    float current = measured_angles.joint[i];
    float target = target_angles.joint[i];

    // Smooth step
    if (abs(current - target) > SERVO_STEP) {
      float direction = (target > current) ? 1.0f : -1.0f;
      current += direction * SERVO_STEP;
    } else {
      current = target;
    }

    servos[i].write(current);
    measured_angles.joint[i] = current;  // keep internal state synced
  }
}

void updateMeasuredAngles() {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    measured_angles.joint[i] = servos[i].read();
  }
}

// === SERIAL INPUT ===
void readSerialInput() {
  static String buffer;

  while (Serial.available()) {
    char incoming = Serial.read();

    if (incoming == '\n') {
      buffer.trim();
      buffer.replace("\r", "");

      float vals[NUM_JOINTS];
      if (parseAngles(buffer, vals)) {
        for (int i = 0; i < NUM_JOINTS; ++i) {
          float max_limit = (i == 5) ? MAX_GRIPPER_ANGLE : MAX_ANGLE;
          target_angles.joint[i] = clamp(vals[i], MIN_ANGLE, max_limit);
        }
      } else {
        Serial.println(F("Error: Invalid input. Expected format: x,x,x,x,x,x"));
      }
      buffer = "";
    } else {
      buffer += incoming;
    }
  }
}

// === HELPERS ===
bool parseAngles(const String& msg, float* out_vals) {
  int indices[NUM_JOINTS - 1];
  int start_idx = 0;

  for (int i = 0; i < NUM_JOINTS - 1; ++i) {
    indices[i] = msg.indexOf(',', start_idx);
    if (indices[i] == -1) return false;
    start_idx = indices[i] + 1;
  }

  out_vals[0] = msg.substring(0, indices[0]).toFloat();
  for (int i = 1; i < NUM_JOINTS - 1; ++i) {
    out_vals[i] = msg.substring(indices[i - 1] + 1, indices[i]).toFloat();
  }
  out_vals[NUM_JOINTS - 1] = msg.substring(indices[NUM_JOINTS - 2] + 1).toFloat();

  return true;
}

float clamp(float val, float min_val, float max_val) {
  return (val < min_val) ? min_val : (val > max_val) ? max_val
                                                     : val;
}

void printJointStatus() {
  Serial.print(F("Target [deg]  : "));
  for (int i = 0; i < NUM_JOINTS; ++i) {
    Serial.print(target_angles.joint[i]);
    if (i < NUM_JOINTS - 1) Serial.print(", ");
  }
  Serial.println();

  Serial.print(F("Measured [deg]: "));
  for (int i = 0; i < NUM_JOINTS; ++i) {
    Serial.print(measured_angles.joint[i]);
    if (i < NUM_JOINTS - 1) Serial.print(", ");
  }
  Serial.println();
}
