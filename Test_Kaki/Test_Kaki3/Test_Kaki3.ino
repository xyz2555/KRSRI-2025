#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  125
#define SERVOMAX  625

// Link lengths
double a1 = 6;
double a2 = 8;

// Servo configuration
const int servoOffset = 90;  // Offset to align with 0 degrees
const int servoMin = 0;      // Minimum angle
const int servoMax = 180;    // Maximum angle

// Leg definitions
enum LegID {
  FRONT_LEFT = 0,
  FRONT_RIGHT = 1,
  BACK_LEFT = 2,
  BACK_RIGHT = 3
};

// Servo channel assignments for each leg
// Each leg has 3 servos: base rotation, shoulder, elbow
const int legServos[4][3] = {
  {8, 10, 11},   // Front Left:  channels 8, 10, 11
  {0, 1, 2},     // Front Right: channels 0, 1, 2
  {4, 5, 6},     // Back Left:   channels 4, 5, 6
  {12, 13, 14}   // Back Right:  channels 12, 13, 14
};

struct Point {
  float x;
  float y;
  float z;
};

struct JointAngles {
  float theta1;  // Base rotation
  float theta2;  // Shoulder
  float theta3;  // Elbow
};

struct LegConfig {
  bool isLeft;     // true for left legs, false for right legs
  float xOffset;   // X offset from robot center
  float yOffset;   // Y offset from robot center
};

// Leg configurations (adjust these based on your robot's geometry)
const LegConfig legConfigs[4] = {
  {true, 3, 4},    // Front Left
  {false, 3, -4},  // Front Right  
  {true, -3, 4},   // Back Left
  {false, -3, -4}  // Back Right
};

JointAngles inverseKinematics(Point target, LegID legId) {
  JointAngles angles;
  LegConfig config = legConfigs[legId];
  
  // Adjust target position relative to leg base
  Point localTarget;
  localTarget.x = target.x - config.xOffset;
  localTarget.y = target.y - config.yOffset;
  localTarget.z = target.z;
  
  // Base rotation angle
  double q0 = atan2(localTarget.y, localTarget.x) * (180/PI);
  
  // Calculate reach in YZ plane
  double r = sqrt(localTarget.y * localTarget.y + localTarget.z * localTarget.z);
  
  // Check if target is reachable
  if (r > (a1 + a2) || r < abs(a1 - a2)) {
    Serial.print("Leg "); Serial.print(legId); Serial.println(": Target position is not reachable!");
    angles.theta1 = servoOffset;
    angles.theta2 = servoOffset;
    angles.theta3 = servoOffset;
    return angles;
  }
  
  // Elbow angle
  double q2 = acos((r * r - (a1 * a1) - (a2 * a2)) / (2 * a1 * a2)) * (180 / PI);
  
  // Shoulder angle calculation
  double a2_sin_q2 = a2 * sin(q2 * PI / 180);
  double a2_cos_q2 = a2 * cos(q2 * PI / 180);
  double beta = atan2(a2_sin_q2, (a1 + a2_cos_q2)) * (180 / PI);
  double gamma = atan2(localTarget.z, localTarget.y) * (180 / PI);
  double q1 = gamma - beta;
  
  angles.theta1 = q0;
  
  // Apply leg-specific transformations
  if (config.isLeft) {
    // Left legs
    angles.theta2 = constrain(q1 + servoOffset, servoMin, servoMax);
    angles.theta3 = constrain(q2, servoMin, servoMax);
  } else {
    // Right legs - mirror the movements
    angles.theta2 = constrain(servoOffset - q1, servoMin, servoMax);
    angles.theta3 = constrain(servoOffset - (q2 - servoOffset), servoMin, servoMax);
  }
  
  return angles;
}

void moveLegServo(LegID legId, JointAngles angles) {
  int* channels = (int*)legServos[legId];
  
  board1.setPWM(channels[0], 0, angleToPulse(angles.theta1));
  board1.setPWM(channels[1], 0, angleToPulse(angles.theta2));
  board1.setPWM(channels[2], 0, angleToPulse(angles.theta3));
  
  Serial.print("Leg "); Serial.print(legId);
  Serial.print(" - Angles: "); Serial.print(angles.theta1);
  Serial.print(", "); Serial.print(angles.theta2);
  Serial.print(", "); Serial.println(angles.theta3);
}

void moveAllLegs(Point target) {
  JointAngles angles[4];
  
  // Calculate angles for all legs
  for (int i = 0; i < 4; i++) {
    angles[i] = inverseKinematics(target, (LegID)i);
  }
  
  // Move all servos simultaneously
  for (int i = 0; i < 4; i++) {
    moveLegServo((LegID)i, angles[i]);
  }
}

void moveSpecificLegs(Point target, LegID legs[], int numLegs) {
  for (int i = 0; i < numLegs; i++) {
    JointAngles angles = inverseKinematics(target, legs[i]);
    moveLegServo(legs[i], angles);
  }
}

double angleToPulse(double ang) {
  double pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  return pulse;
}

// Walking pattern points (adjust these for your desired movement)
const int NUM_POINTS = 4;
Point walkingPattern[NUM_POINTS] = {
  {0, 6, 8},                    // Stance position
  {0, 1.796, 8.481},           // Lift off
  {5, 1.796, 10},              // Forward swing
  {10, 1.796, 10}              // Touch down
};

// Example gait sequences
void standingPosition() {
  Point standPoint = {0, 6, 8};
  moveAllLegs(standPoint);
  delay(1000);
}

void basicWalk() {
  // Simple trot gait - diagonal legs move together
  LegID group1[2] = {FRONT_LEFT, BACK_RIGHT};
  LegID group2[2] = {FRONT_RIGHT, BACK_LEFT};
  
  for (int step = 0; step < NUM_POINTS; step++) {
    // Move group 1
    moveSpecificLegs(walkingPattern[step], group1, 2);
    delay(200);
    
    // Move group 2  
    moveSpecificLegs(walkingPattern[step], group2, 2);
    delay(200);
  }
}

void customLegMovement() {
  // Example: Move each leg individually
  for (int legId = 0; legId < 4; legId++) {
    Serial.print("Moving leg "); Serial.println(legId);
    for (int i = 0; i < NUM_POINTS; i++) {
      JointAngles angles = inverseKinematics(walkingPattern[i], (LegID)legId);
      moveLegServo((LegID)legId, angles);
      delay(300);
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("4-Leg Quadruped Robot Controller");
  
  board1.begin();
  board1.setPWMFreq(60);
  
  // Initialize to standing position
  delay(1000);
  standingPosition();
}

void loop() {
  // Demonstrate different movement patterns
  
  // 1. All legs moving in sync
  Serial.println("All legs moving together:");
  for (int i = 0; i < NUM_POINTS; i++) {
    moveAllLegs(walkingPattern[i]);
    delay(500);
  }
  
  delay(1000);
  
  // 2. Basic walking pattern
  Serial.println("Basic trot walk:");
  basicWalk();
  
  delay(1000);
  
  // 3. Individual leg control
  Serial.println("Individual leg movements:");
  customLegMovement();
  
  delay(2000);
  
  // Return to standing
  standingPosition();
  delay(2000);
}
