#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  125                                                 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  625   

double a1 = 8;
double a2 = 14.5;

const int servo1Offset = 90;  // Offset to align with 0 degrees
const int servo2Offset = 90;  // Offset to align with 0 degrees
const int servo1Min = 0;      // Minimum angle
const int servo1Max = 180;    // Maximum angle
const int servo2Min = 0;      // Minimum angle
const int servo2Max = 180;    // Maximum angle

struct Point {
  float x;
  float y;
  float z;
};

struct JointAngles {
  float theta1;
  float theta2;
  float theta3;
};

JointAngles inverseKinematics(Point target) {
  JointAngles angles;

  double q0 = 0;
  double r = sqrt(target.y * target.y + target.z * target.z);

   if (r > (a1 + a2) || r < abs(a1 - a2)) {
        Serial.println("Target position is not reachable!");
        angles.theta1 = servo1Offset;
        angles.theta2 = servo2Offset;
        return angles;
    }
  
  double q2 = acos((r*r - (a1 * a1) - (a2 * a2)) / (2 * a1 * a2)) * (180 / PI);
//  double q2 = -acos((target.z * target.z + target.y * target.y - a1 * a1 - a2 * a2) / (2 *a1 * a2)) * (180 / PI);
  double a2_sin_q2 = a2 * sin(q2 * PI/180);
  double a2_cos_q2 = a2 * cos(q2 * PI/180);

//  Serial.println(r);
//  Serial.println(q2);
//  Serial.println(a2_sin_q2);
//  Serial.println(a2_cos_q2);

  double beta = atan2(a2_sin_q2, (a1 + a2_cos_q2)) * (180 / PI);
  double gamma = atan2(target.z, target.y) * (180 / PI);
  double q1 = gamma - beta;

  double reverse = abs(servo2Offset - q2);
  
  angles.theta1 = q0;
  angles.theta2 = constrain(q1 + servo1Offset, servo1Min, servo1Max);
  angles.theta3 = constrain(servo2Offset - reverse, servo2Min, servo2Max);
//  angles.theta2 = q1;
//  angles.theta3 = q2;

  return angles;
}

void moveServoSmooth(JointAngles angles) {
  bool moving = true;

  while (moving) {
    board1.setPWM(0,0, angleToPulse(angles.theta1));
    board1.setPWM(1,0, angleToPulse(angles.theta2));
    board1.setPWM(15,0, angleToPulse(angles.theta3));
    moving = false;
  }
}

double angleToPulse(double ang)                             //gets angle in degree and returns the pulse width
  {  double pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
     Serial.print("Angle: ");Serial.print(ang);
     Serial.print(" pulse: ");Serial.println(pulse);
     return pulse;
  }

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  board1.begin();
  board1.setPWMFreq(60);                  // Analog servos run at ~60 Hz updates
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  Point targetPoint = {0, 5.853531763034923, 8.648622196577978};

  JointAngles angles = inverseKinematics(targetPoint);

  moveServoSmooth(angles);
//  delay(5000);
//  Point targetPoint = {0, 80, -100};
//
//  JointAngles angles = inverseKinematics(targetPoint);
//
//  moveServoSmooth(angles);M
}

//int angleToPulse(int ang)                             //gets angle in degree and returns the pulse width
//  {  int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);  // map angle of 0 to 180 to Servo min and Servo max 
//     Serial.print("Angle: ");Serial.print(ang);
//     Serial.print(" pulse: ");Serial.println(pulse);
//     return pulse;
//  }
