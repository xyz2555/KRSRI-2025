#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  125
#define SERVOMAX  625

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

JointAngles angles[4];

//void moveServoSmooth(JointAngles angles) {
//  bool moving = true;
//
//  while (moving) {
//    //Kiri depan
    
//    moving = false;
//  }
//}


void moveAllLegs(JointAngles angles[4]) {
  int channelMap[4][3] = {
    {0,  2,  3},   // kiri depan
    {4,  6,  7},   // kanan depan
    {8,  10, 11},  // kanan belakang
    {12, 14, 15}   // kiri belakang
  };

  for (int leg = 0; leg < 4; leg++) {
    board1.setPWM(channelMap[leg][0], 0, angleToPulse(angles[leg].theta1));
    board1.setPWM(channelMap[leg][1], 0, angleToPulse(angles[leg].theta2));
    board1.setPWM(channelMap[leg][2], 0, angleToPulse(angles[leg].theta3));
  }
}

double angleToPulse(double ang){ 
  double pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  Serial.print("Angle: "); Serial.print(ang);
  Serial.print(" pulse: "); Serial.println(pulse);
  return pulse;
}

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  board1.begin();
  board1.setPWMFreq(60);

  angles[0] = {20, 70, 60};  // kiri depan
  angles[1] = {45, 90, 90};  // kanan depan
  angles[2] = {30, 80, 70};  // kanan belakang
  angles[3] = {60, 60, 60};  // kiri belakang
}

void loop() {
    //Kiri depan
    board1.setPWM(0, 0, angleToPulse(45));
    board1.setPWM(2, 0, angleToPulse(90));
    board1.setPWM(3, 0, angleToPulse(90));

    //Kanan depan
    board1.setPWM(4, 0, angleToPulse(45));
    board1.setPWM(6, 0, angleToPulse(90));
    board1.setPWM(7, 0, angleToPulse(90));

    //Kanan belakang
    board1.setPWM(8, 0, angleToPulse(45));
    board1.setPWM(10, 0, angleToPulse(90));
    board1.setPWM(11, 0, angleToPulse(90));

    //Kiri belakang
    board1.setPWM(12, 0, angleToPulse(45));
    board1.setPWM(14, 0, angleToPulse(90));
    board1.setPWM(15, 0, angleToPulse(90));

    delay (1000);
    
    //Kiri depan
    board1.setPWM(0, 0, angleToPulse(45));
    board1.setPWM(2, 0, angleToPulse(120));
    board1.setPWM(3, 0, angleToPulse(90));

    //Kanan depan
    board1.setPWM(4, 0, angleToPulse(45));
    board1.setPWM(6, 0, angleToPulse(120));
    board1.setPWM(7, 0, angleToPulse(90));

    //Kanan belakang
    board1.setPWM(8, 0, angleToPulse(45));
    board1.setPWM(10, 0, angleToPulse(120));
    board1.setPWM(11, 0, angleToPulse(90));

    //Kiri belakang
    board1.setPWM(12, 0, angleToPulse(45));
    board1.setPWM(14, 0, angleToPulse(120));
    board1.setPWM(15, 0, angleToPulse(90));

    delay (1000);
//  moveAllLegs(angles);
//
//  delay(1000);
//
//  // Contoh ubah sudut kaki satu per satu
//  angles[0] = {40, 80, 70};
//  angles[1] = {60, 100, 100};
//  angles[2] = {50, 90, 80};
//  angles[3] = {30, 50, 50};
//
//  moveAllLegs(angles);
//
//  delay(1000);
//  // put your main code here, to run repeatedly:

}
