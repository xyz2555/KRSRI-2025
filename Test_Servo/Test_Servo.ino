#include <Servo.h>

//Servo servo[4][3];
Servo servo01;
Servo servo02;

const int servo_pin[4][3] = {
  {2, 3, 4},
  {5, 6, 7},
  {8, 9, 10},
  {11, 12, 13}
};

//void servo_attach(void)
//{
//  for (int i = 0; i < 4; i++)
//  {
//    for (int j = 0; j < 3; j++)
//    {
//      servo[i][j].attach(servo_pin[i][j]);
//      delay(100);
//    }
//  }
//}
//
//void servo_detach(void)
//{
//  for (int i = 0; i < 4; i++)
//  {
//    for (int j = 0; j < 3; j++)
//    {
//      servo[i][j].detach();
//      delay(100);
//    }
//  }
//}

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
//  servo_attach();
servo01.attach(3);
servo02.attach(4);
}

void loop() {
  servo01.write(135);
  servo02.write(135);
}
