#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  125
#define SERVOMAX  625

double angleToPulse(double ang)
{ double pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  Serial.print("Angle: "); Serial.print(ang);
  Serial.print(" pulse: "); Serial.println(pulse);
  return pulse;
}

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  board1.begin();
  board1.setPWMFreq(60);
}

void loop() {
  board1.setPWM(7, 0, angleToPulse(90));
  board1.setPWM(1, 0, angleToPulse(90));
  board1.setPWM(15, 0, angleToPulse(90));
  board1.setPWM(8, 0, angleToPulse(90));
  delay(1000);
  // put your main code here, to run repeatedly:

}
