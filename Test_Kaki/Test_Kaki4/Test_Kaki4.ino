#include <FlexiTimer2.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  125
#define SERVOMAX  625

const int servo1Offset = 90;  // Offset to align with 0 degrees
const int servo2Offset = 90;  // Offset to align with 0 degrees
const float stand_seat_speed = 1;

// =============================================================
//                      Use it only for angle
// =============================================================

volatile float angle_expect[4][3];
volatile float angle_now[4][3];

float angle_speed[4][3];
// =============================================================
//                      Use it only for angle
// =============================================================

const float a1 = 6;
const float a2 = 8;

const int servo_pin[4][3] = {
  {0, 2, 3},
  {4, 6, 7},
  {8, 10, 11},
  {12, 14 , 15}
};

volatile float point_expect[4][3];
volatile float point_now[4][3];

volatile int rest_counter;

const float KEEP = 255;

float temp_speed[4][3];
float move_speed;
float speed_multiple = 1;

float angleToPulse(float ang)
{ float pulse = map(ang, 0, 180, SERVOMIN, SERVOMAX);
  Serial.print("Angle: "); Serial.print(ang);
  Serial.print(" pulse: "); Serial.println(pulse);
  return pulse;
}

void setup() {
  Serial.begin(9600);
  board1.begin();
  board1.setPWMFreq(60);

  set_point(0, 0, 8, 6);
  set_point(1, 0, 8, 6);
  set_point(2, 0, 8, 6);
  set_point(3, 0, 8, 6);
  
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }

  // Set all servos to center position immediately
  //  for (int i = 0; i < 4; i++) {
  //    servo_write(i, 90, 90, 90);
  //  }

  delay(1000); // Give servos time to reach center position


  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();
  // put your setup code here, to run once:\


}

void loop() {
  calibrate();
  delay(2000);
//  standby();
//  delay(3000);
  // put your main code here, to run repeatedly:

}\

void calibrate(void){
  move_speed = stand_seat_speed;
  for (int leg = 0 ; leg <4; leg++){
    set_point(leg, 0, 6, 8);
  }
  wait_all_reach();
}

// =============================================================
//                      Use it only for angle
// =============================================================

void servo_service_angle(void) {
  sei();

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      if (abs(angle_now[i][j] - angle_expect[i][j]) >= abs(angle_speed[i][j])) {
        angle_now[i][j] += angle_speed[i][j];
      }
      else {
        angle_now[i][j] = angle_expect[i][j];
      }
    }
    servo_write(i, angle_now[i][0], angle_now[i][1], angle_now[i][2]);
  }

  rest_counter++;
}

void set_servo_angle(int leg, float alpha, float beta, float gamma) {
  angle_expect[leg][0] = alpha;
  angle_expect[leg][1] = beta;
  angle_expect[leg][2] = gamma;

  float distance[3];
  distance[0] = alpha - angle_now[leg][0];
  distance[1] = beta - angle_now[leg][1];
  distance[2] = gamma - angle_now[leg][2];

  float max_distance = max(abs(distance[0]), max(abs(distance[1]), abs(distance[2])));

  if (max_distance < 0.1) {  // If movement is too small
    angle_speed[leg][0] = 0;
    angle_speed[leg][1] = 0;
    angle_speed[leg][2] = 0;
  } else {
    // Calculate proportional speed for synchronized movement
    angle_speed[leg][0] = (distance[0] / max_distance) * move_speed * speed_multiple;
    angle_speed[leg][1] = (distance[1] / max_distance) * move_speed * speed_multiple;
    angle_speed[leg][2] = (distance[2] / max_distance) * move_speed * speed_multiple;
  }

}
// =============================================================
//                      Use it only for angle
// =============================================================

void servo_service(void) {
  sei();
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      if (abs(point_now[i][j] - point_expect[i][j]) >= abs(temp_speed[i][j])) {
        point_now[i][j] += temp_speed[i][j];
      }
      else {
        point_now[i][j] = point_expect[i][j];
      }
    }

    inverse_kinematic(alpha, beta, gamma, point_now[i][0], point_now[i][1], point_now[i][2]);
    servo_write(i, alpha, beta, gamma);
  }
  rest_counter++;
}

void inverse_kinematic(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z) {
  alpha = atan2(y, x) * (180 / PI);
  float r = sqrt(y * y + z * z);

  if (r > (a1 + a2) || r < abs(a1 - a2)) {
    Serial.println("Target position is not reachable!");
    beta = servo1Offset;
    gamma = servo2Offset;
  }

  gamma = acos((r * r - (a1 * a1) - (a2 * a2)) / (2 * a1 * a2)) * (180 / PI);
  float gamma_sin_q2 = a2 * sin(gamma * PI / 180);
  float gamma_cos_q2 = a2 * cos(gamma * PI / 180);

  //  Serial.println(r);
  //  Serial.println(q2);
  //  Serial.println(a2_sin_q2);
  //  Serial.println(a2_cos_q2);

  float beta2 = atan2(gamma_sin_q2, (a1 + gamma_cos_q2)) * (180 / PI);
  float gamma2 = atan2(z, y) * (180 / PI);
  beta = gamma2 - beta2;

  //  float reverse = abs(servo2Offset - gamma);
  //
  //  angles.theta1 = q0;
}

//void wait_reach_angle(int leg) {
//  while (1) {
//    // ❌ Floating point comparison - hampir tidak pernah exact match!
//    if (abs(angle_now[leg][0] - angle_expect[leg][0]) < 1.0 &&
//        abs(angle_now[leg][1] - angle_expect[leg][1]) < 1.0 &&
//        abs(angle_now[leg][2] - angle_expect[leg][2]) < 1.0) {
//      break;
//    }
//    delay(10);
//  }
//}

void wait_reach(int leg) {
  while (1) {
    if (point_now[leg][0] == point_expect[leg][0]) {
      if (point_now[leg][1] == point_expect[leg][1]) {
        if (point_now[leg][2] == point_expect[leg][2]) {
          break;
        }
      }
    }
  }
}

void wait_all_reach(void) {
  for (int i = 0; i < 4; i++) {
    wait_reach(i);
  }
}

void set_point(int leg, float x, float y, float z) {
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP) {
    length_x = x - point_now[leg][0];
  }
  if (y != KEEP) {
    length_y = y - point_now[leg][1];
  }
  if (z != KEEP) {
    length_z = z - point_now[leg][2];
  }

  float Length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / Length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / Length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / Length * move_speed * speed_multiple;

  if (x != KEEP) {
    point_expect[leg][0] = x;
  }
  if (y != KEEP) {
    point_expect[leg][1] = y;
  }
  if (z != KEEP) {
    point_expect[leg][2] = z;
  }
}

void servo_write(int leg, float alpha, float beta, float gamma) {
  if (leg == 0) {
    beta = beta + 90;
    gamma = gamma;
  }

  else if (leg == 1) {
    float reverse = abs(servo2Offset - gamma);
    beta = abs(90 - beta);
    gamma = servo2Offset - reverse;
  }

  else if (leg == 2) {
    float reverse = abs(servo2Offset - gamma);
    beta = abs(90 - beta);
    gamma = servo2Offset - reverse;
  }

  else if (leg == 3) {
    beta = beta + 90;
    gamma = gamma;
  }

  board1.setPWM(servo_pin[leg][0], 0, angleToPulse(alpha));
  board1.setPWM(servo_pin[leg][1], 0, angleToPulse(beta));
  board1.setPWM(servo_pin[leg][2], 0, angleToPulse(gamma));
}
