#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  125
#define SERVOMAX  625
#define NUM_LEGS 4
#define SERVOS_PER_LEG 3

struct JointAngles {
  float theta1;
  float theta2;
  float theta3;
};

// Posisi sekarang dan target untuk setiap servo
float currentAngles[NUM_LEGS][SERVOS_PER_LEG];
float targetAngles[NUM_LEGS][SERVOS_PER_LEG];

int channelMap[4][3] = {
  {0,  2,  3},   // kiri depan
  {4,  6,  7},   // kanan depan
  {8,  10, 11},  // kanan belakang
  {12, 14, 15}   // kiri belakang
};

double angleToPulse(double ang) { 
  return map(ang, 0, 180, SERVOMIN, SERVOMAX);
}

void setAllTargetPositions(JointAngles angles[4]) {
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    targetAngles[leg][0] = angles[leg].theta1;
    targetAngles[leg][1] = angles[leg].theta2;
    targetAngles[leg][2] = angles[leg].theta3;
  }
}

bool moveToTargetSmooth(float speed = 2.0) {
  bool allReached = true;
  
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    for (int servo = 0; servo < SERVOS_PER_LEG; servo++) {
      float diff = targetAngles[leg][servo] - currentAngles[leg][servo];
      
      if (abs(diff) > 0.5) { // Masih ada selisih
        allReached = false;
        
        // Gerak bertahap menuju target
        if (diff > 0) {
          currentAngles[leg][servo] += min(speed, diff);
        } else {
          currentAngles[leg][servo] += max(-speed, diff);
        }
      } else {
        // Pastikan posisi akhir tepat
        currentAngles[leg][servo] = targetAngles[leg][servo];
      }
      
      // Update servo position
      board1.setPWM(channelMap[leg][servo], 0, angleToPulse(currentAngles[leg][servo]));
    }
  }
  
  return allReached; // true jika semua servo sudah mencapai target
}

// Fungsi alternatif dengan durasi yang lebih presisi
void moveToPositionTimed(JointAngles angles[4], unsigned long durationMs) {
  setAllTargetPositions(angles);
  
  unsigned long startTime = millis();
  unsigned long stepDelay = 20; // 20ms per step
  float progress = 0.0;
  
  // Simpan posisi awal
  float startAngles[NUM_LEGS][SERVOS_PER_LEG];
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    for (int servo = 0; servo < SERVOS_PER_LEG; servo++) {
      startAngles[leg][servo] = currentAngles[leg][servo];
    }
  }
  
  while (millis() - startTime < durationMs) {
    progress = (float)(millis() - startTime) / durationMs;
    if (progress > 1.0) progress = 1.0;
    
    // Interpolasi linear untuk setiap servo
    for (int leg = 0; leg < NUM_LEGS; leg++) {
      for (int servo = 0; servo < SERVOS_PER_LEG; servo++) {
        currentAngles[leg][servo] = startAngles[leg][servo] + 
          progress * (targetAngles[leg][servo] - startAngles[leg][servo]);
        
        board1.setPWM(channelMap[leg][servo], 0, angleToPulse(currentAngles[leg][servo]));
      }
    }
    
    delay(stepDelay);
  }
  
  // Pastikan posisi akhir tepat
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    for (int servo = 0; servo < SERVOS_PER_LEG; servo++) {
      currentAngles[leg][servo] = targetAngles[leg][servo];
      board1.setPWM(channelMap[leg][servo], 0, angleToPulse(currentAngles[leg][servo]));
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino Mega Smooth Servo Control!");
  
  board1.begin();
  board1.setPWMFreq(60);
  
  // Inisialisasi posisi awal
  for (int leg = 0; leg < NUM_LEGS; leg++) {
    for (int servo = 0; servo < SERVOS_PER_LEG; servo++) {
      currentAngles[leg][servo] = 90; // Posisi tengah
      targetAngles[leg][servo] = 90;
      board1.setPWM(channelMap[leg][servo], 0, angleToPulse(90));
    }
  }
  
  delay(2000);
  Serial.println("Initialization complete. Starting loop...");
}

void loop() {
  static int loopCounter = 0;
  loopCounter++;
  
  Serial.print("=== Loop iteration #");
  Serial.print(loopCounter);
  Serial.println(" ===");
  
  // Posisi 1: Semua servo ke sudut tertentu
  JointAngles pos1[4] = {
    {45, 90, 90},   // kiri depan
    {45, 90, 90},   // kanan depan
    {45, 90, 90},   // kanan belakang
    {45, 90, 90}    // kiri belakang
  };
  
  Serial.println("Moving to position 1...");
  moveToPositionTimed(pos1, 1000); // 1 detik
  Serial.println("Position 1 reached!");
  delay(500);
  
  // Posisi 2: Ubah sudut joint kedua
  JointAngles pos2[4] = {
    {45, 120, 90},   // kiri depan
    {45, 120, 90},   // kanan depan
    {45, 120, 90},   // kanan belakang
    {45, 120, 90}    // kiri belakang
  };
  
  Serial.println("Moving to position 2...");
  moveToPositionTimed(pos2, 1000); // 1 detik
  Serial.println("Position 2 reached!");
  delay(500);
  
  // Posisi 3: Variasi lebih banyak
  JointAngles pos3[4] = {
    {90, 60, 120},   // kiri depan
    {90, 60, 120},   // kanan depan  
    {90, 60, 120},   // kanan belakang
    {90, 60, 120}    // kiri belakang
  };
  
  Serial.println("Moving to position 3...");
  moveToPositionTimed(pos3, 1500); // 1.5 detik
  Serial.println("Position 3 reached!");
  delay(1000);
  
  Serial.println("Loop completed! Restarting...\n");
}
