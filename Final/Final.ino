  #include <FlexiTimer2.h>
#include <Servo.h>

// ============================================================================
//                              KONSTANTA & KONFIGURASI
// ============================================================================

// Konstanta Servo
const int servo1Offset = 90;           // Offset servo 1
const int servo2Offset = 90;           // Offset servo 2
const float stand_seat_speed = 1;      // Kecepatan gerakan

// Konstanta Gerakan
const float STEP_LENGTH = 0.36;        // Panjang langkah
const float STEP_HEIGHT = 0.7;         // Tinggi angkat kaki

// Posisi standby default (koordinat cartesian)
const float X_STANDBY = 0.0;
const float Y_STANDBY = 0.49585484722313566;
const float Z_STANDBY = 8.180476404905836;

// Parameter kinematik (panjang segmen kaki)
const float a1 = 6.0;                  // Panjang femur
const float a2 = 8.0;                  // Panjang tibia

// Konstanta untuk mempertahankan posisi saat ini
const float KEEP = 255.0;

// Pin servo untuk setiap kaki [kaki][joint]
// Kaki: 0=depan kiri, 1=depan kanan, 2=belakang kanan, 3=belakang kiri
const int servo_pin[4][3] = {
  {2, 3, 4},    // Kaki 0 (depan kiri)
  {5, 6, 7},    // Kaki 1 (depan kanan)
  {8, 9, 10},   // Kaki 2 (belakang kanan)
  {11, 12, 13}  // Kaki 3 (belakang kiri)
};

// ============================================================================
//                              VARIABEL GLOBAL
// ============================================================================

// Servo objects untuk 4 kaki, masing-masing 3 servo (hip, knee, ankle)
Servo servo[4][3];

// Variabel untuk kontrol gerakan
volatile float point_expect[4][3];     // Target posisi [kaki][x,y,z]
volatile float point_now[4][3];        // Posisi saat ini [kaki][x,y,z]
volatile int rest_counter;             // Counter untuk sinkronisasi

float temp_speed[4][3];                // Kecepatan temporer untuk interpolasi
float move_speed;                      // Kecepatan gerakan saat ini
float speed_multiple = 1.0;            // Multiplier kecepatan

// ============================================================================
//                              SETUP & CONTROL SERVO
// ============================================================================

/**
   Menghubungkan semua servo ke pin yang ditentukan
*/
void servo_attach(void) {
  Serial.println("Menghubungkan servo...");
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      servo[i][j].attach(servo_pin[i][j]);
      delay(100);
    }
  }
  Serial.println("Semua servo terhubung");
}

/**
   Memutuskan koneksi semua servo
*/
void servo_detach(void) {
  Serial.println("Memutuskan koneksi servo...");
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      servo[i][j].detach();
      delay(100);
    }
  }
  Serial.println("Semua servo terputus");
}

/**
   Menulis sudut ke servo dengan kompensasi untuk setiap kaki
   @param leg: nomor kaki (0-3)
   @param alpha, beta, gamma: sudut dalam derajat
*/
void servo_write(int leg, float alpha, float beta, float gamma) {
  // Kompensasi sudut untuk setiap kaki berdasarkan orientasi
  switch (leg) {
    case 0: // Depan kiri
      beta = beta + 90;
      // gamma tetap
      break;

    case 1: // Depan kanan
    case 2: // Belakang kanan
      {
        float reverse = abs(servo2Offset - gamma);
        beta = abs(90 - beta);
        gamma = servo2Offset - reverse;
      }
      break;

    case 3: // Belakang kiri
      beta = beta + 90;
      // gamma tetap
      break;
  }

  // Tulis ke servo
  servo[leg][0].write(alpha);
  servo[leg][1].write(beta);
  servo[leg][2].write(gamma);
}

/**
   Interrupt service routine untuk kontrol servo
   Dipanggil setiap 50ms untuk mengupdate posisi servo
*/
void servo_service(void) {
  sei(); // Enable interrupt
  static float alpha, beta, gamma;

  // Update posisi untuk setiap kaki
  for (int i = 0; i < 4; i++) {
    // Interpolasi posisi untuk smooth movement
    for (int j = 0; j < 3; j++) {
      if (abs(point_now[i][j] - point_expect[i][j]) >= abs(temp_speed[i][j])) {
        point_now[i][j] += temp_speed[i][j];
      } else {
        point_now[i][j] = point_expect[i][j];
      }
    }

    // Konversi ke sudut servo menggunakan inverse kinematics
    inverse_kinematic(alpha, beta, gamma, point_now[i][0], point_now[i][1], point_now[i][2]);
    servo_write(i, alpha, beta, gamma);
  }

  rest_counter++;
}

// ============================================================================
//                              INVERSE KINEMATICS
// ============================================================================

/**
   Menghitung sudut servo dari koordinat cartesian (inverse kinematics)
   @param alpha: sudut hip (output)
   @param beta: sudut knee (output)
   @param gamma: sudut ankle (output)
   @param x, y, z: koordinat target
*/
void inverse_kinematic(volatile float &alpha, volatile float &beta, volatile float &gamma,
                       volatile float x, volatile float y, volatile float z) {

  // Hitung sudut hip (alpha)
  if (abs(x) < 0.001) {
    alpha = 90.0;  // Stabilkan alpha saat x≈0
  } else {
    alpha = atan2(y, x) * (180.0 / PI);
    if (alpha < 0) alpha += 180.0; // Pastikan sudut positif
  }

  // Hitung jarak dari hip ke target
  float r = sqrt(y * y + z * z);

  // Cek apakah target dapat dijangkau
  if (r > (a1 + a2) || r < abs(a1 - a2)) {
    Serial.println("PERINGATAN: Target tidak dapat dijangkau!");
    beta = servo1Offset;
    gamma = servo2Offset;
    return;
  }

  // Hitung sudut ankle (gamma) menggunakan law of cosines
  gamma = acos((r * r - (a1 * a1) - (a2 * a2)) / (2 * a1 * a2)) * (180 / PI);

  // Hitung komponen untuk sudut knee (beta)
  float gamma_sin_q2 = a2 * sin(gamma * PI / 180);
  float gamma_cos_q2 = a2 * cos(gamma * PI / 180);

  float beta2 = atan2(gamma_sin_q2, (a1 + gamma_cos_q2)) * (180 / PI);
  float gamma2 = atan2(z, y) * (180 / PI);
  beta = gamma2 - beta2;
}

// ============================================================================
//                              FUNGSI SINKRONISASI
// ============================================================================

/**
   Menunggu satu kaki mencapai target posisi
   @param leg: nomor kaki yang ditunggu
*/
void wait_reach(int leg) {
  while (1) {
    if (point_now[leg][0] == point_expect[leg][0] &&
        point_now[leg][1] == point_expect[leg][1] &&
        point_now[leg][2] == point_expect[leg][2]) {
      break;
    }
  }
}

/**
   Menunggu semua kaki mencapai target posisi
*/
void wait_all_reach(void) {
  for (int i = 0; i < 4; i++) {
    wait_reach(i);
  }
}

// ============================================================================
//                              FUNGSI SET TARGET POSISI
// ============================================================================

/**
   Set target posisi untuk satu kaki
   @param leg: nomor kaki (0-3)
   @param x, y, z: koordinat target (gunakan KEEP untuk mempertahankan)
*/
void set_point(int leg, float x, float y, float z) {
  float length_x = 0, length_y = 0, length_z = 0;

  // Hitung jarak perpindahan untuk setiap axis
  if (x != KEEP) {
    length_x = x - point_now[leg][0];
  }
  if (y != KEEP) {
    length_y = y - point_now[leg][1];
  }
  if (z != KEEP) {
    length_z = z - point_now[leg][2];
  }

  // Hitung total jarak perpindahan
  float Length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  // Hitung kecepatan untuk setiap axis (untuk smooth interpolation)
  if (Length > 0) {
    temp_speed[leg][0] = length_x / Length * move_speed * speed_multiple;
    temp_speed[leg][1] = length_y / Length * move_speed * speed_multiple;
    temp_speed[leg][2] = length_z / Length * move_speed * speed_multiple;
  }

  // Set target posisi
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

// ============================================================================
//                              POSISI DASAR
// ============================================================================

/**
   Kalibrasi - posisi awal untuk kalibrasi servo
*/
void calibrate(void) {
  Serial.println("Memulai kalibrasi...");
  move_speed = stand_seat_speed;

  for (int leg = 0; leg < 4; leg++) {
    set_point(leg, 0, 6, 8);
  }
  wait_all_reach();
  Serial.println("Kalibrasi selesai");
}

/**
   Posisi standby - posisi siap bergerak
*/
void standby(void) {
  Serial.println("Menuju posisi standby...");
  move_speed = stand_seat_speed;

  for (int leg = 0; leg < 4; leg++) {
    set_point(leg, X_STANDBY, Y_STANDBY, Z_STANDBY);
  }
  wait_all_reach();
  Serial.println("Posisi standby tercapai");
}

/**
   Kembalikan satu kaki ke posisi standby
   @param leg: nomor kaki
*/
void back_to_standby(int leg) {
  move_speed = stand_seat_speed;
  set_point(leg, X_STANDBY, Y_STANDBY, Z_STANDBY);
  wait_reach(leg);
}

// ============================================================================
//                              GERAKAN PRIMITIF
// ============================================================================

/**
   Gerakan maju satu langkah
*/
void crawl_forward() {
  // Angkat kaki belakang kanan dan depan kiri
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();

  // Gerakkan ke depan
  set_point(2, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  set_point(0, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  wait_all_reach();

  // Turunkan kaki
  set_point(2, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  set_point(0, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  wait_all_reach();

  // Kembalikan ke posisi standby
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY);

  // Angkat kaki depan kanan dan belakang kiri
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();

  // Gerakkan ke depan
  set_point(1, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  set_point(3, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  wait_all_reach();

  // Turunkan kaki
  set_point(1, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  set_point(3, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  wait_all_reach();

  // Kembalikan ke posisi standby dan persiapan untuk siklus berikutnya
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
}

/**
   Gerakan mundur satu langkah
*/
void crawl_backward() {
  // Angkat kaki belakang kanan dan depan kiri
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();

  // Gerakkan ke belakang
  set_point(2, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  set_point(0, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  wait_all_reach();

  // Turunkan kaki
  set_point(2, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  set_point(0, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  wait_all_reach();

  // Kembalikan ke posisi standby
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY);

  // Angkat kaki depan kanan dan belakang kiri
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();

  // Gerakkan ke belakang
  set_point(1, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  set_point(3, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  wait_all_reach();

  // Turunkan kaki
  set_point(1, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  set_point(3, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  wait_all_reach();

  // Kembalikan ke posisi standby dan persiapan untuk siklus berikutnya
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
}

/**
   Gerakan rotasi kiri (yaw left)
*/
void yaw_left() {
  // Angkat kaki belakang kanan dan depan kiri
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();

  // Gerakkan untuk rotasi kiri
  set_point(2, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  set_point(0, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  wait_all_reach();

  // Turunkan kaki
  set_point(2, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  set_point(0, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  wait_all_reach();

  // Kembalikan ke posisi standby
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY);

  // Angkat kaki depan kanan dan belakang kiri
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();

  // Gerakkan untuk rotasi kiri
  set_point(1, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  set_point(3, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  wait_all_reach();

  // Turunkan kaki
  set_point(1, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  set_point(3, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  wait_all_reach();

  // Kembalikan ke posisi standby dan persiapan untuk siklus berikutnya
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
}

/**
   Gerakan rotasi kanan (yaw right) - belum diimplementasi
*/
void yaw_right() {
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();

  // Gerakkan untuk rotasi kanan
  set_point(2, X_STANDBY - 0.36, Y_STANDBY - 0.18, Z_STANDBY + 2);
  set_point(0, X_STANDBY - 0.36, Y_STANDBY - 0.18, Z_STANDBY + 2);
  wait_all_reach();

  // Turunkan kaki
  set_point(2, X_STANDBY - 0.36, Y_STANDBY - 0.18, Z_STANDBY);
  set_point(0, X_STANDBY - 0.36, Y_STANDBY - 0.18, Z_STANDBY);
  wait_all_reach();

  // Kembalikan ke posisi standby
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY);

  // Angkat kaki depan kanan dan belakang kanan
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();

  // Gerakkan untuk rotasi kanan
  set_point(1, X_STANDBY - 0.36, Y_STANDBY - 0.18, Z_STANDBY + 2);
  set_point(3, X_STANDBY - 0.36, Y_STANDBY - 0.18, Z_STANDBY + 2);
  wait_all_reach();

                    // Turunkan kaki
  set_point(1, X_STANDBY - 0.36, Y_STANDBY - 0.18, Z_STANDBY);
  set_point(3, X_STANDBY - 0.36, Y_STANDBY - 0.18, Z_STANDBY);
  wait_all_reach();

  // Kembalikan ke posisi standby dan persiapan untuk siklus berikutnya
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
}

void crab_sidewalk_left(){
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();

  set_point(2, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  set_point(0, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  wait_all_reach();

  set_point(2, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  set_point(0, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  wait_all_reach();

  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY);

  // Angkat kaki depan kanan dan belakang kiri
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
  
  set_point(1, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  set_point(3, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  wait_all_reach();

  set_point(1, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  set_point(3, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  wait_all_reach();

  // Kembalikan ke posisi standby dan persiapan untuk siklus berikutnya
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
}

void crab_sidewalk_right(){
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();

  set_point(2, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  set_point(0, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  wait_all_reach();

  set_point(2, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  set_point(0, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  wait_all_reach();

  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY);

  // Angkat kaki depan kanan dan belakang kiri
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
  
  set_point(1, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  set_point(3, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY + 2);
  wait_all_reach();

  set_point(1, X_STANDBY + 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  set_point(3, X_STANDBY - 0.36, Y_STANDBY + 0.18, Z_STANDBY);
  wait_all_reach();

  // Kembalikan ke posisi standby dan persiapan untuk siklus berikutnya
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
}

void forward_main(){
  for (int i = 0; i < 10; i++) {
    crawl_forward();
  }
}

void backward_main(){
  for (int i = 0; i < 10; i++) {
    crawl_backward();
  }
}

void yaw_left_main(){
  for (int i = 0; i < 10; i++) {
    yaw_left();
  }
}

void yaw_right_main(){
  for (int i = 0; i < 10; i++) {
    yaw_right();
  }
}

void sidewalk_left_main(){
  for (int i = 0; i < 10; i++) {
    crab_sidewalk_left();
  }
}

void sidewalk_right_main(){
  for (int i = 0; i < 10; i++) {
    crab_sidewalk_right();
  }
}

// ============================================================================
//                              KUMPULAN STAGE GERAKAN
// ============================================================================

/**
   Stage 1: Gerakan maju berulang
*/
void stage1() {
  Serial.println("=== STAGE 1: MAJU ===");

  for (int i = 0; i < 10; i++) {
    Serial.print("Siklus maju ke-");
    Serial.println(i + 1);
    crawl_forward();
  }

  Serial.println("Stage 1 selesai");
}

/**
   Stage 2: Gerakan rotasi kiri
*/
void stage2() {
  Serial.println("=== STAGE 2: ROTASI KIRI ===");

  for (int i = 0; i < 11; i++) {
    Serial.print("Siklus rotasi kiri ke-");
    Serial.println(i + 1);
    yaw_left();
  }

  Serial.println("Stage 2 selesai");
}

/**
   Stage 3: Gerakan mundur berulang
*/
void stage3() {
  Serial.println("=== STAGE 3: MUNDUR ===");

  for (int i = 0; i < 10; i++) {
    Serial.print("Siklus mundur ke-");
    Serial.println(i + 1);
    crawl_backward();
  }

  Serial.println("Stage 3 selesai");
}

// ============================================================================
//                              SETUP & MAIN LOOP
// ============================================================================

void setup() {
  Serial.begin(9600);
  Serial.println("=== QUADRUPED ROBOT CONTROLLER ===");
  Serial.println("Inisialisasi sistem...");

  // Inisialisasi servo
  servo_attach();

  // Inisialisasi posisi
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      point_now[i][j] = point_expect[i][j];
    }
  }

  Serial.println("Menunggu servo stabil...");
  delay(1000);

  // Setup timer interrupt untuk kontrol servo
  FlexiTimer2::set(50, servo_service); // 50ms interval
  FlexiTimer2::start();

  Serial.println("Sistem siap!");
}

void loop() {
  Serial.println("\n=== MEMULAI RUTINITAS GERAKAN ===");

  // Kalibrasi awal
  calibrate();
  delay(2000);

  // Ke posisi standby
  standby();
  delay(1000);

  sidewalk_left_main();
  delay(2000);

  Serial.println("Siklus selesai, mengulang...\n");
}
