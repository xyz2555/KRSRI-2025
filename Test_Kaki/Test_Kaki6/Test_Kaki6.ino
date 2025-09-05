#include <FlexiTimer2.h>
#include <Servo.h>

// ============================================================================
//                            KONSTANTA & VARIABEL GLOBAL
// ============================================================================

// Servo objects untuk 4 kaki, masing-masing 3 servo (hip, knee, ankle)
Servo servo[4][3];

// Konfigurasi servo
const int servo1Offset = 90;  // Offset servo 1
const int servo2Offset = 90;  // Offset servo 2
const float stand_seat_speed = 1; // Kecepatan gerakan

const float STEP_LENGTH = 0.36;   // panjang langkah
const float STEP_HEIGHT = 0.7;    // tinggi angkat kaki

// Pin servo untuk setiap kaki [kaki][joint]
// Kaki: 0=depan kiri, 1=depan kanan, 2=belakang kanan, 3=belakang kiri
const int servo_pin[4][3] = {
  {2, 3, 4},    // Kaki 0 (depan kiri)
  {5, 6, 7},    // Kaki 1 (depan kanan)
  {8, 9, 10},   // Kaki 2 (belakang kanan)
  {11, 12, 13}  // Kaki 3 (belakang kiri)
};

// Posisi standby default (koordinat cartesian)
const float X_STANDBY = 0.0;
const float Y_STANDBY = 0.49585484722313566;
const float Z_STANDBY = 8.180476404905836;

// Parameter kinematik (panjang segmen kaki)
const float a1 = 6.0;  // Panjang femur
const float a2 = 8.0;  // Panjang tibia

// Konstanta untuk mempertahankan posisi saat ini
const float KEEP = 255.0;

// Variabel untuk kontrol gerakan
volatile float point_expect[4][3];  // Target posisi [kaki][x,y,z]
volatile float point_now[4][3];     // Posisi saat ini [kaki][x,y,z]
volatile int rest_counter;          // Counter untuk sinkronisasi

float temp_speed[4][3];    // Kecepatan temporer untuk interpolasi
float move_speed;          // Kecepatan gerakan saat ini
float speed_multiple = 1.0; // Multiplier kecepatan

// ============================================================================
//                            FUNGSI SETUP & CONTROL SERVO
// ============================================================================

/**
 * Menghubungkan semua servo ke pin yang ditentukan
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
 * Memutuskan koneksi semua servo
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

// ============================================================================
//                            FUNGSI POSISI DASAR
// ============================================================================

/**
 * Kalibrasi - posisi awal untuk kalibrasi servo
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
 * Posisi standby - posisi siap bergerak
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
 * Posisi berjalan - mempersiapkan untuk berjalan
 */
void walk(void) {
  Serial.println("Persiapan berjalan...");
  move_speed = stand_seat_speed;
  
  for (int leg = 0; leg < 4; leg++) {
    set_point(leg, 1, 1.7955549577344083, 10);
  }
  wait_all_reach();
  Serial.println("Siap berjalan");
}

// Step: angkat -> maju -> turun
void step_leg(int leg) {
  // Angkat
  set_point(leg, KEEP, Y_STANDBY, Z_STANDBY + STEP_HEIGHT);
  wait_reach(leg);

  // Maju
  set_point(leg, X_STANDBY + STEP_LENGTH, KEEP, KEEP);
  wait_reach(leg);

  // Turun
  set_point(leg, KEEP, KEEP, Z_STANDBY);
  wait_reach(leg);
}

// Move: geser 3 kaki lain sedikit mundur
void move_body(int moving_leg) {
  for (int i = 0; i < 4; i++) {
    if (i != moving_leg) {
      set_point(i, X_STANDBY - STEP_LENGTH/3, KEEP, KEEP);
    }
  }
  wait_all_reach();
}


// ============================================================================
//                            FUNGSI GERAKAN CRAWL GAIT
// ============================================================================

/**
 * Satu langkah maju untuk kaki tertentu (crawl gait)
 * @param leg: nomor kaki (0-3)
 */
void crawl_forward_step(int leg) {
  Serial.print("Langkah maju kaki ");
  Serial.println(leg);
  
  // 1. Angkat kaki
  set_point(leg, KEEP, Y_STANDBY, Z_STANDBY + 0.7);
  wait_reach(leg);
  delay(100);

  // 2. Gerakkan maju
  set_point(leg, X_STANDBY - 0.36, KEEP, KEEP);
  wait_reach(leg);
  delay(100);

  // 3. Turunkan kaki
  set_point(leg, KEEP, KEEP, Z_STANDBY);
  wait_reach(leg);
  delay(100);

  // 4. Geser kaki lain untuk mendorong badan maju
  for (int i = 0; i < 4; i++) {
    if (i != leg) {
      set_point(i, X_STANDBY + 0.12, KEEP, KEEP);
      delay(100);
    }
  }
  wait_all_reach();
}

/**
 * Satu langkah mundur untuk kaki tertentu
 * @param leg: nomor kaki (0-3)
 */
void crawl_backward_step(int leg) {
  Serial.print("Langkah mundur kaki ");
  Serial.println(leg);
  
  // 1. Angkat kaki
  set_point(leg, KEEP, Y_STANDBY, Z_STANDBY + 0.7);
  wait_reach(leg);

  // 2. Gerakkan mundur
  set_point(leg, X_STANDBY + 0.36, KEEP, KEEP);
  wait_reach(leg);

  // 3. Turunkan kaki
  set_point(leg, KEEP, KEEP, Z_STANDBY);
  wait_reach(leg);

  // 4. Geser kaki lain untuk mendorong badan mundur
  for (int i = 0; i < 4; i++) {
    if (i != leg) {
      set_point(i, X_STANDBY - 0.12, KEEP, KEEP);
    }
  }
  wait_all_reach();
}

/**
 * Gerakan rotasi kiri untuk satu kaki (belum diimplementasi)
 */
void crawl_yaw_left_step(int leg) {
  // TODO: Implementasi gerakan rotasi kiri
  Serial.print("Rotasi kiri kaki ");
  Serial.println(leg);
}

/**
 * Gerakan rotasi kanan untuk satu kaki (belum diimplementasi)
 */
void crawl_yaw_right_step(int leg) {
  // TODO: Implementasi gerakan rotasi kanan
  Serial.print("Rotasi kanan kaki ");
  Serial.println(leg);
}

// ============================================================================
//                            POLA GERAKAN LENGKAP
// ============================================================================

/**
 * Pola jalan maju dengan crawl gait
 * Urutan: depan kiri -> belakang kanan -> depan kanan -> belakang kiri
 */
void crawl_forward() {
  Serial.println("Memulai crawl maju...");
  
  crawl_forward_step(0);  // Depan kiri
  delay(50);
  crawl_forward_step(2);  // Belakang kanan
  delay(50);
  crawl_forward_step(1);  // Depan kanan
  delay(50);
  crawl_forward_step(3);  // Belakang kiri
  delay(50);
  
  Serial.println("Crawl maju selesai");
}

/**
 * Pola jalan mundur dengan crawl gait
 */
void crawl_backward() {
  Serial.println("Memulai crawl mundur...");
  
  crawl_backward_step(0);
  delay(50);
  crawl_backward_step(1);
  delay(50);
  crawl_backward_step(2);
  delay(50);
  crawl_backward_step(3);
  delay(50);
  
  Serial.println("Crawl mundur selesai");
}

/**
 * Rotasi kiri penuh
 */
void crawl_yaw_left() {
  Serial.println("Rotasi kiri...");
  crawl_yaw_left_step(0); // Depan kiri
  crawl_yaw_left_step(2); // Belakang kanan
  crawl_yaw_left_step(1); // Depan kanan
  crawl_yaw_left_step(3); // Belakang kiri
}

/**
 * Rotasi kanan penuh
 */
void crawl_yaw_right() {
  Serial.println("Rotasi kanan...");
  crawl_yaw_right_step(0);
  crawl_yaw_right_step(2);
  crawl_yaw_right_step(1);
  crawl_yaw_right_step(3);
}

// ============================================================================
//                            FUNGSI GERAKAN ALTERNATIF
// ============================================================================

/**
 * Angkat kaki dan gerakkan maju
 */
void leg_up_forward(int leg) {
  // 1. Angkat tinggi
  set_point(leg, KEEP, Y_STANDBY, Z_STANDBY + 4);
  wait_reach(leg);

  // 2. Gerakkan maju sambil tetap terangkat
  set_point(leg, X_STANDBY + 0.36, KEEP, Z_STANDBY + 4);
  wait_reach(leg);

  // 3. Turunkan
  set_point(leg, KEEP, KEEP, Z_STANDBY);
  wait_reach(leg);
}

/**
 * Angkat kaki dan gerakkan mundur
 */
void leg_up_backward(int leg) {
  // 1. Angkat tinggi
  set_point(leg, KEEP, Y_STANDBY, Z_STANDBY + 4);
  wait_reach(leg);

  // 2. Gerakkan mundur sambil tetap terangkat
  set_point(leg, X_STANDBY - 0.36, KEEP, Z_STANDBY + 4);
  wait_reach(leg);

  // 3. Turunkan
  set_point(leg, KEEP, KEEP, Z_STANDBY);
  wait_reach(leg);
}

/**
 * Gerakkan kaki mundur tanpa mengangkat
 */
void leg_backward(int leg) {
  set_point(leg, X_STANDBY - 0.36, KEEP, KEEP);
  wait_reach(leg);
}

/**
 * Kembalikan kaki ke posisi standby
 */
void back_to_standby(int leg) {
  move_speed = stand_seat_speed;
  set_point(leg, X_STANDBY, Y_STANDBY, Z_STANDBY);
  wait_reach(leg);
}

/**
 * Pola jalan alternatif dengan gerakan simultan
 */
void crawl_forward3() {
  Serial.println("Crawl forward alternatif...");
  
  // Angkat dan majukan kaki 2 dan 1
  leg_up_forward(2);
  leg_up_forward(1);
  wait_all_reach();
  delay(300);
  
  // Kembalikan ke standby dan mundurkan kaki lain
  back_to_standby(2);
  back_to_standby(1);
  leg_backward(0);
  leg_backward(3);
  wait_all_reach();
  delay(300);
  
  // Angkat dan majukan kaki 3 dan 0
  leg_up_forward(3);
  leg_up_forward(0);
  wait_all_reach();
  delay(300);
  
  // Kembalikan ke standby dan mundurkan kaki lain
  back_to_standby(0);
  back_to_standby(3);
  leg_backward(1);
  leg_backward(2);
  wait_all_reach();
  delay(300);
}

void crawl_forward4() {
  // 1. Step kaki A (0)
  step_leg(0);
  move_body(0);

  // 2. Step kaki C (2)
  step_leg(2);
  move_body(2);

  // 3. Step kaki B (1)
  step_leg(1);
  move_body(1);

  // 4. Step kaki D (3)
  step_leg(3);
  move_body(3);
}


// ============================================================================
//                            RUTINITAS GERAKAN (STAGES)
// ============================================================================

/**
 * Stage 1: Gerakan maju berulang
 */
void stage1() {
  Serial.println("=== STAGE 1: MAJU ===");
  
  for (int i = 0; i < 10; i++) {
    Serial.print("Siklus maju ke-");
    Serial.println(i + 1);
    maju();
  }
  
  Serial.println("Stage 1 selesai, istirahat 3 detik...");
//  delay(3000);
}

/**
 * Stage 2: Gerakan rotasi kiri-kanan
 */
void stage2() {
  Serial.println("=== STAGE 2: ROTASI ===");
  
  Serial.println("Rotasi kiri 5x...");
  for (int i = 0; i < 5; i++) {
    crawl_yaw_left();
  }
  
  Serial.println("Istirahat 3 detik...");
  delay(3000);
  
  Serial.println("Rotasi kanan 5x...");
  for (int i = 0; i < 5; i++) {
    crawl_yaw_right();
  }
  
  Serial.println("Stage 2 selesai");
}

// ============================================================================
//                            SISTEM KONTROL SERVO
// ============================================================================

/**
 * Interrupt service routine untuk kontrol servo
 * Dipanggil setiap 100ms untuk mengupdate posisi servo
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
//                            INVERSE KINEMATICS
// ============================================================================

/**
 * Menghitung sudut servo dari koordinat cartesian (inverse kinematics)
 * @param alpha: sudut hip (output)
 * @param beta: sudut knee (output)  
 * @param gamma: sudut ankle (output)
 * @param x, y, z: koordinat target
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
//                            FUNGSI SINKRONISASI
// ============================================================================

/**
 * Menunggu satu kaki mencapai target posisi
 * @param leg: nomor kaki yang ditunggu
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
 * Menunggu semua kaki mencapai target posisi
 */
void wait_all_reach(void) {
  for (int i = 0; i < 4; i++) {
    wait_reach(i);
  }
}

// ============================================================================
//                            FUNGSI SET TARGET
// ============================================================================

/**
 * Set target posisi untuk satu kaki
 * @param leg: nomor kaki (0-3)
 * @param x, y, z: koordinat target (gunakan KEEP untuk mempertahankan)
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
//                            FUNGSI KONTROL SERVO
// ============================================================================

/**
 * Menulis sudut ke servo dengan kompensasi untuk setiap kaki
 * @param leg: nomor kaki (0-3)
 * @param alpha, beta, gamma: sudut dalam derajat
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

// ============================================================================
//                            SETUP DAN MAIN LOOP
// ============================================================================

void yaw_kiri(){
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
  set_point(2, X_STANDBY + 0.36, Y_STANDBY +0.18 , Z_STANDBY + 2);
  set_point(0, X_STANDBY + 0.36, Y_STANDBY +0.18 , Z_STANDBY + 2);
  wait_all_reach();
  set_point(2, X_STANDBY + 0.36, Y_STANDBY +0.18 , Z_STANDBY );
  set_point(0, X_STANDBY + 0.36, Y_STANDBY +0.18 , Z_STANDBY );
  wait_all_reach();
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
  set_point(1, X_STANDBY + 0.36, Y_STANDBY +0.18 , Z_STANDBY + 2);
  set_point(3, X_STANDBY + 0.36, Y_STANDBY +0.18 , Z_STANDBY + 2);
  wait_all_reach();
  set_point(1, X_STANDBY + 0.36, Y_STANDBY +0.18 , Z_STANDBY );
  set_point(3, X_STANDBY + 0.36, Y_STANDBY +0.18 , Z_STANDBY );
  wait_all_reach();
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
}

void yaw_kanan(){
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
  set_point(2, X_STANDBY - 0.36, Y_STANDBY - 0.18 , Z_STANDBY + 2);
  set_point(0, X_STANDBY - 0.36, Y_STANDBY - 0.18 , Z_STANDBY + 2);
  wait_all_reach();
  set_point(2, X_STANDBY - 0.36, Y_STANDBY - 0.18 , Z_STANDBY );
  set_point(0, X_STANDBY - 0.36, Y_STANDBY - 0.18 , Z_STANDBY );
  wait_all_reach();
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
  set_point(1, X_STANDBY - 0.36, Y_STANDBY - 0.18 , Z_STANDBY + 2);
  set_point(3, X_STANDBY - 0.36, Y_STANDBY - 0.18 , Z_STANDBY + 2);
  wait_all_reach();
  set_point(1, X_STANDBY - 0.36, Y_STANDBY - 0.18 , Z_STANDBY );
  set_point(3, X_STANDBY - 0.36, Y_STANDBY - 0.18 , Z_STANDBY );
  wait_all_reach();
  set_point(1, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(3, X_STANDBY, Y_STANDBY, Z_STANDBY);
  set_point(0, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  set_point(2, X_STANDBY, Y_STANDBY, Z_STANDBY + 2);
  wait_all_reach();
}

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
  FlexiTimer2::set(100, servo_service); // 100ms interval
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
  
  // Jalankan stage 1 (gerakan maju)
  stage1();
  delay(4000);
  
  Serial.println("Siklus selesai, mengulang...\n");
}
