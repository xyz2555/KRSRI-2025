import threading
import cv2
import numpy as np
import time
from adafruit_servokit import ServoKit
import Adafruit_PCA9685
import math
# import onnxruntime as rt

FRAME_SIZE = 320
NAMES = ["dummy", "victim"] # Urutannya harus disesuaikan dengan yang ada di data.yaml

pwm = Adafruit_PCA9685.PCA9685()
SERVOMIN = 125
SERVOMAX = 625
pwm.set_pwm_freq(60)

a1 = 8
a2 = 14.5

servo1_offset = 90
servo2_offset = 90
servo1_min = 0
servo1_max = 180
servo2_min = 0
servo2_max = 180

def preprocess_frame(frame) :
    blob = cv2.dnn.blobFromImage(frame, 
                                 1/255.0, 
                                 (FRAME_SIZE, FRAME_SIZE), 
                                 swapRB=True, 
                                 crop=False)
    return blob

# Fungsi untuk mendeteksi objek dalam frame
def detect_objects(model, blob, frame_width, frame_height):
    class_ids, confs, boxes = [], [], []

    # Setting input ke model dan melakukan prediksi
    model.setInput(blob)
    preds = model.forward()
    preds = preds.transpose((0, 2, 1))  # Menyesuaikan output agar lebih mudah dibaca

    # Mengitung skala berdasarkan ukuran gambar
    x_factor = frame_width / FRAME_SIZE
    y_factor = frame_height / FRAME_SIZE

    rows = preds[0].shape[0]

    # Loop untuk memproses setiap prediksi
    for i in range(rows):
        row = preds[0][i]
        # conf = row[4]
        
        # Mengambil skor kelas dan mencari kelas dengan skor tertinggi
        classes_score = row[4:]
        _, _, _, max_idx = cv2.minMaxLoc(classes_score)
        class_id = max_idx[1]

        # Simpan hanya prediksi dengan confidence di atas threshold
        if classes_score[class_id] > 0.25:
            confs.append(float(classes_score[class_id]))  # Menambahkan confidence score ke list
            class_ids.append(class_id)  # Menyimpan id kelas objek
            
            # Mengambil koordinat bounding box dalam format relatif
            x, y, w, h = row[:4]
            left = int((x - 0.5 * w) * x_factor)
            top = int((y - 0.5 * h) * y_factor)
            width = int(w * x_factor)
            height = int(h * y_factor)

            # Menyimpan bounding box
            boxes.append([left, top, width, height])

    # Menerapkan nms hanya jika ada deteksi
    indexes = []
    if len(boxes) > 0 and len(confs) > 0:
        indexes = cv2.dnn.NMSBoxes(boxes, np.array(confs), 0.2, 0.5)

        # Menyimpan hasil deteksi
        indexes = [(boxes[i], confs[i], class_ids[i] if i < len(class_ids) else -1) for i in indexes.flatten()]

    # Menmbuat variabel untuk mencari objek terbesar
    largest_area = 0
    largest_idx = -1

    for idx, (box, _, _) in enumerate(indexes):
        area = box[2] * box[3]
        if area > largest_area:
            largest_area = area
            largest_idx = idx

    return indexes, largest_idx

# Fungsi untuk enggambar hasil deteksi pada frame
def draw_detections(frame, indexes, largest_idx):
    """Menggambar bounding box dan label pada gambar."""

    frame_height, frame_width = frame.shape[:2]
    window_center = (frame_width // 2, frame_height // 2)

    # Fungsi untuk mengkonversi koordinat absolut menjadi relatif terhadap center
    def to_center_coord(point):
        return (
            point[0] - window_center[0],
            window_center[1] - point[1] # Y relatif terhadap tengah (dibalik karena Y komputer terbalik)
        )

    # Menggambar titik center window
    cv2.circle(frame, window_center, 5, (255, 0, 0), -1)

    for idx, (box, conf, class_id) in enumerate(indexes):
        left, top, width, height = box

        # Titik center bounding box
        bbox_center = (left + width // 2, top + height // 2)

        # Konversi ke koordinat relatif terhadap center window
        center_coord = to_center_coord(bbox_center)
        
        if (idx == largest_idx) and (class_id == 1):
            # Untuk objek terbesar (paling dekat dengan kamera)
            cv2.rectangle(frame, (left, top), (left + width, top + height), (0, 255, 0), 3)
            cv2.line(frame, bbox_center, window_center, (0, 255, 255), 2)
            print(f"{center_coord}")
            # print(type(center_coord))
        else:
            # Untuk objek lain
            print(f"{center_coord}")
            cv2.rectangle(frame, (left, top), (left + width, top + height), (255, 255, 255), 1)
        
        # Membuatlabel bounding box
        label = f"{NAMES[class_id]} {round(float(conf), 2)}"

        # menambahkan labek ke bounding box
        cv2.putText(frame, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

def image():
    
    # Menginisialisasi model 
    model = cv2.dnn.readNet("v5n.onnx")

    cap = cv2.VideoCapture(0)  # Mengambil video
    grabbed = True             # Variabel untuk mengecek apakah frame berhasil diambil
    
    # Inisiasi untuk fps
    frame_count = 0
    start_time = time.time()
    fps_list = []
    fps_update_interval = 10  # update fps setiap 10 frame
    avg_fps = 0 
    
    while True:
        frame_start_time = time.time()
        
        grabbed, capture = cap.read()  # Membaca frame dari webcam

        # Keluar jika tidak ada frame yang diambil
        if not grabbed:
            break

        # Copy frame untuk diproses
        frame = capture.copy()
    
        # Mengubah gambar ke format yang dapat diterima oleh YOLO
        blob = preprocess_frame(frame)

        # Mendeteksi objek dalam frame
        indexes, largest_idx = detect_objects(model, blob, frame.shape[1], frame.shape[0])

        draw_detections(frame, indexes, largest_idx)
        
        # Menghitung fps
        frame_count += 1
        frame_time = time.time() - frame_start_time
        current_fps = 1 / frame_time if frame_time > 0 else 0
        fps_list.append(current_fps)
        
        if frame_count % fps_update_interval == 0:
            avg_fps = sum(fps_list) / len(fps_list)
            if len(fps_list) > 100:
                fps_list = fps_list[-100:]
                
        # Menampilkan fps di frame
        cv2.putText(frame, f"FPS: {current_fps:.1f} Avg: {avg_fps:.1f}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Menampilkan gambar dengan deteksi
        cv2.imshow("Detected", frame)

        # Keluar jika tombol 'q' ditekan
        if cv2.waitKey(1) == ord('q'):
            break
    
    # Hitung dan mencetak rata-rata fps
    end_time = time.time()
    total_time = end_time - start_time
    average_fps = frame_count / total_time if total_time > 0 else 0
    print(f"Average FPS: {average_fps:.2f}")

    # Menutup webcam dan jendela tampilan
    cap.release()
    cv2.destroyAllWindows()

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class JointAngles:
    def __init__(self, theta1=0, theta2=0, theta3=0):
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3

def inverse_kinematics(target):
    angles = JointAngles()
    
    q0 = math.degrees(math.atan2(target.y, target.x))
    r = math.sqrt(target.y ** 2 + target.z ** 2)
    
    if r > (a1 + a2) or r < abs(a1 - a2):
        print("Target position is not reachable!")
        angles.theta1 = servo1_offset
        angles.theta2 = servo2_offset
        return angles
    
    q2 = math.degrees(math.acos((r ** 2 - a1 ** 2 - a2 ** 2) / (2 * a1 * a2)))
    a2_sin_q2 = a2 * math.sin(math.radians(q2))
    a2_cos_q2 = a2 * math.cos(math.radians(q2))
    
    beta = math.degrees(math.atan2(a2_sin_q2, a1 + a2_cos_q2))
    gamma = math.degrees(math.atan2(target.z, target.y))
    q1 = gamma - beta
    
    reverse = abs(servo2_offset - q2)
    
    angles.theta1 = q0
    angles.theta2 = max(min(q1 + servo1_offset, servo1_max), servo1_min)
    angles.theta3 = max(min(servo2_offset - reverse, servo2_max), servo2_min)
    
    return angles

def angle_to_pulse(angle):
    pulse = int((angle - 0) * (SERVOMAX - SERVOMIN) / (180 - 0) + SERVOMIN)
    # print(f"Angle: {angle} pulse: {pulse}")
    return pulse

class ServoController():
    def __init__(self):
        self.points = [
            Point(0, 8, 14.5),
            Point(0, 5.853531763034923, 8.648622196577978),
            Point(5, 8.239610763246004, 6.41648770513905),
            Point(10, 8.239610763246004, 6.41648770513905)
            # Point(10, 8.239610763246004, 6.41648770513905)
        ]
        
        for i in range(4):
            self.angles = inverse_kinematics(self.points[i])
            self.move_servo_smooth(self.angles)
        
    def move_servo_smooth(self, angles):
        pwm.set_pwm(2,0, angle_to_pulse(angles.theta1))
        pwm.set_pwm(1,0, angle_to_pulse(angles.theta2))
        pwm.set_pwm(0,0, angle_to_pulse(angles.theta3))
        time.sleep(0.2)

def ajojing():
    while True:
        ServoController()

if __name__ == '__main__':
    th1 = threading.Thread(target=ajojing)
    th2 = threading.Thread(target=image)
    th1.start()
    th2.start()