import threading
import cv2
import numpy as np
import time
from Servo import ServoController
from ImageProcessing import detect_objects, preprocess_frame, draw_detections
# import onnxruntime as rt

FRAME_SIZE = 320
NAMES = ["dummy", "victim"] # Urutannya harus disesuaikan dengan yang ada di data.yaml

positioning = [4000,4000]

# Fungsi untuk enggambar hasil deteksi pada frame
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
        blob = preprocess_frame(frame, FRAME_SIZE)

        # Mendeteksi objek dalam frame
        indexes, largest_idx = detect_objects(model, blob, frame.shape[1], frame.shape[0], FRAME_SIZE)

        draw_detections(frame, indexes, largest_idx, NAMES, positioning)
        
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
    # print(f"Average FPS: {average_fps:.2f}")

    # Menutup webcam dan jendela tampilan
    cap.release()
    cv2.destroyAllWindows()

def ajojing():
    global positioning
    while True:
        if(positioning[0] == 4000):
            ServoController()
        else:
            KorbanTrack()
            continue

def printer():
    while True:
        if(positioning[0] != 4000):
            print(f"{positioning[0]}")
        else:
            print("None")
        time.sleep(1)

def KorbanTrack():
    while True:
        if(positioning[0] > -20 and positioning[0]<20):
            print("terdetek")
            time.sleep(1)
            time.sleep(1)
            time.sleep(1)
            time.sleep(1)
            time.sleep(1)
            break
        else:
            continue

if __name__ == '__main__':
    th1 = threading.Thread(target=printer)
    th2 = threading.Thread(target=image)
    th3 = threading.Thread(target=ajojing)
    th1.start()
    th2.start()
    th3.start()
    # th3.join()
    # th3.start()
    
    # for i in range(10):
    #     time.sleep(1)
    # for i in range(10):
    #     time.sleep(1)
