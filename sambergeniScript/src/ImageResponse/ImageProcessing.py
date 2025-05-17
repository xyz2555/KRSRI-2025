import cv2
import numpy as np
import time

def preprocess_frame(frame, FRAME_SIZE) :
    blob = cv2.dnn.blobFromImage(frame, 
                                 1/255.0, 
                                 (FRAME_SIZE, FRAME_SIZE), 
                                 swapRB=True, 
                                 crop=False)
    return blob

# Fungsi untuk mendeteksi objek dalam frame
def detect_objects(model, blob, frame_width, frame_height,FRAME_SIZE):
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

def draw_detections(frame, indexes, largest_idx, names, positioning):
    """Menggambar bounding box dan label pada gambar."""
    
    frame_height, frame_width = frame.shape[:2]
    window_center = (frame_width // 2, frame_height // 2)

    # Fungsi konversi koordinat relatif terhadap tengah
    def to_center_coord(point):
        return (
            point[0] - window_center[0],
            window_center[1] - point[1]  # Y relatif (dibalik)
        )

    # Gambar titik tengah frame
    cv2.circle(frame, window_center, 5, (255, 0, 0), -1)
    positioning[0], positioning[1] = 4000, 4000

    for idx, (box, conf, class_id) in enumerate(indexes):
        left, top, width, height = box
        bbox_center = (left + width // 2, top + height // 2)
        center_coord = to_center_coord(bbox_center)

        if (idx == largest_idx) and (class_id == 1):
            cv2.rectangle(frame, (left, top), (left + width, top + height), (0, 255, 0), 3)
            cv2.line(frame, bbox_center, window_center, (0, 255, 255), 2)
            positioning[0], positioning[1] = center_coord
        else:
            cv2.rectangle(frame, (left, top), (left + width, top + height), (255, 255, 255), 1)

        label = f"{names[class_id]} {round(float(conf), 2)}"
        cv2.putText(frame, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

# Fungsi untuk enggambar hasil deteksi pada frame
def image(FRAME_SIZE, names, positioning): 
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

        draw_detections(frame, indexes, largest_idx, names, positioning)
        
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
