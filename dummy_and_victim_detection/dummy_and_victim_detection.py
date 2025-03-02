import cv2
import numpy as np

FRAME_SIZE = 640
# NAMES = ["dummy", "victim"] # Urutannya harus disesuaikan dengan yang ada di data.yaml

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
def process_after_detections(frame, indexes, largest_idx):

    frame_height, frame_width = frame.shape[:2]
    window_center = (frame_width // 2, frame_height // 2)

    # Fungsi untuk mengkonversi koordinat absolut menjadi relatif terhadap center
    def to_center_coord(point):
        return (
            point[0] - window_center[0]
        )

    for idx, (box, _, class_id) in enumerate(indexes):
        left, top, width, height = box

        # Titik center bounding box
        bbox_center = (left + width // 2, top + height // 2)
        
        if (idx == largest_idx) and (class_id == 1):
            # Untuk objek terbesar (paling dekat dengan kamera)
            print(f"{to_center_coord(bbox_center)}")
        
if __name__ == "__main__":
    
    # Menginisialisasi model 
    model = cv2.dnn.readNet("dummy_and_victim_model.onnx")

    # Mengambil video 
    cap = cv2.VideoCapture(0) # SESUAIKAN DENGAN WEBCAM
    
    grabbed = True            # Variabel untuk mengecek apakah frame berhasil diambil

    while True:
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

        process_after_detections(frame, indexes, largest_idx)

    # Menutup webcam dan jendela tampilan
    cap.release()
