import cv2
import numpy as np
import time

FRAME_SIZE = 416  
CONFIDENCE_THRESHOLD = 0.25
NMS_THRESHOLD = 0.5
NAMES = ["dummy", "victim"]  

def preprocess_frame(frame):
    """Preprocess the frame for YOLOv4-tiny input"""
    
    blob = cv2.dnn.blobFromImage(frame, 
                                1/255.0, 
                                (FRAME_SIZE, FRAME_SIZE), 
                                swapRB=True, 
                                crop=False)
    return blob

def process_outputs(outputs, frame_width, frame_height):
    """Process YOLOv4-tiny outputs to get detection results"""
    boxes_output = outputs[0]  
    scores_output = outputs[1]  
    
    class_ids, confidences, boxes = [], [], []
    
    rows = boxes_output.shape[1]
    image_width, image_height = frame_width, frame_height
    
    x_factor = image_width / FRAME_SIZE
    y_factor = image_height / FRAME_SIZE
    
    for r in range(rows):
        row = boxes_output[0, r]
        confidence = scores_output[0, r, :]
        
        class_id = np.argmax(confidence)
        score = float(confidence[class_id])
        
        if score >= CONFIDENCE_THRESHOLD:
            cx, cy, w, h = row[0:4]
            
            left = int((cx - w/2) * x_factor)
            top = int((cy - h/2) * y_factor)
            width = int(w * x_factor)
            height = int(h * y_factor)
            
            boxes.append([left, top, width, height])
            confidences.append(score)
            class_ids.append(class_id)
    
    indices = []
    if len(boxes) > 0:
        indices = cv2.dnn.NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, NMS_THRESHOLD)
        indices = indices.flatten()
    
    result = [(boxes[i], confidences[i], class_ids[i]) for i in indices]
    
    largest_area = 0
    largest_idx = -1
    
    for idx, (box, _, _) in enumerate(result):
        area = box[2] * box[3]
        if area > largest_area:
            largest_area = area
            largest_idx = idx
            
    return result, largest_idx

def draw_detections(frame, detections, largest_idx):
    """Draw detection results on the frame"""
    frame_height, frame_width = frame.shape[:2]
    window_center = (frame_width // 2, frame_height // 2)
    
    def to_center_coord(point):
        return (
            point[0] - window_center[0],
            window_center[1] - point[1]  
        )
    
    cv2.circle(frame, window_center, 5, (255, 0, 0), -1)
    
    for idx, (box, conf, class_id) in enumerate(detections):
        left, top, width, height = box
        
        bbox_center = (left + width // 2, top + height // 2)
        
        center_coord = to_center_coord(bbox_center)
        
        if (idx == largest_idx) and (class_id == 1):  
            cv2.rectangle(frame, (left, top), (left + width, top + height), (0, 255, 0), 3)
            cv2.line(frame, bbox_center, window_center, (0, 255, 255), 2)
            print(f"{center_coord}")
        else:
            cv2.rectangle(frame, (left, top), (left + width, top + height), (255, 255, 255), 1)
        
        label = f"{NAMES[class_id]} {round(float(conf), 2)}"
        
        cv2.putText(frame, label, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

if __name__ == "__main__":
    model = cv2.dnn.readNet("yolov4-tiny-fixedver2.onnx")
    
    cap = cv2.VideoCapture(0)
    grabbed = True
    
    frame_count = 0
    start_time = time.time()
    fps_list = []
    fps_update_interval = 10
    avg_fps = 0
    
    while True:   
        frame_start_time = time.time()
        
        grabbed, capture = cap.read()
        
        if not grabbed:
            break
            
        frame = capture.copy()
        
        blob = preprocess_frame(frame)
        
        model.setInput(blob)
        outputs = model.forward(model.getUnconnectedOutLayersNames())
        
        detections, largest_idx = process_outputs(outputs, frame.shape[1], frame.shape[0])
        
        draw_detections(frame, detections, largest_idx)
        
        frame_count += 1
        frame_time = time.time() - frame_start_time
        current_fps = 1 / frame_time if frame_time > 0 else 0
        fps_list.append(current_fps)
        
        if frame_count % fps_update_interval == 0:
            avg_fps = sum(fps_list) / len(fps_list)
            if len(fps_list) > 100:
                fps_list = fps_list[-100:]  
        
        cv2.putText(frame, f"FPS: {current_fps:.1f} Avg: {avg_fps:.1f}", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        cv2.imshow("YOLOv4-tiny Detection", frame)
        
        if cv2.waitKey(1) == ord('q'):
            break
    
    end_time = time.time()
    total_time = end_time - start_time
    average_fps = frame_count / total_time if total_time > 0 else 0
    print(f"Average FPS: {average_fps:.2f}")
    
    cap.release()
    cv2.destroyAllWindows()
