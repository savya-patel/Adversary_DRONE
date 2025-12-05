from cvlib.object_detection import YOLO
import cv2
import time
import numpy as np

def apply_nms(boxes, confidences, nms_threshold=0.4):
    """Apply Non-Maximum Suppression"""
    if len(boxes) == 0:
        return [], []
    
    boxes = np.array(boxes)
    confidences = np.array(confidences)
    
    boxes_xywh = []
    for box in boxes:
        x1, y1, x2, y2 = box
        w = x2 - x1
        h = y2 - y1
        boxes_xywh.append([x1, y1, w, h])
    
    indices = cv2.dnn.NMSBoxes(boxes_xywh, confidences.tolist(), 
                               score_threshold=0.3, nms_threshold=nms_threshold)
    
    if len(indices) > 0:
        indices = indices.flatten()
        return boxes[indices].tolist(), confidences[indices].tolist()
    return [], []


cap = cv2.VideoCapture(0)
weights = "yolov4-tiny-custom_4000.weights"
config  = "yolov4-tiny-custom.cfg"
labels  = "obj.names"

print("Loading YOLO model...")
yolo = YOLO(weights, config, labels)
print("Model loaded!")

# Adjustable parameters
SHRINK_FACTOR = 0.15  # Adjust this: 0.0 = no shrink, 0.2 = shrink 20%
CONFIDENCE_THRESHOLD = 0.35
count = 0

while True:
    frame_start_time = time.time()
    
    ret, img = cap.read()
    if not ret:
        break
    
    count += 1
    if count % 3 != 0:
        continue
    
    original_height, original_width = img.shape[:2]
    
    # Detect at training size
    img_detection = cv2.resize(img, (416, 416))
    bbox, label, conf = yolo.detect_objects(img_detection, confidence=CONFIDENCE_THRESHOLD)
    
    # Apply NMS
    if len(bbox) > 0:
        bbox, conf = apply_nms(bbox, conf, nms_threshold=0.3)
        label = label[:len(bbox)]
    
    # Scale factors
    scale_x = original_width / 416
    scale_y = original_height / 416
    
    img1 = img.copy()
    
    for idx, (box, lbl, confidence) in enumerate(zip(bbox, label, conf)):
        x1, y1, x2, y2 = box
        
        # Scale to original resolution
        x1 = int(x1 * scale_x)
        y1 = int(y1 * scale_y)
        x2 = int(x2 * scale_x)
        y2 = int(y2 * scale_y)
        
        # Shrink bounding box to fit tighter
        width = x2 - x1
        height = y2 - y1
        shrink_x = int(width * SHRINK_FACTOR)
        shrink_y = int(height * SHRINK_FACTOR)
        
        x1 = x1 + shrink_x
        y1 = y1 + shrink_y
        x2 = x2 - shrink_x
        y2 = y2 - shrink_y
        
        # Clamp to bounds
        x1 = max(0, min(x1, original_width - 1))
        y1 = max(0, min(y1, original_height - 1))
        x2 = max(0, min(x2, original_width - 1))
        y2 = max(0, min(y2, original_height - 1))
        
        # Calculate center point
        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        
        # Draw bounding box (green)
        cv2.rectangle(img1, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw CENTER POINT with crosshair
        # Filled circle
        cv2.circle(img1, (center_x, center_y), 6, (0, 0, 255), -1)
        # Outer circle
        cv2.circle(img1, (center_x, center_y), 10, (255, 255, 0), 2)
        
        # Crosshair lines
        line_length = 20
        cv2.line(img1, (center_x - line_length, center_y), 
                (center_x + line_length, center_y), (0, 0, 255), 2)
        cv2.line(img1, (center_x, center_y - line_length), 
                (center_x, center_y + line_length), (0, 0, 255), 2)
        
        # Label with confidence
        text = f"{lbl}: {confidence:.2f}"
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
        label_y = max(y1 - 10, text_size[1] + 10)
        
        cv2.rectangle(img1, (x1, label_y - text_size[1] - 5), 
                     (x1 + text_size[0] + 5, label_y + 5), (0, 255, 0), -1)
        cv2.putText(img1, text, (x1 + 2, label_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        # Center coordinates below box
        coord_text = f"Center: ({center_x}, {center_y})"
        cv2.putText(img1, coord_text, (x1, y2 + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
        
        # Box dimensions
        dim_text = f"Size: {x2-x1}x{y2-y1}"
        cv2.putText(img1, dim_text, (x1, y2 + 50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        print(f"Drone #{idx+1} | Center: ({center_x},{center_y}) | Box: {x2-x1}x{y2-y1} | Conf: {confidence:.2f}")
    
    # Display frame info
    size_text = f"Size: {original_width} x {original_height}"
    cv2.putText(img1, size_text, (10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    
    fps = 1.0 / (time.time() - frame_start_time)
    fps_text = f"FPS: {fps:.2f}"
    cv2.putText(img1, fps_text, (10, 65), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    
    detect_text = f"Detections: {len(bbox)}"
    cv2.putText(img1, detect_text, (10, 100), 
               cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
    
    cv2.imshow("YOLOv4 Live Detection", img1)
    
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC
        break
    elif key == ord('s'):  # Save
        cv2.imwrite(f"detection_{int(time.time())}.jpg", img1)
        print("Screenshot saved!")
    elif key == ord('+'):  # Increase shrink
        SHRINK_FACTOR = min(0.3, SHRINK_FACTOR + 0.05)
        print(f"Shrink factor: {SHRINK_FACTOR:.2f}")
    elif key == ord('-'):  # Decrease shrink
        SHRINK_FACTOR = max(0.0, SHRINK_FACTOR - 0.05)
        print(f"Shrink factor: {SHRINK_FACTOR:.2f}")

cap.release()
cv2.destroyAllWindows()