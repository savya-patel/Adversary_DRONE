from cvlib.object_detection import YOLO
import cv2
import time

cap = cv2.VideoCapture(0)
weights = "yolov4-tiny-custom_best.weights"
config  = "yolov4-tiny-custom.cfg"
labels  = "obj.names"

# Initialize YOLO once
yolo = YOLO(weights, config, labels)

print("Starting detection (headless mode - saving frames to detections/)...")
print("Press Ctrl+C to stop")

import os
os.makedirs("detections", exist_ok=True)

count = 0
frame_num = 0
try:
    while True:
        ret, img = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
            
        count += 1
        if count % 10 != 0:
            continue
        
        img = cv2.resize(img, (680, 460))
        bbox, label, conf = yolo.detect_objects(img)
        
        # Draw bounding boxes manually (more reliable)
        img1 = img.copy()
        detections = []
        for box, lbl, confidence in zip(bbox, label, conf):
            x1, y1, x2, y2 = box
            # Draw rectangle
            cv2.rectangle(img1, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # Draw label
            text = f"{lbl}: {confidence:.2f}"
            cv2.putText(img1, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            detections.append(f"{lbl}({confidence:.2f})")
        
        # Save frame with detections
        frame_num += 1
        output_path = f"detections/frame_{frame_num:04d}.jpg"
        cv2.imwrite(output_path, img1)
        
        # Print detection info
        if detections:
            print(f"Frame {frame_num}: {', '.join(detections)} -> {output_path}")
        else:
            print(f"Frame {frame_num}: No detections -> {output_path}")
        
        time.sleep(0.1)  # Small delay to avoid flooding
        
except KeyboardInterrupt:
    print("\nStopping...")

cap.release()
print(f"Saved {frame_num} frames to detections/")
