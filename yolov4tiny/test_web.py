from cvlib.object_detection import YOLO
import cv2
from flask import Flask, Response
import threading

app = Flask(__name__)

# Global variables
frame_with_detections = None
lock = threading.Lock()

# Initialize camera and YOLO
cap = cv2.VideoCapture(0)
weights = "yolov4-tiny-custom_best.weights"
config  = "yolov4-tiny-custom.cfg"
labels  = "obj.names"
yolo = YOLO(weights, config, labels)

def detection_loop():
    """Background thread for detection"""
    global frame_with_detections
    count = 0
    
    while True:
        ret, img = cap.read()
        if not ret:
            continue
            
        count += 1
        if count % 10 != 0:
            continue
        
        img = cv2.resize(img, (680, 460))
        bbox, label, conf = yolo.detect_objects(img)
        
        # Draw bounding boxes
        img1 = img.copy()
        for box, lbl, confidence in zip(bbox, label, conf):
            x1, y1, x2, y2 = box
            cv2.rectangle(img1, (x1, y1), (x2, y2), (0, 255, 0), 2)
            text = f"{lbl}: {confidence:.2f}"
            cv2.putText(img1, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Update global frame
        with lock:
            frame_with_detections = img1.copy()

def generate_frames():
    """Generate frames for streaming"""
    while True:
        with lock:
            if frame_with_detections is None:
                continue
            frame = frame_with_detections.copy()
        
        # Encode frame to JPEG
        ret, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>YOLO Detection</title>
        <style>
            body {
                margin: 0;
                padding: 20px;
                background-color: #1e1e1e;
                color: white;
                font-family: Arial, sans-serif;
                text-align: center;
            }
            h1 {
                color: #4CAF50;
            }
            img {
                border: 3px solid #4CAF50;
                border-radius: 8px;
                max-width: 90%;
                height: auto;
            }
        </style>
    </head>
    <body>
        <h1>🎯 YOLOv4-Tiny Live Detection</h1>
        <img src="/video_feed" alt="Video Feed">
        <p>Detection running at ~5 FPS (processing every 10th frame)</p>
    </body>
    </html>
    """

if __name__ == '__main__':
    # Start detection thread
    detection_thread = threading.Thread(target=detection_loop, daemon=True)
    detection_thread.start()
    
    print("Starting web server...")
    print("Access from local machine: http://localhost:5000")
    print("Access from remote: http://<JETSON_IP>:5000")
    print("Press Ctrl+C to stop")
    
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
