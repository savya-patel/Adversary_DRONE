import cv2
import time

cpt = 0
maxFrames = 70

# Used to record the time when we processed last frame
prev_frame_time = 0

# Used to record the time at which we processed current frame
new_frame_time = 0

cap = cv2.VideoCapture(0)

while cpt < maxFrames:
    ret, frame = cap.read()
    
    # Check if a frame was successfully read
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    
    # Frame processing
    frame = cv2.resize(frame, (320, 240))
    
    # Calculate FPS
    new_frame_time = time.time()
    fps = 1 / (new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time
    fps = int(fps)
    fps_text = "FPS: " + str(fps)
    
    # Put FPS text on the frame
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame, fps_text, (10, 30), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
    
    # Display the frame with FPS text
    cv2.imshow("test window", frame)
    
    # Save the frame
    cv2.imwrite("/home/eagle/Adversary_DRONE/yolo4/obj/%d.jpg" %cpt, frame)
    
    # Delay and counter logic
    time.sleep(0.01)
    cpt += 1
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
