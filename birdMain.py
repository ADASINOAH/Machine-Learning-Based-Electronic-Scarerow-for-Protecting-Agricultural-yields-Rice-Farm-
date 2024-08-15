#Machine Learning Code for bird detection
import cv2
import datetime
import os
from ultralytics import YOLO
model = YOLO("model/yolov8n.pt", "v8")
# Set dimensions of video frames
frame_width = 720
frame_height = 640
# Video source is MP4 file stored locally
cap = cv2.VideoCapture(0)
# Only save an image on frame 0
frame_count = 0
if not cap.isOpened():
    print("Cannot open video stream")
    exit()
while True:
    ret, frame = cap.read()
    if not ret:
        print("No video frame available")
        break
    # Resize the frame
    frame = cv2.resize(frame, (frame_width, frame_height))
    # Do prediction on image, with confidence greater than 80%
    detect_params = model.predict(source=[frame], conf=0.8, save=False)
    DP = detect_params[0].numpy()
    if len(DP) != 0:
        for i in range(len(detect_params[0])):
            boxes = detect_params[0].boxes
            box = boxes[i]
            clsID = box.cls.numpy()[0]
            conf = box.conf.numpy()[0]
            bb = box.xyxy.numpy()[0]
            c = box.cls
            # Name of object detected (e.g. 'bird')
            class_name = model.names[int(c)]
        # If the class name contains the word 'bird', do something with the frame
        if 'bird' in class_name.lower():
            if frame_count == 0:
                current_time = datetime.datetime.now()
                filename = os.path.join("images", current_time.strftime("bird_%Y-%m-%d_%H-%M-%S-%f.jpg"))
                success = cv2.imwrite(filename, frame)
            if frame_count == 10:
                frame_count = 0
            else:
                frame_count += 1
    # Display the frame onscreen
    cv2.imshow("Object Detection", frame)
    # End program when q is pressed
    if cv2.waitKey(1) == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
