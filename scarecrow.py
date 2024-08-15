import cv2
import datetime
import os
import random
import serial
import RPi.GPIO as GPIO
import time
from ultralytics import YOLO

# Initialize YOLO model
model = YOLO("model/yolov8n.pt", "v8")

# Video setup
frame_width = 720
frame_height = 640
cap = cv2.VideoCapture(0)

# Serial setup for DFPlayer Mini
try:
    ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    ser.flush()
except Exception as e:
    print(f"Error opening serial port: {e}")

# GPIO setup for vibrator motor
FORWARD_PWM_PIN = 23
BACKWARD_PWM_PIN = 24
FORWARD_ENABLE_PIN = 21
BACKWARD_ENABLE_PIN = 22

GPIO.setmode(GPIO.BOARD)
GPIO.setup(FORWARD_PWM_PIN, GPIO.OUT)
GPIO.setup(BACKWARD_PWM_PIN, GPIO.OUT)
GPIO.setup(FORWARD_ENABLE_PIN, GPIO.OUT)
GPIO.setup(BACKWARD_ENABLE_PIN, GPIO.OUT)

pwm_forward = GPIO.PWM(FORWARD_PWM_PIN, 1000)
pwm_backward = GPIO.PWM(BACKWARD_PWM_PIN, 1000)
pwm_forward.start(0)
pwm_backward.start(0)

# GPIO setup for servo motor
SERVO_PIN = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm_servo = GPIO.PWM(SERVO_PIN, 50)
pwm_servo.start(7.5)

def move_forward(speed):
    GPIO.output(FORWARD_ENABLE_PIN, GPIO.HIGH)
    GPIO.output(BACKWARD_ENABLE_PIN, GPIO.LOW)
    pwm_forward.ChangeDutyCycle(speed)
    pwm_backward.ChangeDutyCycle(0)

def move_backward(speed):
    GPIO.output(FORWARD_ENABLE_PIN, GPIO.LOW)
    GPIO.output(BACKWARD_ENABLE_PIN, GPIO.HIGH)
    pwm_forward.ChangeDutyCycle(0)
    pwm_backward.ChangeDutyCycle(speed)

def stop_motor():
    GPIO.output(FORWARD_ENABLE_PIN, GPIO.LOW)
    GPIO.output(BACKWARD_ENABLE_PIN, GPIO.LOW)
    pwm_forward.ChangeDutyCycle(0)
    pwm_backward.ChangeDutyCycle(0)

def set_angle(angle):
    duty_cycle = angle / 18 + 2.5
    pwm_servo.ChangeDutyCycle(duty_cycle)
    time.sleep(1)

def send_command(command, param1=0, param2=0):
    start_byte = 0x7E
    version = 0xFF
    length = 0x06
    end_byte = 0xEF
    data = [start_byte, version, length, command, param1, param2]
    checksum = -(sum(data[1:6])) & 0xFF
    data.append(checksum)
    data.append(end_byte)
    ser.write(bytearray(data))

def play_sound():
    send_command(0x03, 0x00, 0x01)

if not cap.isOpened():
    print("Cannot open video stream")
    exit()

frame_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        print("No video frame available")
        break

    frame = cv2.resize(frame, (frame_width, frame_height))
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
            class_name = model.names[int(c)]

        if 'bird' in class_name.lower():
            action = random.choice(['servo', 'sound', 'vibrator'])
            if action == 'servo':
                set_angle(90)  # Example: move servo to 90 degrees
            elif action == 'sound':
                play_sound()
            elif action == 'vibrator':
                move_forward(50)
                time.sleep(1)
                stop_motor()

            if frame_count == 0:
                current_time = datetime.datetime.now()
                filename = os.path.join("images", current_time.strftime("bird_%Y-%m-%d_%H-%M-%S-%f.jpg"))
                cv2.imwrite(filename, frame)
            if frame_count == 10:
                frame_count = 0
            else:
                frame_count += 1

    cv2.imshow("Object Detection", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()










 







