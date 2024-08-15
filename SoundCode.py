#Speaker Sound Producing code with DFROBOT #mp3 player

import serial
# Initialize serial communication with DFPlayer Mini
try:
    ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)
    ser.flush()
    print("Serial port opened successfully.")
except Exception as e:
    print(f"Error opening serial port: {e}")
def send_command(command, param1=0, param2=0):
    '''Send a command to the DFPlayer Mini'''
    start_byte = 0x7E
    version = 0xFF
    length = 0x06
    end_byte = 0xEF
    data = [start_byte, version, length, command, param1, param2]
    checksum = -(sum(data[1:6])) & 0xFF
    data.append(checksum)
    data.append(end_byte)
    ser.write(bytearray(data))
    print(f"Sent command: {[hex(x) for x in data]}")
def play_sound():
    '''Play sound on the DFPlayer Mini'''
    send_command(0x03, 0x00, 0x01)  # Play track 1


#Vibrator motor Control code

import RPi.GPIO as GPIO
import time

# Define Pin Numbers
FORWARD_PWM_PIN = 23
BACKWARD_PWM_PIN = 24
FORWARD_ENABLE_PIN = 21
BACKWARD_ENABLE_PIN = 22

# Initialize GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(FORWARD_PWM_PIN, GPIO.OUT)
GPIO.setup(BACKWARD_PWM_PIN, GPIO.OUT)
GPIO.setup(FORWARD_ENABLE_PIN, GPIO.OUT)
GPIO.setup(BACKWARD_ENABLE_PIN, GPIO.OUT)

# Set up PWM
pwm_forward = GPIO.PWM(FORWARD_PWM_PIN, 1000)
pwm_backward = GPIO.PWM(BACKWARD_PWM_PIN, 1000)
pwm_forward.start(0)
pwm_backward.start(0)

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

print('Motor control script running...')

try:
    while True:
        move_forward(50)
        time.sleep(5)
        stop_motor()
        time.sleep(2)
        move_backward(50)
        time.sleep(5)
        stop_motor()
        time.sleep(2)

except KeyboardInterrupt:
    print("Script interrupted.")
finally:
    GPIO.cleanup()
