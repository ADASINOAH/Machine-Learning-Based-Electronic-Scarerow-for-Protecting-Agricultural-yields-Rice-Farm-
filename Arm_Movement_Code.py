#The servo motor control Code
import RPi.GPIO as GPIO
import time
# Configure GPIO
SERVO_PIN = 17  # Update with your actual pin number
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)
# Configure PWM
pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz frequency (common for servos)
pwm.start(7.5)  # Initial position (neutral)
def set_angle(angle):
    duty_cycle = angle / 18 + 2.5
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(1)
try:
    while True:
        # Move servo to 0 degrees
        set_angle(0)
        time.sleep(1)

        # Move servo to 90 degrees
        set_angle(90)
        time.sleep(1)
        
        # Move servo to 180 degrees
        set_angle(180)
        time.sleep(1)
except KeyboardInterrupt:
    pass

pwm.stop()
GPIO.cleanup()

