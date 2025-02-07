from gpiozero import Servo
from time import sleep

SERVO_PIN = 14

# Create servo object connected to GPIO14
# The min_pulse_width and max_pulse_width values are typical for SG90 servos
servo = Servo(
    SERVO_PIN,
    min_pulse_width=0.5/1000, # 0.5ms = 500µs pulse for 0 degrees
    max_pulse_width=2.5/1000 # 2.5ms = 2500µs pulse for 180 degrees
)

try:
    while True:
        # Move to minimum position (-1 corresponds to 0 degrees)
        servo.value = -1
        sleep(1)
        
        # Move to middle position (0 corresponds to 90 degrees)
        servo.value = 0
        sleep(1)
        
        # Move to maximum position (1 corresponds to 180 degrees)
        servo.value = 1
        sleep(1)

except KeyboardInterrupt:
    # Cleanup happens automatically when the Servo object is destroyed
    servo.detach()