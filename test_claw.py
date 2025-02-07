from gpiozero import Servo
from time import sleep

SERVO_PIN = 14

# We don't want to open the claw too much
CLAW_MAX_OPEN = 0
CLAW_MAX_CLOSE = 1

def move_smooth(servo, target, current_pos):
    step = 0.02  # Use smaller increments for smoother motion
    if current_pos < target:
        while current_pos < target:
            current_pos = min(current_pos + step, CLAW_MAX_CLOSE)
            servo.value = current_pos
            sleep(0.01)  # Slightly longer delay for stability

    else:
        while current_pos > target:
            current_pos = max(current_pos - step, CLAW_MAX_OPEN)
            servo.value = current_pos
            sleep(0.01)

    return current_pos  # Return the final position

# Create servo object connected to GPIO14
# The min_pulse_width and max_pulse_width values are typical for MG996R servos
servo = Servo(
    SERVO_PIN,
    min_pulse_width=0.5/1000, # 0.5ms = 500µs pulse for 0 degrees
    max_pulse_width=2.5/1000 # 2.5ms = 2500µs pulse for 180 degrees
)

try:

    current_position = CLAW_MAX_OPEN
    servo.value = current_position

    while True:
        # Wait for keyboard input. Once received, open or close the claw
        key = input("Enter 'o' to open the claw, 'c' to close it: ")
        if key == 'o':
            current_position = move_smooth(servo, CLAW_MAX_OPEN, current_position)
        elif key == 'c':
            current_position = move_smooth(servo, CLAW_MAX_CLOSE, current_position)


except KeyboardInterrupt:
    # Cleanup happens automatically when the Servo object is destroyed
    servo.detach()