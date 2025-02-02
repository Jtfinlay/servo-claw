import RPi.GPIO as GPIO
import time

class ServoController:
    def __init__(self, pin=12):  # GPIO 12 (Pin 32) is one of the PWM capable pins
        self.servo_pin = pin
        # SG90 specific duty cycle values
        self.min_duty = 2.5   # 0 degrees
        self.mid_duty = 7.5   # 90 degrees
        self.max_duty = 12.5  # 180 degrees
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)        # Use BCM pin numbering
        GPIO.setup(self.servo_pin, GPIO.OUT)
        
        # Initialize PWM at 50Hz (standard for SG90)
        self.pwm = GPIO.PWM(self.servo_pin, 50)
        self.pwm.start(self.mid_duty)  # Start at center position
        time.sleep(0.5)  # Allow servo to center
        self.pwm.ChangeDutyCycle(0)  # Stop pulses
    
    def angle_to_duty_cycle(self, angle):
        """Convert angle (0-180) to duty cycle for SG90."""
        if angle < 0:
            angle = 0
        elif angle > 180:
            angle = 180
            
        duty_cycle = self.min_duty + (angle/180.0) * (self.max_duty - self.min_duty)
        return duty_cycle
    
    def set_angle(self, angle):
        """Set servo to specified angle."""
        duty_cycle = self.angle_to_duty_cycle(angle)
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.3)  # Allow time for SG90 to move
        self.pwm.ChangeDutyCycle(0)  # Stop pulses to prevent jitter
    
    def sweep(self, start_angle=0, end_angle=180, step=10, delay=0.5):
        """Sweep servo between specified angles."""
        if start_angle > end_angle:
            step = -abs(step)
        
        for angle in range(start_angle, end_angle + (1 if step > 0 else -1), step):
            self.set_angle(angle)
            time.sleep(delay)
    
    def cleanup(self):
        """Clean up GPIO on exit."""
        self.pwm.stop()
        GPIO.cleanup()

# Example usage
if __name__ == "__main__":
    try:
        # Create servo controller instance
        servo = ServoController(pin=12)  # Using GPIO 12 (Pin 32)
        
        print("Testing SG90 servo movement...")
        
        # Test key positions
        positions = [
            (90, "center"),
            (0, "full counter-clockwise"),
            (180, "full clockwise"),
            (90, "back to center")
        ]
        
        for angle, description in positions:
            print(f"Moving to {angle} degrees ({description})")
            servo.set_angle(angle)
            time.sleep(1)
        
        print("\nPerforming smooth sweep...")
        servo.sweep(0, 180, step=10, delay=0.1)
        
    except KeyboardInterrupt:
        print("\nProgram stopped by user")
    finally:
        # servo.cleanup()
        print("GPIO cleanup completed")