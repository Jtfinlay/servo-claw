import cv2
import numpy as np
import ArducamDepthCamera as ac
from gpiozero import Servo
from time import sleep, time

TRIGGER_DISTANCE = 300
SERVO_PIN = 14

# We don't want to open the claw too much
CLAW_MAX_OPEN = 0
CLAW_MAX_CLOSE = 1

class UserRect:
    def __init__(self) -> None:
        self.start_x = 0
        self.start_y = 0
        self.end_x = 0
        self.end_y = 0

    @property
    def rect(self):
        return (
            self.start_x,
            self.start_y,
            self.end_x - self.start_x,
            self.end_y - self.start_y,
        )

    @property
    def slice(self):
        return (slice(self.start_y, self.end_y), slice(self.start_x, self.end_x))

    @property
    def empty(self):
        return self.start_x == self.end_x and self.start_y == self.end_y


confidence_value = 30
selectRect, followRect = UserRect(), UserRect()


def getPreviewRGB(preview: np.ndarray, confidence: np.ndarray) -> np.ndarray:
    preview = np.nan_to_num(preview)
    preview[confidence < confidence_value] = (0, 0, 0)
    return preview


def move_smooth(servo, target, current_pos, time_since_last_draw):
    # Calculate how far we can move based on elapsed time
    MOVEMENT_RATE = 1.0 # Units per second - adjust this to control speed
    max_step = MOVEMENT_RATE * time_since_last_draw
    
    # Calculate the desired movement
    movement = target - current_pos
    # Clamp the movement to our maximum step size
    if abs(movement) > max_step:
        movement = max_step if movement > 0 else -max_step
    
    # Calculate new position
    new_pos = current_pos + movement
    # Clamp to valid range. Round to 2 decimal places
    new_pos = round(max(CLAW_MAX_OPEN, min(CLAW_MAX_CLOSE, new_pos)), 2)
    

    # Update servo
    print("Moving from", current_pos, "to", new_pos)
    servo.value = new_pos
    return new_pos


def on_confidence_changed(value):
    global confidence_value
    confidence_value = value


def usage(argv0):
    print("Usage: python " + argv0 + " [options]")
    print("Available options are:")
    print(" -d        Choose the video to use")


def main():
    print("Arducam Depth Camera Demo.")
    print("  SDK version:", ac.__version__)

    cam = ac.ArducamCamera()
    cfg_path = None
    # cfg_path = "file.cfg"

    # Create servo object connected to GPIO14
    # The min_pulse_width and max_pulse_width values are typical for MG996R servos
    servo = Servo(
        SERVO_PIN,
        min_pulse_width=0.5/1000, # 0.5ms = 500µs pulse for 0 degrees
        max_pulse_width=2.5/1000 # 2.5ms = 2500µs pulse for 180 degrees
    )

    black_color = (0, 0, 0)
    white_color = (255, 255, 255)

    ret = 0
    if cfg_path is not None:
        ret = cam.openWithFile(cfg_path, 0)
    else:
        ret = cam.open(ac.Connection.CSI, 0)
    if ret != 0:
        print("Failed to open camera. Error code:", ret)
        return

    ret = cam.start(ac.FrameType.DEPTH)
    if ret != 0:
        print("Failed to start camera. Error code:", ret)
        cam.close()
        return

    r = cam.getControl(ac.Control.RANGE)

    info = cam.getCameraInfo()
    print(f"Camera resolution: {info.width}x{info.height}")

    cv2.namedWindow("preview", cv2.WINDOW_AUTOSIZE)

    if info.device_type == ac.DeviceType.VGA:
        # Only VGA support confidence
        cv2.createTrackbar(
            "confidence", "preview", confidence_value, 255, on_confidence_changed
        )


    claw_target_position = CLAW_MAX_OPEN
    claw_current_position = claw_target_position
    servo.value = claw_current_position
    last_draw_time = time()

    while True:
        frame = cam.requestFrame(2000)
        if frame is not None and isinstance(frame, ac.DepthData):
            depth_buf = frame.depth_data
            confidence_buf = frame.confidence_data

            result_image = (depth_buf * (255.0 / r)).astype(np.uint8)
            result_image = cv2.applyColorMap(result_image, cv2.COLORMAP_RAINBOW)
            result_image = getPreviewRGB(result_image, confidence_buf)

            cv2.normalize(confidence_buf, confidence_buf, 1, 0, cv2.NORM_MINMAX)

            cv2.imshow("preview_confidence", confidence_buf)

            cv2.rectangle(result_image, followRect.rect, white_color, 1)

            if selectRect.empty:
                selectRect.start_x = result_image.shape[1] // 2 - 4
                selectRect.start_y = result_image.shape[0] // 2 - 4
                selectRect.end_x = result_image.shape[1] // 2 + 4
                selectRect.end_y = result_image.shape[0] // 2 + 4


            if not selectRect.empty:
                cv2.rectangle(result_image, selectRect.rect, black_color, 2)
                distance = np.mean(depth_buf[selectRect.slice]) 

                if distance < TRIGGER_DISTANCE:
                    claw_target_position = CLAW_MAX_CLOSE
                else:
                    claw_target_position = CLAW_MAX_OPEN

                # get time since last draw, and pass it in to move_smooth so that we don't have to sleep
                # and can move the claw smoothly
                if (claw_current_position != claw_target_position):
                    time_since_last_draw = time() - last_draw_time
                    claw_current_position = move_smooth(servo, claw_target_position, claw_current_position, time_since_last_draw)


            cv2.imshow("preview", result_image)
            cam.releaseFrame(frame)

        last_draw_time = time()
        key = cv2.waitKey(1)
        if key == ord("q"):
            break

    cam.stop()
    cam.close()


if __name__ == "__main__":
    main()