import time
import cv2
import numpy as np
import logging
from picarx_improved import Picarx
from robot_hat import Servo
from picamera2 import Picamera2
import libcamera

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

class LineFollowerWithCamera:
    def __init__(self, max_turn_angle=90, k_p=0.15):  # Increased max turn angle and k_p
        self.car = Picarx()
        self.max_turn_angle = max_turn_angle
        self.k_p = k_p  # Higher proportional gain for steeper turns
        self.turn_servo = Servo("P2")
        self.turn_servo.angle(0)
        self.tilt_servo = Servo("P1")
        self.tilt_servo.angle(0)
        
        # Initialize Camera
        self.picam2 = Picamera2()
        preview_config = self.picam2.preview_configuration
        preview_config.size = (640, 480)
        preview_config.format = 'RGB888'
        preview_config.colour_space = libcamera.ColorSpace.Sycc()
        preview_config.buffer_count = 4
        preview_config.controls.FrameRate = 30
        
        # Start Camera
        try:
            self.picam2.start()
        except Exception as e:
            logging.error(f"Error starting camera: {e}")
            exit(1)

    def get_frame(self):
        self.tilt_servo.angle(30)
        # Capture an image from the camera
        img = self.picam2.capture_array()

        return img

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)  # Reduce noise
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        gray = clahe.apply(gray) 
        gray =  gray[frame.shape[0]//4*3:-1,:]

        _, thresh = cv2.threshold(gray, 85, 255, cv2.THRESH_BINARY_INV)

        cv2.imwrite("debug_frame.jpg", gray)
        cv2.imwrite("debug_thresh.jpg", thresh)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)



        if not contours:
            logging.debug("No contours found.")
            return 0

        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
        else:
            cx = frame.shape[1] // 2  # Default to center if no contour

        error = (cx - (frame.shape[1] // 2)) / (frame.shape[1] // 2)
        logging.debug(f"cx: {cx}, error: {error}")
        return error


    def set_turn_proportion(self, error, error_threshold=0.1):
        """ Adjusts servo angle based on error using a proportional controller.
            If the error is small, keep going straight.
        """
        if abs(error) < error_threshold:
            turn_angle = 0  # Keep the car straight
            logging.debug(f"Error: {error},Turn Angle: {turn_angle}")
            self.turn_servo.angle(turn_angle)

        else:
            # Exponential scaling for sharper turns when necessary
            MIN_TURN_ANGLE = 20  # Ensures sharper turns
            turn_proportion = -self.k_p * error
            turn_angle = self.max_turn_angle * turn_proportion

            # Ensure minimum turn angle for larger deviations
            if abs(turn_angle) < MIN_TURN_ANGLE:
                turn_angle = -MIN_TURN_ANGLE if turn_angle > 0.02 else MIN_TURN_ANGLE

            # Clamp turn angle within limits
            turn_angle = max(-self.max_turn_angle, min(self.max_turn_angle, turn_angle))
            logging.debug(f"Error: {error}, Turn Proportion: {turn_proportion}, Turn Angle: {turn_angle}")
            self.turn_servo.angle(turn_angle)

        
            

    def follow_line(self):
        try:
            while True:
                # Get the camera frame
                frame = line_follower.get_frame()
                cv2.imwrite("test_image.jpg", frame)
                logging.info("Captured test_image.jpg")


                # Process the frame to get the line error
                error = self.process_frame(frame)

                # Adjust the car's steering based on the error
                self.set_turn_proportion(error)

                # Move the car forward at a constant speed
                self.car.forward(20)  # Speed can be increased if necessary
                time.sleep(0.05)

        except KeyboardInterrupt:
            logging.info("Line following stopped.")
            self.car.stop()
            self.picam2.stop()

if __name__ == "__main__":
    line_follower = LineFollowerWithCamera(max_turn_angle=90, k_p=0.15)  # Sharper turns with higher parameters
    line_follower.follow_line()
