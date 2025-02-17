import logging
import time
import cv2
from picarx_improved import Picarx
from vilib import Vilib

logging.basicConfig(format="%(asctime)s: %(message)s", level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)


class CameraSensing():
    def __init__(self, camera_enabled=False):
        self.robot = Picarx()
        if camera_enabled:
            self.robot.set_cam_tilt_angle(-30)
            time.sleep(0.1)
            Vilib.camera_start(vflip=False, hflip=False)
            Vilib.display(local=True, web=True)
            self.image_name = 'img'
            self.image_path = "picarx"
            time.sleep(0.5)
    
    def get_grayscale_data(self):
        return self.robot.get_grayscale_data()

    def capture_camera_image(self):
        Vilib.take_photo(self.image_name, self.image_path)
    
    def shutdown_camera(self):
        Vilib.camera_close()


class LineInterpretation():
    def __init__(self, sensitivity=2.0, polarity=1):
        self.sensitivity = sensitivity
        self.polarity = polarity

    def calculate_line_position(self, grayscale_data):
        """Identifies sharp changes in sensor values (edges) to determine the vehicle's position relative to the line."""
        if self.polarity == 1:
            grayscale_data = [data - min(grayscale_data) for data in grayscale_data]
        elif self.polarity == -1:
            grayscale_data = [data - max(grayscale_data) for data in grayscale_data]
        
        left, center, right = map(abs, grayscale_data)
        
        if left > right:
            return self.polarity * (1 - (center - left) / max(left, center)) if (center - left) / max(left, center) >= 0 else self.polarity * (abs((center - left) / max(left, center)))
        
        return self.polarity * (-1 + (center - right) / max(right, center)) if (center - right) / max(right, center) >= 0 else self.polarity * ((center - right) / max(right, center))


    def line_position_from_camera(self, image_path, image_name):
        """Uses OpenCV to process the camera image and determine the position of the line."""
        camera_data = cv2.imread(f'{image_path}/{image_name}.jpg')
        _, width, _ = camera_data.shape
        grayscale = cv2.cvtColor(camera_data, cv2.COLOR_BGR2GRAY)
        
        # Thresholding
        _, binary = cv2.threshold(grayscale, 200, 255, cv2.THRESH_BINARY) if self.polarity == 1 else cv2.threshold(grayscale, 50, 255, cv2.THRESH_BINARY_INV)
        
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0.0

        largest_contour = max(contours, key=cv2.contourArea)
        centroid = cv2.moments(largest_contour)
        
        if centroid['m00'] != 0:
            centroid_x = int(centroid['m10'] / centroid['m00'])
            normalized_position = (centroid_x - width / 2) / (width / 2)
            return normalized_position
        return 0.0


class UltrasonicSensing():
    def __init__(self, car):
        self.car = car
        
    def read_distance(self):
        return round(self.car.ultrasonic.read(), 2)


class ObstacleInterpretation():
    def __init__(self, distance_threshold=20):
        self.distance_threshold = distance_threshold
    
    def detect_obstacle(self, ultrasonic_data):
        return ultrasonic_data <= self.distance_threshold and ultrasonic_data >= 0


class Car():
    def __init__(self, scaling_factor=25):
        self.angle_scale = scaling_factor
        
    def follow_line(self, car, line_position):
        if -0.2 < line_position < 0.2:
            car.set_dir_servo_angle(0)
            car.forward(20)
        else:
            car.set_dir_servo_angle(line_position * self.angle_scale)
            car.forward(20)
    
    def stop_motors(self, car):
        car.stop()

    def stop(self, car):
        car.stop()
    

if __name__ == "__main__":
    camera_sensing = CameraSensing(camera_enabled=True)
    line_interpreter = LineInterpretation(sensitivity=2.0, polarity=-1)
    car = Car()

    while True:
        camera_sensing.capture_camera_image()
        line_position = line_interpreter.line_position_from_camera(camera_sensing.image_path, camera_sensing.image_name)
        logging.debug(f"Line position: {line_position}")
        car.follow_line(car=camera_sensing.robot, line_position=line_position)
