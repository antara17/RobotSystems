import time
import logging
from concurrent.futures import ThreadPoolExecutor
from threading import Event
from robot_hat import ADC, Servo
from picarx_improved import Picarx
import numpy as np

# Logging configuration
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

# Shutdown event for graceful termination
shutdown_event = Event()

class Bus:
    """ A simple bus to facilitate communication between threads. """
    def __init__(self):
        self.data = None

    def write(self, value):
        self.data = value

    def read(self):
        return self.data

class Sensing:
    """ Reads grayscale sensor data from ADC pins. """
    def __init__(self, grayscale_pins=['A0', 'A1', 'A2']):
        self.sensors = [ADC(pin) for pin in grayscale_pins]

    def get_grayscale_data(self):
        return [sensor.read() for sensor in self.sensors]

class Interpreter:
    """ Processes sensor data to determine turning proportion for line following. """
    def __init__(self, line_threshold=170):
        self.line_threshold = line_threshold
        self.last_error = 0
        self.sum_error = 0

    def interpret(self, sensor_values, k_p=1.0, k_i=0.0, k_d=0.0):
        sensor_values = np.array(sensor_values)
        line_index = np.argmin(sensor_values)
        position = line_index - len(sensor_values) / 2.0
        turn_proportion = position / (len(sensor_values) / 2.0)

        error = turn_proportion
        pid = k_p * error + k_i * self.sum_error + k_d * (error - self.last_error)
        self.sum_error += error
        self.last_error = error

        return pid

class Controller:
    """ Controls the turning angle of the robot based on interpreted values. """
    def __init__(self, max_turn_angle=40):
        self.max_turn_angle = max_turn_angle
        self.turn_servo = Servo("P2")
        self.turn_servo.angle(0)
        self.car = Picarx()

    def set_turn_proportion(self, turn_proportion):
        turn_angle = self.max_turn_angle * turn_proportion
        self.turn_servo.angle(turn_angle)
        logging.debug(f"Turn Proportion: {turn_proportion}, Turn Angle: {turn_angle}")

    def move_forward(self, speed=40):
        self.car.forward(speed)

    def stop(self):
        self.car.stop()

# Thread functions
def sensor_function(sensor_bus, delay):
    sensor = Sensing()
    while not shutdown_event.is_set():
        sensor_data = sensor.get_grayscale_data()
        sensor_bus.write(sensor_data)
        logging.info(f"Sensor Data: {sensor_data}")
        time.sleep(delay)

def interpreter_function(sensor_bus, interpreter_bus, delay):
    interpreter = Interpreter()
    while not shutdown_event.is_set():
        sensor_data = sensor_bus.read()
        if sensor_data is not None:
            turn_proportion = interpreter.interpret(sensor_data)
            interpreter_bus.write(turn_proportion)
            logging.info(f"Turn Proportion: {turn_proportion}")
        time.sleep(delay)

def control_function(interpreter_bus, delay):
    controller = Controller()
    while not shutdown_event.is_set():
        turn_proportion = interpreter_bus.read()
        if turn_proportion is not None:
            controller.set_turn_proportion(turn_proportion)
            controller.move_forward(20)
        time.sleep(delay)

if __name__ == "__main__":
    sensor_bus = Bus()
    interpreter_bus = Bus()

    sensor_delay = 0.1
    interpreter_delay = 0.1
    control_delay = 0.1

    with ThreadPoolExecutor(max_workers=3) as executor:
        futures = [
            executor.submit(sensor_function, sensor_bus, sensor_delay),
            executor.submit(interpreter_function, sensor_bus, interpreter_bus, interpreter_delay),
            executor.submit(control_function, interpreter_bus, control_delay)
        ]

        try:
            while not shutdown_event.is_set():
                time.sleep(1)
        except KeyboardInterrupt:
            print("Shutting down...")
            shutdown_event.set()
        finally:
            executor.shutdown()
