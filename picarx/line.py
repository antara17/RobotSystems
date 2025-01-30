import time
import os
import numpy as np
import logging
from robot_hat import ADC, Servo, fileDB
from robot_hat import Grayscale_Module
from robot_hat.utils import reset_mcu
from picarx_improved import Picarx

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

reset_mcu()
time.sleep(0.2)

class Sensing:
    CONFIG = '/opt/picar-x/picar-x.conf'

    def __init__(self, grayscale_pins: list = ['A0', 'A1', 'A2'], config: str = CONFIG):
        reset_mcu()
        time.sleep(0.2)
        self.config_file = fileDB(config, 777, os.getlogin())
        self.adc0, self.adc1, self.adc2 = [ADC(pin) for pin in grayscale_pins]
        start  =time.time()
        while (time.time()-start < 3):
            self.get_grayscale_data()
        

    def get_grayscale_data(self):
        # sensor_values = list.copy(self.grayscale.read())
        sensor_values = [self.adc0.read(),self.adc1.read(),self.adc2.read()]
        logging.debug(f"Sensor Values: {sensor_values}")

        return sensor_values

class Interpreter:
    def __init__(self, sensitivity=20, polarity="dark", line_threshold=170):
        self.sensitivity = sensitivity
        self.polarity = polarity
        self.line_threshold = line_threshold
        self.last_error = 0
        self.sum_error = 0
        self.sensor_with_line_last_detected = 1

    def has_no_significant_difference(self, sensor_reading):
        # if working with light line, then flip sensor readings such that the max value is the line
        # get the reading that is suspect to be the line
        sensor_reading = np.array(sensor_reading)
        line_index = np.argmin(sensor_reading)
        floor_index = [0, 1, 2]
        floor_index.remove(line_index)
        # check that the difference of the line reading to the average surrounding reading is at least more than the threshold
        line_reading = sensor_reading[line_index]
        avg_floor_reading = np.mean(sensor_reading[floor_index])
        floor_line_difference = np.abs(line_reading - avg_floor_reading)
        #return whether the difference is significant
        if(floor_line_difference <= (self.line_threshold)):
            return(True)
        else:
            print(floor_line_difference)
            self.sensor_with_line_last_detected = line_index
            return(False)

    def interpret(self, sensor_values, scaling_function="PID", k_p=1.0, k_i=0.0, k_d=0.0):
        # Average the 3 grayscale sensor readings
        if self.has_no_significant_difference(sensor_values):
            print("last line detect:",(self.sensor_with_line_last_detected-1))
            return (0.7*self.sensor_with_line_last_detected-1)

        # Detect the line by checking the averaged sensor values against a threshold
        if self.polarity == "dark":
            line_index = np.argmin(sensor_values)  # Get the index of the minimum value (darkest sensor)
        else:
            line_index = np.argmax(sensor_values)  # Get the index of the maximum value (lightest sensor)

        # Calculate the position of the line relative to the sensors
        position = line_index - len(sensor_values) / 2.0  # Centering the sensor range
        turn_proportion = position / (len(sensor_values) / 2.0)  # Normalize to -1 to 1

        # Apply PID control if scaling function is PID
        if scaling_function == "PID":
            error = turn_proportion
            pid = k_p * error + k_i * (self.sum_error + error) + k_d * (error - self.last_error)
            turn_proportion = pid
            self.sum_error += error
            self.last_error = error

        # If PID is not used, use simple proportional control
        elif scaling_function == "linear":
            turn_proportion = position / 1.0

        # Return the final turn proportion to control the servo
        return turn_proportion
    
    

class Controller:
    def __init__(self, max_turn_angle=40):
        self.max_turn_angle = max_turn_angle
        self.turn_servo = Servo("P2")
        self.turn_servo.angle(0)

    def set_turn_proportion(self, turn_proportion):
        turn_angle = float(self.max_turn_angle * turn_proportion)
        self.turn_servo.angle(turn_angle)
        logging.debug(f"Turn Proportion: {turn_proportion}, Turn Angle: {turn_angle}")

class LineFollower:
    def __init__(self):
        self.car = Picarx()
        self.sensor = Sensing()
        self.interpreter = Interpreter()
        self.controller = Controller()
        

    def follow_line(self):
        try:
            while True:
                # Get sensor data
                sensor_values = self.sensor.get_grayscale_data()
                

                # Interpret the sensor values to determine how much to turn
                turn_proportion = self.interpreter.interpret(sensor_values, scaling_function="PID", k_p=1.0, k_i=0.0, k_d=0.0)

                # Control the car's turn based on the turn proportion
                self.controller.set_turn_proportion(turn_proportion)

                # Move the car forward at a constant speed
                self.car.forward(10)
                time.sleep(0.05)

        except KeyboardInterrupt:
            logging.info("Line following stopped.")
            self.car.stop()

if __name__ == "__main__":
    line_follower = LineFollower()
    line_follower.follow_line()
