import time
import logging
from rossros import Bus, Producer, ConsumerProducer, Consumer, Timer, runConcurrently
from thread import CameraSensing, LineInterpretation, Car, UltrasonicSensing, ObstacleInterpretation

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")


camera_sensing = CameraSensing(camera_enabled=True)
ultrasonic_sensing = UltrasonicSensing(car=camera_sensing.robot)
line_interpreter = LineInterpretation(sensitivity=2.0, polarity=-1)
obstacle_interpreter = ObstacleInterpretation(distance_threshold=20)
car = Car()


sensor_bus = Bus(name="Sensor Bus")
ultrasonic_bus = Bus(name="Ultrasonic Bus")
interpretation_bus = Bus(name="Interpretation Bus")
obstacle_bus = Bus(name="Obstacle Detection Bus")
termination_bus = Bus(name="Termination Bus")

sensor_delay = 0.1
interpretation_delay = 0.1
controller_delay = 0.1
run_duration = 5


def get_grayscale_data():
    grayscale = camera_sensing.robot.get_grayscale_data()
    logging.info(f"Grayscale data: {grayscale}")
    return grayscale

def get_ultrasonic_data():
    ultrasonic_distance = ultrasonic_sensing.read_distance()
    logging.info(f"Ultrasonic distance: {ultrasonic_distance}")
    return ultrasonic_distance

# Functions for interpreting data
def interpret_line_position(grayscale_data):
    try:
        line_position = line_interpreter.calculate_line_position(grayscale_data)
        logging.info(f"Line position: {line_position}")
        return line_position
    except Exception as e:
        logging.error(f"Error interpreting line position: {e}")
        return 0

def interpret_obstacle(ultrasonic_data):
    try:
        obstacle_detected = obstacle_interpreter.detect_obstacle(ultrasonic_data)
        logging.info(f"Obstacle detected: {obstacle_detected} (distance: {ultrasonic_data})")
        return obstacle_detected
    except Exception as e:
        logging.error(f"Error interpreting obstacle: {e}")
        return True

# Functions for controlling the vehicle
def control_vehicle(line_position):
    logging.info(f"Moving vehicle based on line position: {line_position}")
    car.follow_line(car=camera_sensing.robot, line_position=line_position)

def control_vehicle_obstacle(obstacle_present):
    if obstacle_present:
        logging.info(f"Obstacle detected! Stopping motors.")
        car.stop_motors(car=camera_sensing.robot)
        time.sleep(5)

# Runtime setup using the Timer class
timer = Timer(
    output_buses=termination_bus,
    duration=run_duration,
    delay=0.1,
    termination_buses=termination_bus,
    name="Runtime Timer"
)

# Producer and consumer setup for the message buses
sensor_producer = Producer(
    producer_function=get_grayscale_data,
    output_buses=sensor_bus,
    delay=sensor_delay,
    termination_buses=termination_bus,
    name="Sensor Producer"
)

ultrasonic_producer = Producer(
    producer_function=get_ultrasonic_data,
    output_buses=ultrasonic_bus,
    delay=sensor_delay,
    termination_buses=termination_bus,
    name="Ultrasonic Producer"
)

line_interpreter_consumer = ConsumerProducer(
    consumer_producer_function=interpret_line_position,
    input_buses=sensor_bus,
    output_buses=interpretation_bus,
    delay=interpretation_delay,
    termination_buses=termination_bus,
    name="Line Interpreter"
)

obstacle_detector_consumer = ConsumerProducer(
    consumer_producer_function=interpret_obstacle,
    input_buses=ultrasonic_bus,
    output_buses=obstacle_bus,
    delay=interpretation_delay,
    termination_buses=termination_bus,
    name="Obstacle Interpreter"
)

vehicle_controller_consumer = Consumer(
    consumer_function=control_vehicle,
    input_buses=interpretation_bus,
    delay=controller_delay,
    termination_buses=termination_bus,
    name="Vehicle Controller"
)

try:
    runConcurrently([sensor_producer, ultrasonic_producer, line_interpreter_consumer, obstacle_detector_consumer, vehicle_controller_consumer, timer])
except KeyboardInterrupt:
    car.stop(car=camera_sensing.robot)
    camera_sensing.shutdown_camera()
finally:
    car.stop(car=camera_sensing.robot)
    camera_sensing.shutdown_camera()
