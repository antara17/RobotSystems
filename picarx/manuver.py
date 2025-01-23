# maneuver.py
import time
from picarx_improved import Picarx



def move_forward(px, speed, duration):
    """
    Move the car forward at the given speed for the specified duration.
    """
    print(f"Moving forward with speed {speed}")
    px.set_dir_servo_angle(-30)
    px.forward(speed)
    time.sleep(20)
    px.stop()

def move_backward(px, speed, duration):
    
    print(f"Moving backward with speed {speed}")
    px.backward(speed)
    time.sleep(duration)
    px.stop()

def parallel_park_left(px, speed):
    
    print("Starting parallel parking left")
    px.set_dir_servo_angle(-30)
    px.backward(speed) 
    time.sleep(1)  
    px.stop()

    px.set_dir_servo_angle(30)
    px.backward(speed)
    time.sleep(1) 
    px.stop()

    
    px.set_dir_servo_angle(0)  
    px.backward(speed)
    time.sleep(0.5)  
    px.stop()
    print("Parallel parking to the left complete.")


    print("Parking completed.")
    px.stop()



def parallel_park_right(px, speed):
 
    print("Starting parallel parking right")

    px.set_dir_servo_angle(30)  
    px.backward(speed) 
    time.sleep(1)  
    px.stop()

    px.set_dir_servo_angle(-30)  
    px.backward(speed)
    time.sleep(1)  
    px.stop()

    px.set_dir_servo_angle(0)
    px.backward(speed)
    time.sleep(0.5) 
    px.stop()

    print("Parallel parking to the right complete.")
    px.stop()




def three_point_turn_left(px, speed):
   
    px.set_dir_servo_angle(-45) 
    px.forward(speed)
    time.sleep(1)  
    px.set_dir_servo_angle(45)  
    px.backward(speed)
    time.sleep(1) 
    px.set_dir_servo_angle(-45)  
    px.forward(speed)
    time.sleep(1)
    px.stop()

def three_point_turn_right(px, speed):
   
    px.set_dir_servo_angle(45)  
    px.forward(speed)
    time.sleep(1) 
    px.set_dir_servo_angle(-45)  
    px.backward(speed)
    time.sleep(1) 
    px.set_dir_servo_angle(45)  
    px.forward(speed)
    time.sleep(1)
    px.stop()

def main():
    """
    Main loop to allow user to input commands for maneuvers.
    """
    px = Picarx()  # Initialize Picarx
    while True:
        print("\nEnter a maneuver command (or 'exit' to quit):")
        print("1: Move Forward")
        print("2: Move Backward")
        print("3: Parallel Park Left")
        print("4: Parallel Park Right")
        print("5: Three-Point Turn Left")
        print("6: Three-Point Turn Right")
        user_input = input("Enter your choice: ").strip()

        if user_input == '1':
            move_forward(px, 50, 2)  # Move forward for 2 seconds
        elif user_input == '2':
            move_backward(px, 50, 2)  # Move backward for 2 seconds
        elif user_input == '3':
            parallel_park_left(px, 50)  # Parallel park left
        elif user_input == '4':
            parallel_park_right(px, 50)  # Parallel park right
        elif user_input == '5':
            three_point_turn_left(px, 50)  # Three-point turn left
        elif user_input == '6':
            three_point_turn_right(px, 50)  # Three-point turn right
        elif user_input.lower() == 'exit':
            print("Exiting...")
            break
        else:
            print("Invalid command. Please try again.")

if __name__ == "__main__":
    main()

