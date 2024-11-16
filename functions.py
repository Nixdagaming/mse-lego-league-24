from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import math

# Define parameters
axle_track = 153  # Length between contact patches of wheels (in mm)
wheel_diameter = 56  # Diameter of the wheel (in mm)

"""
Ports:

S1 - Color sensor
S2 - Ultrasonic sensor
S3 - Gyroscope

A - Left motor
B - Right motor
"""
# Initialize objects
ev3 = EV3Brick()
color = ColorSensor(Port.S1)
ultrasonic = UltrasonicSensor(Port.S2)
gyro = GyroSensor(Port.S3)
wheel_left = Motor(Port.A, Direction.COUNTERCLOCKWISE)
wheel_right = Motor(Port.B)
drive_base = DriveBase(wheel_left, wheel_right, wheel_diameter, axle_track)  # Create a drive base object

# Set drive base settings (speed, acceleration)
drive_base.settings(200, 1000, 45, 1000) #mm/s, mm/s^2, deg/s, deg/s^2

# Function to get the clamped angle of the gyroscope
def get_gyro_angle():
    return (gyro.angle() + 180) % 360 - 180

class Robot:
    def __init__(self):
        self.drive_base = drive_base
        self.ultrasonic = ultrasonic
        self.color = color
    
    # Function to drive the robot to a specific position
    def drive_to(self, target_x, target_y):
        
        # Get the current position of the robot
        current_pos = self.get_current_position()
        
        # Calculate the difference between the target and current positions
        delta_x = (target_x - current_pos[0]) * 10
        delta_y = (target_y - current_pos[1]) * 10
        
        # Calculate the angle to turn
        theta = math.atan2(delta_y, delta_x)
        angle_to_turn = math.degrees(theta)
        
        # Turn to face the target position
        self.drive_base.turn(angle_to_turn - get_gyro_angle())
        
        # Drive straight to the target position
        distance = math.sqrt(delta_x**2 + delta_y**2)
        self.drive_base.straight(distance)
    
    # Function to follow a line
    def one_sensor_follow_line(self):
        while True:
            # Calculate the error
            error = (self.color.reflection() - 50) * 2
            
            # Calculate the speed adjustment based on the error magnitude
            base_speed = max(50, 150 - abs(error) * 2)  # Reduce speed more for larger errors
            turn_rate = error * 3  # Turning proportional to the error
            
            # Drive the robot
            self.drive_base.drive(base_speed, turn_rate)
            
            # Debugging information
            print(f"Error: {error}, Base Speed: {base_speed}, Turn Rate: {turn_rate}")

    def face(self, angle: int):
        """
        Turn the robot to face a specific direction.
        
        Args:
            angle (int): The desired angle in degrees.
        """
        # Calculate the difference between the desired angle and the current gyroscope angle
        turn_angle = (angle - get_gyro_angle()) % 360
        
        # If the calculated turn angle is greater than 180 degrees, adjust it to be less than 180 degrees
        if turn_angle > 180:
            turn_angle -= 360
        
        # Turn the robot by the calculated angle
        self.drive_base.turn(turn_angle)

    def get_current_position(self):
        
        x, y = None, None
        
        while x is None or y is None:
            # Turn to detect the targets
            self.drive_base.drive(0, -45)
            if get_gyro_angle() == -90:
                # If gyroscope is pointing west, read ultrasonic sensor
                y = self.ultrasonic.distance() / 10
            elif get_gyro_angle() in [-180, 180]:
                # If gyroscope is pointing south, read ultrasonic sensor again
                x = self.ultrasonic.distance() / 10
        
        # Stop driving and turn off motors
        self.drive_base.drive(0, 0)
        self.drive_base.stop()
        
        print("Current Position:", x, y)
        return x, y

#* MAIN FUNCTION
def main():
    robot = Robot()
    pass

if __name__ == "__main__":
    main()
