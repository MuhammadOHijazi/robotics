<<<<<<< HEAD
from controller import Robot, Camera
import math
import time 


class Box:
    def __init__(self, color, x, y, z):
        self.color = color
        self.x = x
        self.y = y
        self.z = z

class RobotController(Robot):

    def __init__(self):
        super().__init__()

        # Setup for wheels
        self.timestep = int(self.getBasicTimeStep())
        self.wheels = [self.getDevice(f"wheel{i}") for i in range(1, 5)]
=======
from controller import Robot
<<<<<<< HEAD
from collections import deque
import numpy as np
=======
>>>>>>> 5eb0d570632cdb4bc6a07f016fa7c45759e525ec
import math

class RobotController(Robot):
    def __init__(self):
        Robot.__init__(self)

        # Setup for wheels
        self.timestep = int(self.getBasicTimeStep())
        self.wheels = [self.getDevice("wheel" + str(i)) for i in range(1, 5)]
        self.detected_box_color = None
        self.is_picking_up = False
        self.pickup_state = 'align'
        
        self.camera = self.getDevice("camera")
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)
>>>>>>> 1c93a9aca767828fd6e12460e7e567e07b3bef3a
        for wheel in self.wheels:
            wheel.setPosition(float("inf"))
            wheel.setVelocity(0)

<<<<<<< HEAD
        # Setup wheel motors
        self.front_left_motor = self.getDevice('wheel1')
        self.front_right_motor = self.getDevice('wheel2')
        self.back_left_motor = self.getDevice('wheel3')
        self.back_right_motor = self.getDevice('wheel4')
        
        # Set wheel motors position and velocity
        motors = [self.front_left_motor, self.front_right_motor, self.back_left_motor, self.back_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)

        # Setup for sensors
        self.sensors_index = range(1, 12)
        self.sensors = [self.getDevice(f"ir{index}") for index in self.sensors_index]
        for sensor in self.sensors:
            sensor.enable(self.timestep)

        self.sensors_coefficient = [5000, 4500, 4000, 3000, 2000, 1000, -1000, -2000, -3000, -4000, -4500, -5000]
        
        # setup static / constant values
        self.max_velocity = 14.81
        self.movement_velocity = 4.0
        self.rotation_velocity = 2.0 

        # PID coefficients
        self.Kp = 0.001
        self.Kd = 0.0002
        self.last_error = 0

        # Setup for camera
        self.camera = self.getDevice("camera")
        self.camera.enable(self.timestep)
        self.camera.recognitionEnable(self.timestep)

        # Setup for color queue
        self.color_queue = []

        #! Initialize gripper motors.
        self.finger1 = self.getDevice("finger::left")
        self.finger2 = self.getDevice("finger::right")
        self.finger1.setVelocity(1.5)
        self.finger2.setVelocity(1.5) # 0.03
        self.fingerMinPosition = self.finger1.getMinPosition()
        self.fingerMaxPosition = self.finger1.getMaxPosition()

        # setup Arms sensors
        self.armPositionSensors = []
        self.armPositionSensors.append(self.getDevice("arm1sensor"))
        self.armPositionSensors.append(self.getDevice("arm2sensor"))
        self.armPositionSensors.append(self.getDevice("arm3sensor"))
        self.armPositionSensors.append(self.getDevice("arm4sensor"))
        self.armPositionSensors.append(self.getDevice("arm5sensor"))
        for sensor in self.armPositionSensors:
            sensor.enable(self.timestep)

        # setup Arms Motors
=======
        self.max_velocity = 14.81
<<<<<<< HEAD
        self.movement_velocity = 4.0  # تقليل السرعة
        self.movement_velocity = 6.28  

        # Setup for sensors
        self.road_map = ["forward", "left", "forward", "right", "forward"]  # Example road map
        self.current_step = 0 
=======
        self.MOVEMENT_VELOCITY = 12
        self.movement_velocity = 6.28  

        # Setup for sensors
>>>>>>> 5eb0d570632cdb4bc6a07f016fa7c45759e525ec
        self.sensors_index = [1, 2, 3, 4, 5, 6, 7, 8,9,10,11,12]
        self.sensors = []
        self.sensors_coefficient = [5000,4500,4000, 3000, 2000, 1000, -1000, -2000, -3000, -4000,-4500,-5000]
        for index in self.sensors_index:
            sensor = self.getDevice("ir" + str(index))
            sensor.enable(self.timestep)
            self.sensors.append(sensor)
>>>>>>> 1c93a9aca767828fd6e12460e7e567e07b3bef3a
        self.armMotors = []
        self.armMotors.append(self.getDevice("arm1"))
        self.armMotors.append(self.getDevice("arm2"))
        self.armMotors.append(self.getDevice("arm3"))
        self.armMotors.append(self.getDevice("arm4"))
        self.armMotors.append(self.getDevice("arm5"))
        self.armMotors[0].setVelocity(1.5) # maxVelocity = 1.5
        self.armMotors[1].setVelocity(1.5)
        self.armMotors[2].setVelocity(1.5)
        self.armMotors[3].setVelocity(0.5)
        self.armMotors[4].setVelocity(1.5)

<<<<<<< HEAD
        self.COUNTER_PICKK_UP=0
        self.COUNTER_REV_PICKK_UP=0

        # Box Queue
        self.box_queue = {
            "Green": [Box("Green", -3.74 - 0.3 * i, -4.08, 0.0749) for i in range(5)],
            "Yellow": [Box("Yellow", -3.74 - 0.3 * i, -1.46, 0.0749) for i in range(5)],
            "Blue": [Box("Blue", -3.74 - 0.3 * i, 1.23, 0.0749) for i in range(5)],
            "Red": [Box("Red", -3.74 - 0.3 * i, 3.77, 0.0749) for i in range(5)],
        }
        
        # setup GPS for detect the cooridinates of the Kuka in the World
        self.gps = self.getDevice("gps")
        self.gps.enable(self.timestep)

        # define the initial cooridinates for the Kuka 1 in the World
        self.current_x = -1.17477
        self.current_y = -2.04883
        self.current_z = 0.151811

        # Initialize Compass
        self.compass = self.getDevice("compass")
        if self.compass is None:
            raise Exception("Compass device not found! Check robot configuration.")
        # Enable Compass
        self.compass.enable(self.timestep)

    def clamp_speed(self, speed):
        return max(min(speed, self.max_velocity), -self.max_velocity)

    def read_sensors_value(self):
        value = 0
        for index, sensor in enumerate(self.sensors):
            sensor_value = sensor.getValue()
            if 300 <= sensor_value <= 550:
                value += self.sensors_coefficient[index]
        return value

    def steering(self, left_speed, right_speed):
        left_speed = self.clamp_speed(left_speed)
        right_speed = self.clamp_speed(right_speed)
        self.wheels[0].setVelocity(left_speed)
        self.wheels[2].setVelocity(left_speed)
        self.wheels[1].setVelocity(right_speed)
        self.wheels[3].setVelocity(right_speed)

    # detect the Colors of the plane in the ground to detect the colors Queue 
    def detect_color(self):
        image = self.camera.getImage()
        width = self.camera.getWidth()
        height = self.camera.getHeight()

        # Initialize color counters
        red_count = green_count = blue_count = yellow_count = 0
        total_pixels = (width // 2) * (height // 2)  # Adjust for sampling

        # Iterate over the sampled image
        for x in range(0, width, 2):  # Skip every second pixel
            for y in range(0, height, 2):
                red = Camera.imageGetRed(image, width, x, y)
                green = Camera.imageGetGreen(image, width, x, y)
                blue = Camera.imageGetBlue(image, width, x, y)

                if red > 200 and green > 200 and blue < 100:
                    yellow_count += 1
                elif red > 200 and green < 100 and blue < 100:
                    red_count += 1
                elif red < 100 and green > 200 and blue < 100:
                    green_count += 1
                elif red < 100 and green < 100 and blue > 200:
                    blue_count += 1

        # Calculate percentages
        red_percentage = (red_count / total_pixels) * 100
        green_percentage = (green_count / total_pixels) * 100
        blue_percentage = (blue_count / total_pixels) * 100
        yellow_percentage = (yellow_count / total_pixels) * 100

        # Detect the dominant color and update the queue
        if red_percentage > 1 and "Red" not in self.color_queue:
            self.color_queue.append("Red")
            print("*********Red color detected**********")
        elif green_percentage > 1 and "Green" not in self.color_queue:
            self.color_queue.append("Green")
            print("*********Green color detected*************")
        elif blue_percentage > 1 and "Blue" not in self.color_queue:
            self.color_queue.append("Blue")
            print("**********Blue color detected***********")
        elif yellow_percentage > 1 and "Yellow" not in self.color_queue:
            self.color_queue.append("Yellow")
            print("*********Yellow color detected************")
        else:
            print("**********No significant color detected**************")

    def print_color_queue(self):
        print("FIRST ROBOT, \tColor Queue:", self.color_queue)

    def get_color_queue(self):
        if len(self.color_queue) <4 and self.current_y <= -0.2:
            print('starting our new movements')
            self.detect_color()
            self.print_color_queue()

    # get informations from the Compass and convert it to know the angle position of the Kuka Robot
    def get_compass_heading(self):
        """
        Retrieve the robot's current compass heading in degrees.
        Assumes a compass sensor is used and returns a value between 0 and 360.
        """
        compass_values = self.compass.getValues()  # Compass sensor values
        heading = math.atan2(compass_values[0], compass_values[2])
        heading_degrees = (heading * 180 / math.pi) % 360  # Convert radians to degrees
        return heading_degrees

=======

        self.armPositionSensors = []
        self.armPositionSensors.append(self.getDevice("arm1sensor"))
        self.armPositionSensors.append(self.getDevice("arm2sensor"))
        self.armPositionSensors.append(self.getDevice("arm3sensor"))
        self.armPositionSensors.append(self.getDevice("arm4sensor"))
        self.armPositionSensors.append(self.getDevice("arm5sensor"))
        for sensor in self.armPositionSensors:
            sensor.enable(self.timestep)

        #! Initialize gripper motors.
        self.finger1 = self.getDevice("finger::left")
        self.finger2 = self.getDevice("finger::right")
        self.finger1.setVelocity(1.5)
        self.finger2.setVelocity(1.5) # 0.03
        self.fingerMinPosition = self.finger1.getMinPosition()
        self.fingerMaxPosition = self.finger1.getMaxPosition()
        
        # PID coefficients
<<<<<<< HEAD
        self.Kp = 0.001
        self.Kd = 0.0002 
=======
        self.Kp = 0.0015
        self.Kd = 0.00015 
>>>>>>> 5eb0d570632cdb4bc6a07f016fa7c45759e525ec
        self.last_error = 0
        # PID error terms

        # Initialize wheel motors
        self.front_left_motor = self.getDevice('wheel1')
        self.front_right_motor = self.getDevice('wheel2')
        self.back_left_motor = self.getDevice('wheel3')
        self.back_right_motor = self.getDevice('wheel4')
        
        # Set wheel motors position and velocity
        motors = [self.front_left_motor, self.front_right_motor, self.back_left_motor, self.back_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
        self.COUNTER_PICKK_UP=0
        self.COUNTER_REV_PICKK_UP=0
        self.WALL_THRESHOLD = 900
        self.MOVEMENT_VELOCITY = 12
        self.turn_speed = math.pi / 2
        self.base_speed = 6.28
        self.wall_sensors = []
        self.sensors_index = ['if'+str(i) for i in range(1, 14)]
        for sensor_name in self.sensors_index:
            sensor = self.getDevice(sensor_name)
            sensor.enable(self.timestep)
            self.wall_sensors.append(sensor)  
        # Set initial values for control variables
<<<<<<< HEAD
        self.turn_duration = 500  # Placeholder: the duration of a turn (to be calibrated)
        self.turn_around_duration = 3000  # Placeholder: the duration of a 180-degree turn (to be calibrated)
        self.step(self.timestep)




    def execute_next_instruction(self):
        if self.current_step >= len(self.road_map):
            print("All roadmap instructions executed.")
            return

        instruction = self.road_map[self.current_step]
        print(f"Executing instruction: {instruction} (Step {self.current_step + 1} of {len(self.road_map)})")
        if instruction == "left":
            self.turn_left(self.movement_velocity)
        elif instruction == "right":
            self.turn_right(self.movement_velocity)
        elif instruction == "forward":
            self.move_forward(self.movement_velocity)

        self.current_step += 1




    def detect_waypoint(self):
        objects = self.camera.getRecognitionObjects()
        for obj in objects:
            print(f"Detected object model: {obj.getModel()}")  # Debug
            if obj.getModel() == "waypoint":  # Detect a waypoint marker
                print("Waypoint detected!")
                return True
        return False





    def navigate_maze(self):

        target_position = (2.4, 1.0, 0.1)  # Replace with your desired target position
        self.set_target_location(target_position)
        print(f"تم تعيين الموقع الهدف: {target_position}")
        while self.step(self.timestep) != -1:
            print("التنقل عبر المتاهة...")
            
            # Step 1: Calculate potential distances for each path
            forward_distance = self.calculate_distance_after_move(0) if self.is_path_forward() else float("inf")
            right_distance = self.calculate_distance_after_move(-math.pi / 2) if self.is_path_right() else float("inf")
            left_distance = self.calculate_distance_after_move(math.pi / 2) if self.is_path_left() else float("inf")

            # Debugging output for distances
            print(f"المسافة إذا تم التقدم للأمام: {forward_distance}")
            print(f"المسافة إذا تم الانعطاف لليمين: {right_distance}")
            print(f"المسافة إذا تم الانعطاف لليسار: {left_distance}")

            # Step 2: Determine the best path (shortest distance to target)
            distances = {
                "forward": forward_distance,
                "right": right_distance,
                "left": left_distance
            }
            best_path = min(distances, key=distances.get)

            # Step 3: Execute movement based on the best path
            if distances[best_path] == float("inf"):
                print("لا يوجد مسار متاح، التوقف.")
                self.halt()
                break

            if best_path == "forward":
                print("التقدم للأمام.")
                self.move_forward(self.movement_velocity)
            elif best_path == "right":
                print("الانعطاف لليمين.")
                self.rotate(-math.pi / 2)
                self.move_forward(self.movement_velocity)
            elif best_path == "left":
                print("الانعطاف لليسار.")
                self.rotate(math.pi / 2)
                self.move_forward(self.movement_velocity)

            # Step 4: Update target position and re-evaluate
            target_position = self.get_target_position()
            if target_position is None:
                print("لم يتم العثور على الهدف أثناء التنقل.")
                break

    def calculate_distance_after_move(self, angle):
        # Temporarily rotate to simulate the move
        self.rotate(angle)
        # Estimate the new distance to the target
        target_position = self.get_target_position()
        if target_position is None:
            return float("inf")
        return self.calculate_distance(target_position)


=======
        self.turn_duration = 1500  # Placeholder: the duration of a turn (to be calibrated)
        self.turn_around_duration = 3000  # Placeholder: the duration of a 180-degree turn (to be calibrated)
        self.step(self.timestep)

    def navigate_maze(self):
        while self.step(self.timestep) != -1:
            action, position, distance = self.detect_box()
            distance_x , distance_y ,distance_z = self.detect_red_floor()
            print("Start Take Direction")
            if action == 'change_direction':
               print("change_direction")
               self.avoid_obstacle(position, distance)
            elif action == 'go_and_catch' and distance > 0.04:
                  print("go_and_catch")
                  if (self.pickup_state != 'idle') and (self.detected_box_color[1] != 1):
                        print("pickup_state")
                        self.approach_box(position, distance)
                        print(f"distance_x :{distance_x} distance_y:{distance_y} distance_z:{distance_z}" )
            elif distance_x <= 0.08:
                            self.halt()
                            self.rev_take_box_from_back()
                            self.rev_take_box_from_back()
                            break             
            else:
                # if not self.is_path_right() and not self.is_path_left() and self.is_wall_corner():
                    # print("turn_around")
                    # self.rotate(math.pi)  # Rotate 180 degrees        
                if self.is_wall_corner():
                    print("corner_turn_left")
                    self.rotate(-math.pi / 2)  # Rotate 90 degrees to the left
                    self.move_forward(self.MOVEMENT_VELOCITY)
                elif self.is_path_right():
                    print("turn_right")
                    self.rotate(-math.pi / 2)  # Rotate 90 degrees to the left
                    self.move_forward(self.MOVEMENT_VELOCITY)
                elif self.is_path_forward():
                    print("move_forward")
                    self.move_forward(self.MOVEMENT_VELOCITY)
                elif self.is_path_left():
                    print("turn_left")
                    self.rotate(math.pi / 2)  # Rotate 90 degrees to the right
                    self.move_forward(self.MOVEMENT_VELOCITY)
                else:
                    print("turn_around")
                    self.rotate(math.pi)  # Rotate 180 degrees
>>>>>>> 5eb0d570632cdb4bc6a07f016fa7c45759e525ec
    def rotate(self, angle):
        # Rotate the robot by a specified angle (in radians)
        rotation_speed = self.turn_speed if angle > 0 else -self.turn_speed
        rotation_time = abs(angle / self.turn_speed)
        self.set_wheel_speeds(rotation_speed, -rotation_speed, rotation_speed, -rotation_speed)
        
        end_time = self.getTime() + rotation_time
        while self.getTime() < end_time:
            if self.step(self.timestep) == -1:
                break
    def is_wall_corner(self):
        # Check if there is a path to the right using the right sensors
        return self.wall_sensors[12].getValue() < self.WALL_THRESHOLD 
    
    def is_path_right(self):
        # Check if there is a path to the right using the right sensors
        return all(sensor.getValue() >= self.WALL_THRESHOLD for sensor in self.wall_sensors[9:12])
    def is_move_right(self):
        # Check if there is a path to the right using the right sensors
        return all(sensor.getValue() >= self.WALL_THRESHOLD/2 for sensor in self.wall_sensors[9:12])
    
    def is_path_forward(self):
        # Check if there is a path forward using the front sensors
        return all(sensor.getValue() >= self.WALL_THRESHOLD for sensor in self.wall_sensors[:3])
    
    def is_path_left(self):
        # Check if there is a path to the left using the left sensors
        return all(sensor.getValue() >= self.WALL_THRESHOLD for sensor in self.wall_sensors[3:6])
    
    def move_forward(self, speed):
        self.set_wheel_speeds(speed, speed, speed, speed)
    
    def turn_right(self, speed):
        self.rotate_clockwise(speed)
    
    def turn_left(self, speed):
        self.rotate_counterclockwise(speed)
    
    def turn_around(self, speed):
        self.rotate_clockwise(speed)
    
    def rotate_clockwise(self, speed):
        self.set_wheel_speeds(speed, -speed, speed, -speed)
    
    def rotate_counterclockwise(self, speed):
        self.set_wheel_speeds(-speed, speed, -speed, speed)
    
>>>>>>> 1c93a9aca767828fd6e12460e7e567e07b3bef3a
    def set_wheel_speeds(self, fl, fr, bl, br):
        self.front_left_motor.setVelocity(fl)
        self.front_right_motor.setVelocity(fr)
        self.back_left_motor.setVelocity(bl)
        self.back_right_motor.setVelocity(br)
    
<<<<<<< HEAD
    def move_forward(self, speed):
        self.set_wheel_speeds(speed, speed, speed, speed)
    
    def move_backward(self, speed):
        self.set_motor_velocity(-speed, -speed, -speed, -speed)

    def set_motor_velocity(self, left_velocity, right_velocity):
        self.front_left_motor.setVelocity(left_velocity)
        self.back_left_motor.setVelocity(left_velocity)
        self.front_right_motor.setVelocity(right_velocity)
        self.back_right_motor.setVelocity(right_velocity)

    def stop_motors(self):
        self.set_motor_velocity(0.0, 0.0)

    # get current angle 
    def get_current_angle(self):
        """Get the current orientation angle of the robot using the compass."""
        compass_values = self.compass.getValues()
        print(f"Compass values: {compass_values}")  # طباعة القيم
        
        if not compass_values or len(compass_values) < 2:
            raise ValueError("Invalid compass values!")
        
        # Calculate angle in degrees
        angle = math.degrees(math.atan2(compass_values[0], compass_values[1]))
        if math.isnan(angle):
            raise ValueError("Calculated angle is NaN!")
        if angle < 0:
            angle += 360
        return angle
    
    def rotate_to_target_angle(self, target_angle):
        """Rotate the robot to the target angle using the compass."""
        while True:
            try:
                current_angle = self.get_current_angle()
                angle_difference = (target_angle - current_angle + 360) % 360
                print(f"current_angle: {current_angle}, target_angle: {target_angle}, angle_difference: {angle_difference}")

                # Stop if the angle difference is small enough
                if angle_difference < 1.0 or angle_difference > 359.0:
                    break

                # Determine direction of rotation
                if angle_difference <= 180:
                    self.set_motor_velocity(self.rotation_velocity, -self.rotation_velocity)
                else:
                    self.set_motor_velocity(-self.rotation_velocity, self.rotation_velocity)

                # Step simulation
                if self.step(self.timestep) == -1:
                    break
            except ValueError as e:
                print(f"Error: {e}. Retrying...")
                self.stop_motors()
                self.wait_for_sensors(5)  # انتظر قليلاً وحاول مرة أخرى
        self.stop_motors()

    def turn_left_90_degrees(self):
        current_angle = self.get_current_angle()
        target_angle = (current_angle + 90) % 360
        self.rotate_to_target_angle(target_angle)

    def turn_right_90_degrees(self):
        current_angle = self.get_current_angle()
        target_angle = (current_angle - 90 + 360) % 360
        self.rotate_to_target_angle(target_angle)

    def stop_robot(self):
        # Stop the robot by setting all wheel velocities to 0
        for wheel in self.wheels:
            wheel.setVelocity(0)
    
    def stop_movement(self):
        self.steering(0, 0)
        
    # function to get the target coordinates for robot to go for 
    def move_to_coordinates(self):
        if self.color_queue:
            first_color = self.color_queue[0]
            if first_color in self.box_queue and self.box_queue[first_color]:
                target_box = self.box_queue[first_color][0]
                print(f"Coordinates of {first_color}: x={target_box.x}, y={target_box.y}, z={target_box.z}")
                return target_box.x, target_box.y, target_box.z
        return 0.0, 0.0, 0.0

    def get_gps_position(self):
        print("First ROBOT")
        self.current_x,self.current_y,self.current_z = self.gps.getValues()
        print(f"First ROBOT,\tCurrent_X: {self.current_x}\tCurrent_y: {self.current_y}\t")
    
    def start(self):
        time.sleep(1)
        print(f'Current Y: {self.current_y}')
        while self.step(self.timestep) != -1 and self.current_y <= -0.2:
            self.get_gps_position()
            print(f'Current Y: {self.current_y}')
            self.move_forward(self.max_velocity)
            self.get_color_queue()
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(1)
        self.turn_left_90_degrees()
        self.wait_for_sensors(10)
        time.sleep(1)
        print(f'Current Y: {self.current_y}, Current X: {self.current_x}')
        while self.step(self.timestep) != -1 and self.current_x >=-2.34:
             self.get_gps_position()
             print(f'Current X: {self.current_x}')
             self.move_forward(self.max_velocity)    
        self.stop_movement()
        self.stop_robot()
        # end start

    def go_to_red(self):
            self.wait_for_sensors(10)
            time.sleep(1)
            self.turn_right_90_degrees()
            while self.step(self.timestep) != -1 and self.current_y < 3.70:
                self.get_gps_position()
                print(f'Current Y: {self.current_y}')
                self.move_forward(self.max_velocity)
            self.stop_movement()
            self.stop_robot()
            self.wait_for_sensors(10)
            time.sleep(1)
            self.wait_for_sensors(10)
            time.sleep(1)
            self.turn_left_90_degrees()
            self.wait_for_sensors(10)
            time.sleep(1)


    def go_to_blue(self):
        self.wait_for_sensors(10)
        time.sleep(1)
        self.turn_right_90_degrees()
        self.wait_for_sensors(10)
        time.sleep(1)
        while self.step(self.timestep) != -1 and self.current_y < 1.14:
            self.get_gps_position()
            print(f'Current Y: {self.current_y}')
            self.move_forward(self.max_velocity)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(1)
        self.turn_left_90_degrees()
        self.wait_for_sensors(10)
        time.sleep(1)

    def go_to_yellow(self):
        self.wait_for_sensors(10)
        time.sleep(1)
        self.turn_left_90_degrees()
        self.wait_for_sensors(10)
        time.sleep(1)
        while self.step(self.timestep) != -1 and self.current_y > -1.45:
            self.get_gps_position()
            print(f'Current Y: {self.current_y}')
            self.move_forward(self.max_velocity)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(1)
        self.turn_right_90_degrees()
        self.wait_for_sensors(10)
        time.sleep(1)


    def go_to_green(self):
        self.wait_for_sensors(10)
        time.sleep(1)
        self.turn_left_90_degrees()
        self.wait_for_sensors(10)
        time.sleep(1)
        while self.step(self.timestep) != -1 and self.current_y > -4.0:
            self.get_gps_position()
            print(f'Current Y: {self.current_y}')
            self.move_forward(self.max_velocity)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(1)
        self.turn_right_90_degrees()
        self.wait_for_sensors(10)
        time.sleep(1)

    def wait_for_sensors(self, steps=10):
        for _ in range(steps):
            if self.step(self.timestep) == -1:
                break
      
    # drop the box if its carrying it 
=======
    def wait_for_turn_to_complete(self):
        self.step(int(self.turn_duration))
    
    def wait_for_turn_around_to_complete(self):
        self.step(int(self.turn_around_duration))

    def forward(self,time):
        for wheel in self.wheels:
            wheel.setVelocity(7.0) # maxVelocity = 14.81
        self.step(time * self.timestep)
    
    def backward(self,time):
        for wheel in self.wheels:
            wheel.setVelocity(-7.0) # maxVelocity = 14.81
        self.step(time * self.timestep)
    
    def halt(self):
        for wheel in self.wheels:
            wheel.setVelocity(0.0)
    
    def fold_arms(self):
        self.armMotors[0].setPosition(-2.9)
        self.armMotors[1].setPosition(1.5)
        self.armMotors[2].setPosition(-2.6)
        self.armMotors[3].setPosition(1.7)
        self.armMotors[4].setPosition(0)
         
    
    def stretch_arms(self):
        self.armMotors[0].setPosition(2.9)
        self.armMotors[1].setPosition(-1.0)
        self.armMotors[2].setPosition(2.5)
        self.armMotors[3].setPosition(-1.7)
        self.armMotors[4].setPosition(0)
    
    
    def turn_around(self,time):
        self.wheels[0].setVelocity(14)
        self.wheels[1].setVelocity(-14)
        self.wheels[2].setVelocity(14)
        self.wheels[3].setVelocity(-14)
        self.step(time * self.timestep)
    def disable_line_sensors(self):
        for sensor in self.sensors:
            sensor.disable()
            
    def enable_line_sensors(self):
        for sensor in self.sensors:
            sensor.enable(self.timestep) 
    # Wall Sensors
    def disable_wall_sensors(self):
        for sensor in self.wall_sensors:
            sensor.disable()
            
    def enable_wall_sensors(self):
        for sensor in self.wall_sensors:
            sensor.enable(self.timestep) 
    def clamp_speed(self, speed):
        return max(min(speed, self.max_velocity), -self.max_velocity)

    def read_sensors_value(self):
        value = 0
        for index, sensor in enumerate(self.sensors):
<<<<<<< HEAD
            #print(f"Sensor {index + 1} value: {sensor.getValue()}")
            sensor_value = sensor.getValue()
            # Debug print for each sensor value
            # print(f"Sensor {index + 1}: {sensor_value}")  
            if 300 <= sensor_value <= 550:
=======
            sensor_value = sensor.getValue()
            # Debug print for each sensor value
            # print(f"Sensor {index + 1}: {sensor_value}")  
            if sensor_value > 200:
>>>>>>> 5eb0d570632cdb4bc6a07f016fa7c45759e525ec
                value += self.sensors_coefficient[index]
        return value

    def steering(self, left_speed, right_speed):
        left_speed = self.clamp_speed(left_speed)
        right_speed = self.clamp_speed(right_speed)
        
        # Debug prints for wheel velocities
        # print(f"Clamped Left wheel speed: {left_speed}")
        # print(f"Clamped Right wheel speed: {right_speed}")
        
        self.wheels[0].setVelocity(left_speed)
        self.wheels[2].setVelocity(left_speed)
        self.wheels[1].setVelocity(right_speed)
        self.wheels[3].setVelocity(right_speed)
    def detect_box(self):
        # Get the recognized objects from the camera
        objects = self.camera.getRecognitionObjects()
        is_box_detected = False
        print(f"Number of objects detected: {len(objects)}")  
        for obj in objects:
            model_label = obj.getModel()  
            print(f"Detected object model: {model_label}")  
            colors = obj.getColors()
            red = colors[0]
            green = colors[1]
            blue = colors[2]
            current_color = (red, green, blue)
            self.detected_box_color = current_color
            print(f"Detected box : {( self.detected_box_color)}") 
            print(f"Box detected with color: R={red:.2f}, G={green:.2f}, B={blue:.2f}")
            if model_label == 'box': 
                x,y = obj.getPositionOnImage()
                image_width = self.camera.getWidth()
                position = (x - (image_width / 2)) / (image_width / 2)  # Normalized [-1, 1]
                distance = abs(obj.getPosition()[0] ) 
                if green > red and green > blue:
                    print(f"distance box {distance}")
                    if  distance <= 0.4: 
                        return 'change_direction', position, distance  
                else:
                        if distance  <= 0.35:
                        # COUNTER_PICKK_UP=COUNTER_PICKK_UP+1 
                        # COUNTER_PICKK_UP=COUNTER_PICKK_UP+1
                              return 'go_and_catch', position, distance 
        return 'no_box', 0, float("inf")  
<<<<<<< HEAD
    def get_target_position(self):
        objects = self.camera.getRecognitionObjects()
        for obj in objects:
            if obj.getModel() == "target":  # افترض أن الهدف له اسم "target"
                position = obj.getPosition()  # إرجاع (x, y, z)
                return position
        return None  # إذا لم يتم العثور على الهدف

    def calculate_distance(self, target_position):
        x, y, z = target_position
        distance = math.sqrt(x**2 + z**2)  # المسافة الثنائية (x, z)
        print(f"Calculated distance to target: {distance}")
        return distance

    def calculate_angle(self, target_position):
        x, y, z = target_position
        angle = math.atan2(z, x)  # الزاوية بين الروبوت والهدف
        print(f"Calculated angle to target: {math.degrees(angle)} degrees")
        return angle
    
    def set_target_location(self, target_position):
        self.target_position = target_position
        print(f"Target location set to: {self.target_position}")
    
    def line_follow(self):
        goal = 0
        reading = self.read_sensors_value()
        error = goal - reading
        error_rate = error - self.last_error
        P = self.Kp * error
        D = self.Kd * error_rate
        self.last_error = error
=======
    def line_follow(self):
        goal = 0
        reading = self.read_sensors_value()
        # print(f"reading is{reading}")
        error = goal - reading
        P = self.Kp * error

        error_rate = error - self.last_error
        self.last_error = error
        D = self.Kd * error_rate
>>>>>>> 5eb0d570632cdb4bc6a07f016fa7c45759e525ec
        # Print the P and D values for debugging
        # print(f"P value: {P}")
        # print(f"D value: {D}")
        # Calculate steering using PID output (without the integral term)
        steering_correction = P + D
        left_speed = self.movement_velocity / 2 - steering_correction
        right_speed = self.movement_velocity / 2 + steering_correction
        self.steering(left_speed, right_speed)
<<<<<<< HEAD

        if self.detect_waypoint():
            print("Waypoint detected.")
            self.halt()
            return True
        return False


    def follow_line_to_target(self):

        target_position = (2.4, 1.0, 0.1)  # Replace with your desired target position (x, y, z)
        self.set_target_location(target_position)
        print(f"تم تعيين الموقع الهدف: {target_position}")

        target_position = self.get_target_position()
        if target_position is None:
            print("Target not found")
            return

        while self.step(self.time_step) != -1:

            angle = self.calculate_angle(target_position)
            distance = self.calculate_distance(target_position)


            print(f"Calculated angle to target: {math.degrees(angle)} degrees")
            print(f"Calculated distance to target: {distance}")


            if abs(angle) > 0.1: 
                print(f"Adjusting angle: {math.degrees(angle)} degrees")
                self.rotate(angle)

            if distance > 0.05: 
                print(f"التحرك للأمام، المسافة إلى الهدف: {distance}")
                self.move_forward()
            else:
                print("تم الوصول إلى الهدف!")
                self.halt()
                break

            target_position = self.get_target_position()
            if target_position is None :
                print("فقد الهدف أثناء التتبع.")
                break
                
    
=======
        marker_detected = self.detect_gate_marker()
        if marker_detected:
            print("Switch To Navigate Maze")
            return True  # Indicate that the gate has been detected
        return False  # Indicate that the gate has not been detected
>>>>>>> 5eb0d570632cdb4bc6a07f016fa7c45759e525ec

    def detect_gate_marker(self):
        objects = self.camera.getRecognitionObjects()
        for obj in objects:
            model_label = obj.getModel()
            distance = abs(obj.getPosition()[0] ) 
            if model_label.lower() == 'rectangular panel':
                print(f"Distance is{distance}")
                if distance <= 2.95:
                    return True
        return False
      
    def avoid_obstacle(self,position,distance):
        # self.disable_line_sensors()  # Disable line sensors to avoid interference
        avoidance_speed = self.max_velocity / 2  # Adjust as needed
        self.disable_line_sensors()  # Disable line sensors to avoid interference
        print("Disable Line Sensor")
            # self.disable_line_sensors()  # Disable line sensors to avoid interference

        if position < 0:
            print("right")
             # Box to the left, navigate right
            self.steering(0, 0)
            self.step(50 * self.timestep)  # Wait a moment
            self.steering(-4, 4)
            print("1 Done")
            self.step(50 * self.timestep)  # Wait a moment
            print("2 Done")
            self.steering(0, 0)
            print("3 Done")
            # self.forward(1)
            self.steering(5,5)
            print("4 Done")
            self.step(50 * self.timestep)  # Wait a moment
            print("5 Done")
            self.steering(9,-9)

        elif position > 0:
            print("left")
            self.steering(0, 0)
            self.step(50 * self.timestep)  # Wait a moment
            self.steering(4, -4)
            self.step(50 * self.timestep)  # Wait a moment
            self.steering(0, 0)
            # self.forward(1)
            self.steering(5,5)
            self.step(50 * self.timestep)  # Wait a moment
            self.steering(-8,8)
        else:
            print("Center")
             # Box to the left, navigate right
            self.steering(0, 0)
            self.step(50 * self.timestep)  # Wait a moment
            self.steering(-4, 4)
            print("1 Done")
            self.step(50 * self.timestep)  # Wait a moment
            print("2 Done")
            self.steering(0, 0)
            print("3 Done")
            # self.forward(1)
            self.steering(5,5)
            print("4 Done")
            self.step(50 * self.timestep) 
            print("5 Done")
            self.steering(9,-9)
        self.step(1000)  
        self.enable_line_sensors()  #
        self.line_follow()  
        self.movement_velocity = self.max_velocity 
    
    def perform_pick_and_place(self):
        self.halt()
        # self.step(100 * self.timestep)  # Wait a moment
        self.pick_up()
        self.step(100 * self.timestep)  # Wait a moment
        # self.turn_around(0)
        self.drop()
        # self.forward(400)
        # self.turn_around(70)
        self.step(100 * self.timestep)  # Wait a moment
        self.open_grippers()
        self.step(50 * self.timestep)  # Wait a moment
        self.hand_up()
        print("End Pick Up")
        # self.step(50 * self.timestep)  # Wait a moment
        # self.backward(60)
        # self.fold_arms()
        # self.forward(300)
        # self.halt()
    def open_grippers(self):
        max_position = min(self.fingerMaxPosition, 0.025)  # Assuming 0.025 is the max limit
        self.finger1.setPosition(max_position)
        self.finger2.setPosition(max_position)
        self.step(70 * self.timestep)
    
    def drop(self):
        print(f"COUNTER_PICKK_UP{self.COUNTER_PICKK_UP }")
        if self.COUNTER_PICKK_UP == 1 :
            self.armMotors[0].setPosition(-2.9)
            self.armMotors[1].setPosition(0)
            self.armMotors[2].setPosition(-1)
            self.armMotors[3].setPosition(-1)
            self.armMotors[2].setPosition(-1.7)
            self.armMotors[4].setPosition(2.9)
        elif self.COUNTER_PICKK_UP == 2 :
            self.armMotors[0].setPosition(2.9)
            self.armMotors[1].setPosition(0)
            self.armMotors[2].setPosition(-1)
            self.armMotors[3].setPosition(-1)
            self.armMotors[2].setPosition(-1.7)
            self.armMotors[4].setPosition(2.9)
>>>>>>> 1c93a9aca767828fd6e12460e7e567e07b3bef3a
    def hand_up(self):
        self.armMotors[0].setPosition(0)
        self.armMotors[1].setPosition(0)
        self.armMotors[2].setPosition(0)
        self.armMotors[3].setPosition(0)
        self.armMotors[4].setPosition(0)
<<<<<<< HEAD

    # pickup  the box from the Ground
    def pick_up(self):
=======
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)

    def pick_up(self):
            self.pickup_state = 'open_gripper'
>>>>>>> 1c93a9aca767828fd6e12460e7e567e07b3bef3a
            print("open_gripper")
            self.step(50 * self.timestep)  
            self.finger1.setPosition(self.fingerMaxPosition)
            self.finger2.setPosition(self.fingerMaxPosition)
<<<<<<< HEAD
=======
            self.pickup_state = 'lower_arm'
>>>>>>> 1c93a9aca767828fd6e12460e7e567e07b3bef3a
            print("lower arm")
            self.step(100 * self.timestep) 
            self.armMotors[1].setPosition(-1.13)
            self.armMotors[2].setPosition(-1.10)
<<<<<<< HEAD
            self.armMotors[3].setPosition(-1.35)
=======
            self.armMotors[3].setPosition(-1.3)
            self.pickup_state = 'close_gripper'
>>>>>>> 1c93a9aca767828fd6e12460e7e567e07b3bef3a
            print("close gripper")
            self.step(100 * self.timestep)  
            self.finger1.setPosition(0.013)  
            self.finger2.setPosition(0.013)
            self.step(50 * self.timestep)  
<<<<<<< HEAD
            print("lift arm")
            self.step(50 * self.timestep)  
            self.armMotors[1].setPosition(0)
            print("complete")
            self.step(50 * self.timestep)  # Wait a moment
            print("idle")
            self.armMotors[0].setPosition(-2.90)
            self.armMotors[1].setPosition(0.30)
            self.armMotors[2].setPosition(-1.35)
            self.armMotors[3].setPosition(-1.30)
            self.step(100 * self.timestep)
            # deraise the hand a little to be able to put the hand correctly
            self.armMotors[1].setPosition(-0.30)
            self.step(50 * self.timestep)
            print("Drop the box on the Basket")
            self.finger1.setPosition(self.fingerMaxPosition)
            self.finger2.setPosition(self.fingerMaxPosition)
            self.step(50 * self.timestep)
            # raise the hand a little bit to move freely without touch it
            self.armMotors[1].setPosition(0.30)
            self.step(50 * self.timestep)
            print("return To start Position")
            self.COUNTER_PICKK_UP += 1 
            self.hand_up()

    def pick_green_and_retrun_to_node(self):
        self.pick_up()
        time.sleep(1)
        self.wait_for_sensors(10)
        self.turn_left_90_degrees()
        self.wait_for_sensors(10)
        time.sleep(1)
        self.turn_left_90_degrees()
        self.wait_for_sensors(10)
        time.sleep(1)
        print('finish Turning')
        self.get_gps_position()
        time.sleep(1)
        while self.step(self.timestep) != -1 and self.current_x <-2.45:
            self.get_gps_position()
            print(f'Current x: {self.current_x}')
            self.move_forward(self.max_velocity)
        self.stop_movement()
        self.stop_robot()
        time.sleep(1)
        self.turn_left_90_degrees()
        self.wait_for_sensors(10)
        time.sleep(1)
        while self.step(self.timestep) != -1 and self.current_y <-0.0:
            self.get_gps_position()
            print(f'Current Y: {self.current_y}')
            self.move_forward(self.max_velocity)
        self.stop_movement()
        self.stop_robot()
        self.turn_right_90_degrees()
        self.wait_for_sensors(10)
        time.sleep(1)
        
    def return_form_node_to_wall(self):
        self.wait_for_sensors(10)
        time.sleep(1)

        while self.step(self.timestep) != -1 and self.current_x <-0.5:
            self.get_gps_position()
            print(f'Current Y: {self.current_y}')
            self.move_forward(self.max_velocity)
        self.stop_movement()
        self.stop_robot()
        time.sleep(1)

    def pick_up_form_basket(self):
        # rotate the hand to basket position
        self.armMotors[0].setPosition(-2.90)
        self.armMotors[1].setPosition(0.30)
        self.armMotors[2].setPosition(-1.35)
        self.armMotors[3].setPosition(-1.30)
        self.step(100 * self.timestep)
        # open the fingers
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(100 * self.timestep)
        # down the hand a little to be able to catch the Box
        self.armMotors[1].setPosition(-0.30)
        self.step(100 * self.timestep)
        # Catch the Box
        self.finger1.setPosition(0.013)  
        self.finger2.setPosition(0.013)
        self.step(100 * self.timestep)
        # Get the Hand to Up 
        self.hand_up()
        self.step(100 * self.timestep)
        # Down the Hand to the Height of the Wall
        self.armMotors[2].setPosition(-1.0)
        self.armMotors[3].setPosition(-1.5)
        self.step(100 * self.timestep)
        # open the finger to Drop the Box
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(100 * self.timestep)
        # return the to the basic place
        self.hand_up()

    def go_to_picK_first_red_box(self):
        self.wait_for_sensors(10)
        time.sleep(1)
        while self.step(self.timestep) != -1 and self.current_x >= -3.3150000:
            self.get_gps_position()
            print(f'Current X: {self.current_x}')
            self.move_forward(0.50)
            self.get_gps_position()
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(1)

    # def move_left(self, speed=1.0):
    #     self.set_motor_velocity(-speed, speed, speed, -speed)

    # def move_right(self, speed=1.0):
    #     self.set_motor_velocity(speed, -speed, -speed, speed)

    # def move_to_position(self, target_x, target_y, speed=2.0):
    #     while True:
    #         self.step()
            
    #         print(f"Current Position: ({self.current_x}, {self.current_y})")
    #         # Calculate differences
    #         diff_x = target_x - self.current_x
    #         diff_y = target_y - self.current_y

    #         # إذا كان الفرق في المحورين أقل من العتبة، أوقف الروبوت
    #         if abs(diff_x) < 0.01 and abs(diff_y) < 0.001:  # Threshold for precision
    #             print("Arrived at target position!")
    #             self.stop()
    #             break

    #         # حركة الروبوت على المحور X
    #         if abs(diff_x) >= 0.01:  # إذا كان الفرق كبيرًا على المحور X
    #             if diff_x > 0:
    #                 self.move_forward(speed)  # التحرك للأمام
    #             else:
    #                 self.move_backward(speed)  # الرجوع للخلف
    #         else:
    #             self.stop()  # أوقف الحركة على المحور X

    #         # حركة الروبوت على المحور Y
    #         if abs(diff_y) >= 0.01:  # إذا كان الفرق كبيرًا على المحور Y
    #             if diff_y > 0:
    #                 self.move_right(speed)  # التحرك لليمين
    #             else:
    #                 self.move_left(speed)  # التحرك لليسار
    #         else:
    #             self.stop()  # أوقف الحركة على المحور Y


robot = RobotController()
robot.start()
robot.print_color_queue()

for color in robot.color_queue:
    if color == 'Red':
        print("\nThe Color Is Now RED\n")
        robot.go_to_red()
        # robot.go_to_picK_first_red_box()
        # # robot.move_to_position(target_x = -3.30032, target_y = 3.76234)
        # robot.pick_up()
    elif color == 'Blue':
        print("\nThe Color Is Now Blue\n")
        robot.go_to_blue()
    elif color == 'Green':
        print("\nThe Color Is Now Green \n")
        robot.go_to_green()
    elif color == 'Yellow':
        print("\nThe Color Is Now Yellow \n")
        robot.go_to_yellow()
    else:
        print('\nTheere is no Colors detect Bro\n')
        break
    


    
=======
            self.pickup_state = 'lift_arm'
            print("lift arm")
            self.step(50 * self.timestep)  
            self.armMotors[1].setPosition(0)
            self.pickup_state = 'complete'
            print("complete")
            self.step(50 * self.timestep)  # Wait a moment
            self.pickup_state = 'idle'
            print("idle")

      
    def approach_box(self, position, distance):
        print(f"the Distance is {distance}")
        pickup_distance = 0.13 
        if distance <= pickup_distance:
            self.COUNTER_PICKK_UP= self.COUNTER_PICKK_UP+1
            print(f"COUNTER_PICKK_UP is : {self.COUNTER_PICKK_UP}")
            self.perform_pick_and_place()
        else:
            turn_direction = 1 if position > 0 else -1
    
            turn_speed = self.max_velocity * turn_direction * 0.5 
            move_speed = self.max_velocity * 0.5  
    
            self.steering(turn_speed, -turn_speed)
            self.forward(1)  
    
            self.detect_box()
            # self.line_follow()
            
    def detect_red_floor(self ):
        objects = self.camera.getRecognitionObjects()
        distance_x=10000
        distance_y=10000
        distance_z=10000
        for obj in objects:
            model_label = obj.getModel() 
            # if model_label != 'box' and model_label.lower() != 'rectangular panel':
            if model_label == 'floor_red':

                print("1")
                distance_x = abs(obj.getPosition()[0] )         
                distance_y = abs(obj.getPosition()[1] ) 
                distance_z = abs(obj.getPosition()[2] ) 

                print(f"distance_x is :  {distance_x}\n") 
                print(f"distance_y is :  {distance_y}\n") 
                print(f"distance_z is :  {distance_z}\n") 

                colors = obj.getColors()
                red, green, blue = colors[0], colors[1], colors[2]
            # Define thresholds for red color detection
                if red == 1 and green == 0 and blue == 0:
                    # if distance_x <= 0.08 and distance_y <= 0.025 and distance_z  <= 0.0037 :
                        print("2")
                        return distance_x , distance_y ,distance_z
        return distance_x , distance_y ,distance_z


    def rev_pick_up(self):
        
        if self.COUNTER_REV_PICKK_UP == 1:
            self.armMotors[0].setPosition(-2.90)
            self.armMotors[1].setPosition(-0.30)
            self.armMotors[2].setPosition(-1.35)
            self.armMotors[3].setPosition(-1.30)
            self.finger1.setPosition(self.fingerMaxPosition)
            self.finger2.setPosition(self.fingerMaxPosition)
        elif self.COUNTER_REV_PICKK_UP == 2:
            self.armMotors[0].setPosition(2.90)
            self.armMotors[1].setPosition(-0.25)
            self.armMotors[2].setPosition(-1.35)
            self.armMotors[3].setPosition(-1.30)
            self.finger1.setPosition(self.fingerMaxPosition)
            self.finger2.setPosition(self.fingerMaxPosition)   

    def rev_close_grippers(self):
        self.finger1.setPosition(0.013)     # Close gripper.
        self.finger2.setPosition(0.013)

    def rev_hand_up(self):
        self.armMotors[0].setPosition(0)
        self.armMotors[1].setPosition(0)
        self.armMotors[2].setPosition(0)
        self.armMotors[3].setPosition(0)
        self.armMotors[4].setPosition(0)

    def rev_fold_arms(self):
        self.armMotors[0].setPosition(0)
        self.armMotors[1].setPosition(0)
        self.armMotors[2].setPosition(0)
        self.armMotors[3].setPosition(0)
        self.armMotors[4].setPosition(0)

    def rev_drop(self):
        # Move arm down
        self.armMotors[3].setPosition(0)
        self.armMotors[2].setPosition(-0.3)
        self.step(100 * self.timestep)

        self.armMotors[1].setPosition(-1.0)
        self.step(100 * self.timestep)

        self.armMotors[3].setPosition(-1.5)
        self.step(100 * self.timestep)

        self.armMotors[2].setPosition(-0.4)
        self.step(50 * self.timestep)
        self.armMotors[4].setPosition(-1)

        # Open gripper.
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(50 * self.timestep)

    def rev_take_box_from_back(self):
        self.COUNTER_REV_PICKK_UP=self.COUNTER_REV_PICKK_UP+1
        print("halt")
        self.halt()
        self.step(100 * self.timestep)
        print("pick_up")
        self.rev_pick_up()
        self.step(100 * self.timestep)
        print("close_grippers")
        self.rev_close_grippers()
        self.step(100 * self.timestep)
        print("hand_up")
        self.rev_hand_up()
        # forward(20)
        # halt()
        self.step(100 * self.timestep)
        print("drop")
        self.rev_drop()
        # hand_up()
        self.step(100 * self.timestep)
        print("fold_arms")
        self.rev_fold_arms()

<<<<<<< HEAD
    def turn_left(self, speed):
        self.set_wheel_speeds(-speed, speed, -speed, speed)
        self.step(int(self.turn_duration))  # Adjust turn_duration for proper alignment
        self.halt()

        self.move_forward(speed)
        self.step(200)  # 200 ms for re-alignment (tweak as needed)
        self.halt()

    def turn_right(self, speed):
        self.set_wheel_speeds(speed, -speed, speed, -speed)
        self.step(int(self.turn_duration))  # Adjust turn_duration for proper alignment
        self.halt()

        self.move_forward(speed)
        self.step(200)  # 200 ms for re-alignment (tweak as needed)
        self.halt()

    def process_camera_image(self):
        # Get camera image as raw data
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        image = self.camera.getImage()
        
        # Convert raw image to numpy array
        img_array = np.frombuffer(image, np.uint8).reshape((height, width, 4))
        
        # Extract RGB values
        rgb_array = img_array[:, :, :3]  # Ignore alpha channel

        # Define exact colors based on your Webots material settings
        colors = {
            'blue': [255, 0, 0],
            'green': [0, 255, 0],
            'red': [0, 0, 255],
            'yellow': [0, 255, 255]
        }

        # Initialize detected colors queue with a maximum size of 4
        if not hasattr(self, 'detected_color_queue'):
            self.detected_color_queue = deque(maxlen=4)

        # Detect colors in the current frame
        for color_name, rgb_value in colors.items():
            # Create a mask for the color
            mask = (rgb_array == rgb_value).all(axis=2)
            if mask.any():  # If any pixel matches the color
                # Add to queue only if not already present
                if color_name not in self.detected_color_queue:
                    self.detected_color_queue.append(color_name)

        # Print the queue of detected colors
        print(f"Queue of Detected Colors: {list(self.detected_color_queue)}")
=======

>>>>>>> 5eb0d570632cdb4bc6a07f016fa7c45759e525ec



    def loop(self):
        while self.step(self.timestep) != -1:
<<<<<<< HEAD
            self.process_camera_image()
            if self.current_step >= len(self.road_map):
                print("Road map completed. Stopping.")
                self.halt()
                break
                print("startlinefollow")
            line_reached = self.line_follow()
            if line_reached:
                print("Reached waypoint. Taking next instruction.")
                self.execute_next_instruction()      





                
                  
                  # التوقف عند الوصول إلى الهدف
                # action, position, distance = self.detect_box()
                # if action == 'change_direction':
                #     print("change_direction")
                #     self.avoid_obstacle(position, distance)
                # elif action == 'go_and_catch' :
                #     print("go_and_catch")
                #     if (self.pickup_state != 'idle') and (self.detected_box_color[1] != 1):
                #         print("pickup_state")
                #         self.approach_box(position, distance)
                # else:
                #      if self.line_follow() ==  True:  # If line_follow returns True, gate marker is detected
                #         print("Start Navigate Maze")
                #         self.navigate_maze()
                #         break
=======
        #   self.move_forward(2)
            # self.COUNTER_REV_PICKK_UP+=1       
                action, position, distance = self.detect_box()
                if action == 'change_direction':
                    print("change_direction")
                    self.avoid_obstacle(position, distance)
                elif action == 'go_and_catch' :
                    print("go_and_catch")
                    if (self.pickup_state != 'idle') and (self.detected_box_color[1] != 1):
                        print("pickup_state")
                        self.approach_box(position, distance)
                else:
                     if self.line_follow() ==  True:  # If line_follow returns True, gate marker is detected
                        print("Start Navigate Maze")
                        self.navigate_maze()
                        break
>>>>>>> 5eb0d570632cdb4bc6a07f016fa7c45759e525ec

                        # break  # Break out of the loop after navigating the maze
# After navigating the maze, you might want to end the loop

robot = RobotController()
robot.loop()
>>>>>>> 1c93a9aca767828fd6e12460e7e567e07b3bef3a
