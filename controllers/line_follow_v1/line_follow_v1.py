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
        for wheel in self.wheels:
            wheel.setPosition(float("inf"))
            wheel.setVelocity(0)

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
    
    def set_wheel_speeds(self, fl, fr, bl, br):
        self.front_left_motor.setVelocity(fl)
        self.front_right_motor.setVelocity(fr)
        self.back_left_motor.setVelocity(bl)
        self.back_right_motor.setVelocity(br)
    
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
    def hand_up(self):
        self.armMotors[0].setPosition(0)
        self.armMotors[1].setPosition(0)
        self.armMotors[2].setPosition(0)
        self.armMotors[3].setPosition(0)
        self.armMotors[4].setPosition(0)
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)

    def pick_up(self):
            self.pickup_state = 'open_gripper'
            print("open_gripper")
            self.step(50 * self.timestep)  
            self.finger1.setPosition(self.fingerMaxPosition)
            self.finger2.setPosition(self.fingerMaxPosition)
            self.pickup_state = 'lower_arm'
            print("lower arm")
            self.step(100 * self.timestep) 
            self.armMotors[1].setPosition(-1.13)
            self.armMotors[2].setPosition(-1.10)
            self.armMotors[3].setPosition(-1.3)
            self.pickup_state = 'close_gripper'
            print("close gripper")
            self.step(100 * self.timestep)  
            self.finger1.setPosition(0.013)  
            self.finger2.setPosition(0.013)
            self.step(50 * self.timestep)  
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