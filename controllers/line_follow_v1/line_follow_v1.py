from controller import Robot, Camera , Emitter
import math
import time 
import struct

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
        for wheel in self.wheels:
            wheel.setPosition(float("inf"))
            wheel.setVelocity(0)

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

        # self.max_velocity = 14.81
        self.max_velocity = 7.0
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

        self.COUNTER_PICK_UP=0

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

        self.emitter = self.getDevice('emitter')
        self.cube = 100
        

    def send_boolean(self, value):
        self.emitter.send(struct.pack('i', value))
        print(" SEND !!")

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

    def set_wheel_speeds(self, fl, fr, bl, br):
        self.front_left_motor.setVelocity(fl)
        self.front_right_motor.setVelocity(fr)
        self.back_left_motor.setVelocity(bl)
        self.back_right_motor.setVelocity(br)
    
    def move_forward(self, speed):
        self.set_wheel_speeds(speed, speed, speed, speed)
    
    def move_backward(self, speed):
        self.set_motor_velocity(-speed, -speed, -speed, -speed)

    def move_right(self, speed):
        self.set_motor_velocity(-speed, speed, speed, -speed)

    def move_left(self, speed):
        self.set_motor_velocity(speed, -speed, -speed, speed)

    def set_motor_rotation_velocity(self, left_velocity, right_velocity):
        self.front_left_motor.setVelocity(left_velocity)
        self.back_left_motor.setVelocity(left_velocity)
        self.front_right_motor.setVelocity(right_velocity)
        self.back_right_motor.setVelocity(right_velocity)

    def set_motor_velocity(self, front_left, front_right, back_left, back_right):
        self.front_left_motor.setVelocity(front_left)
        self.front_right_motor.setVelocity(front_right)
        self.back_left_motor.setVelocity(back_left)
        self.back_right_motor.setVelocity(back_right)

    def stop_motors(self):
        self.set_motor_velocity(0.0, 0.0,0.0,0.0)

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
                    self.set_motor_rotation_velocity(self.rotation_velocity, -self.rotation_velocity)
                else:
                    self.set_motor_rotation_velocity(-self.rotation_velocity, self.rotation_velocity)

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

    def turn_left_180_degrees(self):
        current_angle = self.get_current_angle()
        target_angle = (current_angle + 180) % 360
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

    def move_to_correct_x_position(self, target_x, speed = 1.0):
        while True:
            self.step(self.timestep)
            self.wait_for_sensors(10)
            self.get_gps_position()
            # self.wait_for_sensors(10)
            print(f"Current Position: ({self.current_x}, {self.current_y})")
            diff_x = target_x - self.current_x

            if abs(diff_x) <= 0.001:  # Threshold for precision
                print("Arrived at target position!")
                self.stop_movement()
                self.stop_robot()
                self.stop_motors()
                self.wait_for_sensors(10)
                break
            print(f'diff_x {diff_x}')
            if abs(diff_x) > 0.001: 
                if diff_x > 0:
                    print("Moving backward to align X")
                    self.move_backward(speed)  
                else:
                    print("Moving forward to align X")
                    self.move_forward(speed)  
            else:
                self.stop_movement()
                self.stop_robot() 
                self.stop_motors()
                self.wait_for_sensors(10)
    
    def move_to_correct_x_position_Opposite(self, target_x, speed = 1.0):
        while True:
            self.step(self.timestep)
            self.wait_for_sensors(10)
            self.get_gps_position()
            # self.wait_for_sensors(10)
            print(f"Current Position: ({self.current_x}, {self.current_y})")
            diff_x = target_x - self.current_x

            if abs(diff_x) <= 0.001:  # Threshold for precision
                print("Arrived at target position!")
                self.stop_movement()
                self.stop_robot()
                self.stop_motors()
                self.wait_for_sensors(10)
                break
            print(f'diff_x {diff_x}')
            if abs(diff_x) > 0.001: 
                if diff_x > 0:
                    print("Moving forward to align X")
                    self.move_forward(speed)  
                   
                else:
                    print("Moving backward to align X")
                    self.move_backward(speed)  
            else:
                self.stop_movement()
                self.stop_robot() 
                self.stop_motors()
                self.wait_for_sensors(10)

    def move_to_correct_y_position(self, target_y, speed=1.0):
        while True:
            self.step(self.timestep)
            self.wait_for_sensors(10)
            self.get_gps_position()
            # self.wait_for_sensors(10)
            diff_y = target_y - self.current_y
            if abs(diff_y) <= 0.001:  
                print("Arrived at target position!")
                self.stop_movement()
                self.stop_robot()
                self.stop_motors()
                self.wait_for_sensors(10)
                break
            print(f'diff_y {diff_y}')
            if abs(diff_y) >= 0.001:  
                if diff_y > 0:
                    print("Moving right to align Y")
                    self.move_right(speed) 
                else:
                    print("Moving left to align Y")
                    self.move_left(speed) 
            else:
                self.stop_movement()
                self.stop_robot()
                self.stop_motors()
                self.wait_for_sensors(10)

    def move_to_correct_y_position_Opposite(self, target_y, speed=1.0):
        while True:
            self.step(self.timestep)
            self.wait_for_sensors(10)
            self.get_gps_position()
            diff_y = target_y - self.current_y
            if abs(diff_y) <= 0.001:  
                print("Arrived at target position!")
                self.stop_movement()
                self.stop_robot()
                self.stop_motors()
                self.wait_for_sensors(10)
                break
            print(f'diff_y {diff_y}')
            if abs(diff_y) >= 0.001:  
                if diff_y > 0:
                    print("Moving left to align Y")
                    self.move_left(speed)
                else:
                    print("Moving right to align Y")
                    self.move_right(speed) 
            else:
                self.stop_movement()
                self.stop_robot()
                self.stop_motors()
                self.wait_for_sensors(10)
                 
    def move_to_position(self, target_x, target_y, speed=1.0):
        self.move_to_correct_x_position(target_x,speed)
        self.wait_for_sensors(100)
        time.sleep(0.25)
        self.move_to_correct_y_position(target_y,speed)
        self.wait_for_sensors(100)
        time.sleep(0.25)
        self.move_to_correct_x_position(target_x,speed)
        self.wait_for_sensors(100)
        time.sleep(0.25)
        self.move_to_correct_y_position(target_y,speed)


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
            time.sleep(0.5)
            self.get_gps_position()
            while self.step(self.timestep) != -1 and self.current_y < 3.70:
                self.get_gps_position()
                print(f'Current Y: {self.current_y}')
                self.move_right(7.0)
            self.stop_movement()
            self.stop_robot()
            self.wait_for_sensors(10)
            time.sleep(1)

    def go_to_blue(self):
        self.wait_for_sensors(10)
        time.sleep(0.5)
        self.get_gps_position()
        while self.step(self.timestep) != -1 and self.current_y < 1.14:
            self.get_gps_position()
            print(f'Current Y: {self.current_y}')
            self.move_right(7.0)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(0.5)

    def go_to_yellow(self):
        self.wait_for_sensors(10)
        time.sleep(0.5)
        self.get_gps_position()
        while self.step(self.timestep) != -1 and self.current_y > -1.45:
            self.get_gps_position()
            print(f'Current Y: {self.current_y}')
            self.move_left(7.0)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(0.5)

    def go_to_green(self):
        self.wait_for_sensors(10)
        time.sleep(0.5)
        self.get_gps_position()
        while self.step(self.timestep) != -1 and self.current_y > -4.0:
            self.get_gps_position()
            print(f'Current Y: {self.current_y}')
            self.move_left(7.0)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(0.5)

    def wait_for_sensors(self, steps=10):
        for _ in range(steps):
            if self.step(self.timestep) == -1:
                break
      
    # drop the box if its carrying it 
    def hand_up(self):
        self.armMotors[0].setPosition(0)
        self.armMotors[1].setPosition(0)
        self.armMotors[2].setPosition(0)
        self.armMotors[3].setPosition(0)
        self.armMotors[4].setPosition(0)

    # pickup  the box from the Ground
    def pick_up(self):
            print("open_gripper")
            self.step(100 * self.timestep)  
            self.finger1.setPosition(self.fingerMaxPosition)
            self.finger2.setPosition(self.fingerMaxPosition)
            print("lower arm")
            self.step(100 * self.timestep) 
            self.armMotors[1].setPosition(-1.10)
            self.armMotors[2].setPosition(-1.05)
            self.armMotors[3].setPosition(-1.00)
            print("close gripper")
            self.step(100 * self.timestep)  
            self.finger1.setPosition(0.010)  
            self.finger2.setPosition(0.010)
            self.step(100 * self.timestep)  
            print("lift arm")
            self.step(100 * self.timestep)  
            self.armMotors[1].setPosition(0.30)
            print("complete")
            self.step(100 * self.timestep)  # Wait a moment
            print("idle")
            self.armMotors[0].setPosition(-2.90)
            self.armMotors[1].setPosition(0.30)
            self.armMotors[2].setPosition(-1.35)
            self.armMotors[3].setPosition(-1.30)
            self.step(100 * self.timestep)
            # deraise the hand a little to be able to put the hand correctly
            self.armMotors[1].setPosition(-0.25)
            self.step(100 * self.timestep)
            print("Drop the box on the Basket")
            self.finger1.setPosition(self.fingerMaxPosition)
            self.finger2.setPosition(self.fingerMaxPosition)
            self.step(100 * self.timestep)
            # raise the hand a little bit to move freely without touch it
            self.armMotors[1].setPosition(0.30)
            self.step(100 * self.timestep)
            self.hand_up()
            self.step(100 * self.timestep)
            print("return To start Position")
            
    def pick_green_and_retrun_to_node(self):
        self.pick_up()
        time.sleep(0.5)
        self.wait_for_sensors(10)
        print('finish Turning')
        self.get_gps_position()
        while self.step(self.timestep) != -1 and self.current_x <-2.45:
            self.get_gps_position()
            print(f'Current x: {self.current_x}')
            self.move_backward(7.0)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(0.5)
        while self.step(self.timestep) != -1 and self.current_y <= 0.0:
            self.get_gps_position()
            print(f'Current Y: {self.current_y}')
            self.move_right(7.0)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(0.5)
        
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
        # add corrections to positions
        self.move_to_correct_x_position(-0.523046)
        self.wait_for_sensors(10)
        self.move_to_correct_y_position_Opposite(-0.126407)
        self.wait_for_sensors(10)

    def return_from_wall_to_node(self):
        self.wait_for_sensors(10)
        time.sleep(0.5)
        while self.step(self.timestep) != -1 and self.current_x >=-1.17:
             self.get_gps_position()
             print(f'Current X: {self.current_x}')
             self.move_backward(7.0)    
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(0.5)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(0.5)
        self.turn_left_180_degrees()
        self.wait_for_sensors(10)
        time.sleep(0.5)
        print(f'Current Y: {self.current_y}, Current X: {self.current_x}')
        while self.step(self.timestep) != -1 and self.current_x >=-2.34:
             self.get_gps_position()
             print(f'Current X: {self.current_x}')
             self.move_forward(self.max_velocity)    
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(0.5)
        # add corrections to positions
        self.move_to_correct_x_position(-2.38364)
        self.wait_for_sensors(10)
        self.move_to_correct_y_position(-0.148089)
        self.wait_for_sensors(10)

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
        self.armMotors[3].setPosition(-1.32)
        self.step(100 * self.timestep)
        # Catch the Box
        self.finger1.setPosition(0.012)  
        self.finger2.setPosition(0.012)
        self.step(100 * self.timestep)
        # Get the Hand to Up 
        self.hand_up()
        self.step(100 * self.timestep)
        # Down the Hand to the Height of the Wall
        self.armMotors[2].setPosition(-0.90)
        self.armMotors[3].setPosition(-0.95)
        self.step(100 * self.timestep)
        # open the finger to Drop the Box
        self.finger1.setPosition(self.fingerMaxPosition)
        self.finger2.setPosition(self.fingerMaxPosition)
        self.step(100 * self.timestep)
        # return the to the basic place
        self.hand_up()
        self.step(100 * self.timestep)
        # change the Vlaue of 
        self.COUNTER_PICK_UP +=1
        self.send_boolean(self.cube)
        time.sleep(1)

    def return_from_red_to_node(self):
        self.wait_for_sensors(10)
        self.get_gps_position()
        time.sleep(0.5)
        while self.step(self.timestep) != -1 and self.current_x <-2.45:
            self.get_gps_position()
            print(f'Current x: {self.current_x}')
            self.move_backward(7.0)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(0.5)
        while self.step(self.timestep) != -1 and self.current_y >= 0.0:
            self.get_gps_position()
            print(f'Current Y: {self.current_y}')
            self.move_left(7.0)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(100)
        time.sleep(1)

    def return_from_blue_to_node(self):
        self.wait_for_sensors(10)
        self.get_gps_position()
        time.sleep(0.5)
        while self.step(self.timestep) != -1 and self.current_x <-2.45:
            self.get_gps_position()
            print(f'Current x: {self.current_x}')
            self.move_backward(7.0)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(0.5)
        while self.step(self.timestep) != -1 and self.current_y >= 0.0:
            self.get_gps_position()
            print(f'Current Y: {self.current_y}')
            self.move_left(7.0)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(0.5)

    def return_from_yellow_to_node(self):
        self.wait_for_sensors(10)
        print('finish Turning')
        self.get_gps_position()
        time.sleep(0.5)
        while self.step(self.timestep) != -1 and self.current_x <-2.45:
            self.get_gps_position()
            print(f'Current x: {self.current_x}')
            self.move_backward(7.0)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(0.5)
        while self.step(self.timestep) != -1 and self.current_y <= 0.0:
            self.get_gps_position()
            print(f'Current Y: {self.current_y}')
            self.move_right(7.0)
        self.stop_movement()
        self.stop_robot()
        self.wait_for_sensors(10)
        time.sleep(0.5)

    def go_from_node_to_wall(self):
        self.get_gps_position()
        self.wait_for_sensors(10)
        self.turn_left_180_degrees()
        time.sleep(0.5)
        self.wait_for_sensors(10)
        while self.step(self.timestep) != -1 and self.current_x < -0.55:
             self.get_gps_position()
             print(f'Current X: {self.current_x}')
             self.move_forward(self.max_velocity)    
        self.stop_movement()
        self.stop_robot()
        # add corrections to positions
        self.move_to_correct_x_position_Opposite(-0.523046)
        self.wait_for_sensors(10)
        self.move_to_correct_y_position_Opposite(-0.132864)
        self.wait_for_sensors(10)
        time.sleep(1)


robot = RobotController()
# # robot.pick_up_form_basket()
# robot.start()
# robot.print_color_queue()

# for color in robot.color_queue:
#     if color == 'Red':
#         print("\nThe Color Is Now RED\n")
#         robot.go_to_red()
#         time.sleep(1)
#         # check again
#         robot.get_gps_position()
#         robot.move_to_position(target_x = -3.30, target_y = 3.76234)
#         robot.pick_up()
#         robot.return_from_red_to_node()
#         robot.go_from_node_to_wall()
#         robot.move_to_correct_y_position_Opposite(target_y= -0.132864)
#         robot.pick_up_form_basket()
#         robot.return_from_wall_to_node()

#     elif color == 'Blue':
#         print("\nThe Color Is Now Blue\n")
#         robot.go_to_blue()
#         time.sleep(1)
#         robot.get_gps_position()
#         robot.move_to_position(target_x = -3.30, target_y = 1.25)
#         robot.pick_up()
#         robot.return_from_blue_to_node()
#         robot.go_from_node_to_wall()
#         robot.move_to_correct_y_position_Opposite(target_y= -0.132864)
#         robot.pick_up_form_basket()
#         robot.return_from_wall_to_node()
#     elif color == 'Green':
#         print("\nThe Color Is Now Green \n")
#         robot.go_to_green()
#         time.sleep(1)
#         robot.get_gps_position()
#         robot.move_to_position(target_x = -3.305, target_y = -4.07)
#         robot.pick_green_and_retrun_to_node()
#         robot.go_from_node_to_wall()
#         robot.move_to_correct_y_position_Opposite(target_y= -0.132864)
#         robot.pick_up_form_basket()
#         robot.return_from_wall_to_node()

#     elif color == 'Yellow':
#         print("\nThe Color Is Now Yellow \n")
#         robot.go_to_yellow()
#         robot.get_gps_position()
#         robot.move_to_position(target_x = -3.31, target_y = -1.46)
#         robot.pick_up()
#         robot.return_from_yellow_to_node()
#         robot.go_from_node_to_wall()
#         robot.move_to_correct_y_position_Opposite(target_y= -0.132864)
#         robot.pick_up_form_basket()
#         robot.return_from_wall_to_node()
#     else:
#         print('\nTheere is no Colors detect Bro\n')
#         break
    


    