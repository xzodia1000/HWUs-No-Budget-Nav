from controller import Robot
from datetime import datetime
import math
import numpy as np


class Controller:
    def __init__(self, robot):        
        # Robot Parameters
        self.robot = robot
        self.time_step = 32 # ms
        self.max_speed = 1  # m/s
 
        # Enable Motors
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        self.velocity_left = 0
        self.velocity_right = 0
    
        # Enable Proximity Sensors
        # There are 8 sensors in total
        self.proximity_sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.proximity_sensors.append(self.robot.getDevice(sensor_name))
            self.proximity_sensors[i].enable(self.time_step)

        # Enable Light Sensors
        self.light_sensors = []
        for i in range(8):
            sensor_name = 'ls' + str(i)
            self.light_sensors.append(self.robot.getDevice(sensor_name))
            self.light_sensors[i].enable(self.time_step)
       
        # Enable Ground Sensors
        # There are 3 ground sensors
        self.left_ir = self.robot.getDevice('gs0')
        self.left_ir.enable(self.time_step)
        self.center_ir = self.robot.getDevice('gs1')
        self.center_ir.enable(self.time_step)
        self.right_ir = self.robot.getDevice('gs2')
        self.right_ir.enable(self.time_step)

        self.distance_coef_a = 0.5
        self.distance_coef_b = -1.3
        
        # Data
        self.inputs = []
        self.inputsPrevious = []

        # Line Following Flags
        self.follow_line_mode = True

        # Obstacle Avoidance Flags
        self.avoid_obstacle_mode = False
        self.distance = 0.0
        self.distance_tolerance = 0.1
        self.passed_white_area = False

        # Realign Flags
        self.realign_mode = False
        self.alternate_realign_counter = 1
        
        # Flag
        self.flag_turn = 0
        self.light_detected = 0
        self.is_turning = 0
        self.obstacle_cylinder_flag = 0
        self.obstacle_box_flag = 0
        self.left_cylinder_counter = 0
        self.left_box_counter = 0
        self.right_box_counter = 0
        self.right_cylinder_counter = 0
        self.crossed_obstacles = 0
        self.fork_flag = 0

    def turn_right(self):
        # Flags used to stop the robot from auto-correcting itself during the turn
        self.is_turning = True
        self.flag_turn = 0

        # Make the left wheel go forward and the right wheel go backwards
        self.velocity_left = 1.0
        self.velocity_right = -1.0

    def turn_left(self):
        # Adjust these values as needed
        self.velocity_left = 1.0
        self.velocity_right = -1.0

    def is_at_distance(self, ir_value):
        distance = self.distance_coef_a * math.pow(ir_value, self.distance_coef_b)

        if distance < 80:
            print("Switch to obstacle avoidance mode")
            self.distance = distance
            return True
        else:
            return False

    def is_on_line(self):
        # Check all pairs (0,1), (0,2), and (1,2) from self.inputs[0:3]
        if (self.inputs[0] < 0.35 and self.inputs[1] < 0.35) or \
                (self.inputs[0] < 0.35 and self.inputs[2] < 0.35) or \
                (self.inputs[1] < 0.35 and self.inputs[2] < 0.35):
            return True
        else:
            self.passed_white_area = True
            return False

    def get_current_distance(self, ir_value):
        return self.distance_coef_a * math.pow(ir_value, self.distance_coef_b)

    # def adjust_steering(self, towards_obstacle):
    #     if towards_obstacle:
    #         self.velocity_left = 6.8
    #         self.velocity_right = -6.8
    #     else:
    #         self.velocity_left = 0
    #         self.velocity_right = 0
    # def avoid_left_cylinder(self):
    #     # First turn right
    #     if(self.left_cylinder_counter == 0):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = 2.0
    #         self.velocity_right = -2.0
    #
    #     # Then walk straight
    #     elif(self.left_cylinder_counter == 1):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = 2.0
    #         self.velocity_right = 2.0
    #
    #     # Then turn left
    #     elif(self.left_cylinder_counter == 2):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = -1.0
    #         self.velocity_right = 1.0
    #
    #     # Then walk straight
    #     elif(self.left_cylinder_counter == 3):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = 2.0
    #         self.velocity_right = 2.0
    #
    # def avoid_left_box(self):
    #     # First turn right
    #     if(self.left_box_counter == 0):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = 2.0
    #         self.velocity_right = -2.0
    #
    #     # Then walk straight
    #     elif(self.left_box_counter == 1):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = 2.0
    #         self.velocity_right = 2.0
    #
    #     # Then turn left
    #     elif(self.left_box_counter == 2):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = -1.0
    #         self.velocity_right = 1.0
    #
    #     # Then walk straight
    #     elif(self.left_box_counter == 3):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = 2.0
    #         self.velocity_right = 2.0
    #
    #     #
    #
    # def avoid_right_box(self):
    #     # First turn left
    #     if(self.right_box_counter == 0):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = -2.0
    #         self.velocity_right = 2.0
    #
    #     # Then walk straight
    #     elif(self.right_box_counter == 1):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = 4.0
    #         self.velocity_right = 4.0
    #
    #     # Then turn right
    #     elif(self.right_box_counter == 2):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = 2.0
    #         self.velocity_right = -2.0
    #
    #     # Then walk straight
    #     elif(self.right_box_counter == 3 or self.right_box_counter == 4):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = 5.0
    #         self.velocity_right = 5.0
    #
    # def avoid_right_cylinder(self):
    #     # First turn left
    #     if (self.right_cylinder_counter == 0):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = -2.0
    #         self.velocity_right = 2.0
    #
    #     # Then walk straight
    #     elif (self.right_cylinder_counter == 1):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = 5.0
    #         self.velocity_right = 5.0
    #
    #     # Then turn right
    #     elif (self.right_cylinder_counter == 2):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = 1.0
    #         self.velocity_right = -1.0
    #
    #     # Then walk straight
    #     elif (self.right_cylinder_counter == 3):
    #         self.is_turning = True
    #         self.flag_turn = 0
    #         self.velocity_left = 2.0
    #         self.velocity_right = 2.0
    #
    # def clip_value(self, value, min_max):
    #     """
    #     Clip value between min and max
    #
    #     Args:
    #         value (float): value to be clipped
    #         min_max (float): min and max value
    #
    #     Returns:
    #         float: clipped value
    #     """
    #     if (value > min_max):
    #         return min_max
    #     elif (value < -min_max):
    #         return -min_max
    #     return value

    # def sense_compute_and_actuate(self):
    #     # This function runs once per time-step and decides what the robot should do next
    #
    #     # If robot is sensing something
    #     if(len(self.inputs) > 0 and len(self.inputsPrevious) > 0):
    #         # Check for any possible collision
    #         #print(self.inputs)
    #         if(np.max(self.inputs[3:11]) > 0.4):
    #             # Time
    #             time = datetime.now()
    #             print("({} - {}) Object or walls detected with values {}".format(time.second, time.microsecond, self.inputs[11]))
    #
    #             # Check for Obstacle
    #
    #             # Check for Cylinder
    #             if(self.inputs[11] > 0.15):
    #                 self.obstacle_cylinder_flag = True
    #
    #             # Check for Box
    #             elif (self.inputs[11] > 0.08):
    #                 self.obstacle_box_flag = True
    #
    #         # Check for Light
    #         if(self.inputs[3] == 0 and self.inputs[4] == 0 and self.inputs[9] == 0 and self.inputs[10] == 0):
    #             self.light_detected = 1
    #         elif(self.inputs[5] == 0):
    #             self.light_detected = 1
    #         elif(self.inputs[8] == 0):
    #             self.light_detected = 1
    #         elif(self.inputs[6] == 0 and self.inputs[7] == 0):
    #             self.light_detected = 1
    #
    #         # Check for Fork
    #         self.fork_flag = np.isclose(self.inputs[0], 0.76, atol=0.1) and np.isclose(self.inputs[1], 0.76, atol=0.1) and np.isclose(self.inputs[2], 0.76, atol=0.1)
    #
    #         # If light is on, go left
    #         if (self.light_detected and self.fork_flag):
    #             print("Turning Left")
    #             self.turn_left()
    #
    #         # If light is off, go right
    #         if ((not self.light_detected) and self.fork_flag):
    #             print("Turning Right")
    #             self.turn_right()
    #
    #         # If light is on, and the detected obstacle is the cylinder
    #         if (self.light_detected and self.obstacle_cylinder_flag):
    #             self.avoid_left_cylinder()
    #             self.left_cylinder_counter += 1
    #
    #             # If the robot is still not back on the line, keep turning
    #             if(np.min(self.inputs[0:3]) < 0.3 and self.left_cylinder_counter >= 3):
    #                 self.obstacle_cylinder_flag = False
    #                 self.is_turning = True
    #                 self.flag_turn = 0
    #                 self.velocity_left = 5.0
    #                 self.velocity_right = -5.0
    #
    #         # If light is on, and the detected obstacle is the box
    #         if (self.light_detected and self.obstacle_box_flag):
    #             self.avoid_left_box()
    #             self.left_box_counter += 1
    #
    #             print(self.inputs[0:3])
    #             if (np.min(self.inputs[0:3]) < 0.3 and self.left_box_counter >= 3):
    #                 self.obstacle_box_flag = False
    #                 self.is_turning = True
    #                 self.flag_turn = 0
    #                 self.velocity_left = 5.0
    #                 self.velocity_right = -5.0
    #                 self.crossed_obstacles = True
    #
    #         if (not self.light_detected and self.obstacle_box_flag):
    #             self.avoid_right_box()
    #             self.right_box_counter += 1
    #
    #             if (np.min(self.inputs[0:3]) < 0.3 and self.right_box_counter >= 3):
    #                 self.obstacle_box_flag = False
    #                 self.is_turning = True
    #                 self.flag_turn = 0
    #                 self.velocity_left = -5.0
    #                 self.velocity_right = 5.0
    #                 self.crossed_obstacles = True
    #
    #         if (not self.light_detected and self.obstacle_cylinder_flag):
    #             self.avoid_right_cylinder()
    #             self.right_cylinder_counter += 1
    #
    #             if (np.min(self.inputs[0:3]) < 0.3 and self.right_cylinder_counter >= 3):
    #                 self.obstacle_box_flag = False
    #                 self.is_turning = True
    #                 self.flag_turn = 0
    #                 self.velocity_left = -5.0
    #                 self.velocity_right = 5.0
    #                 self.crossed_obstacles = True
    #
    #
    #         if (self.crossed_obstacles and np.isclose(self.inputs[0], 0.76, atol=0.1) and np.isclose(self.inputs[1], 0.76, atol=0.1) and np.isclose(self.inputs[2], 0.76, atol=0.1)):
    #             self.is_turning = True
    #             self.flag_turn = 0
    #             self.velocity_left = -2.0
    #             self.velocity_right = 2.0
    #
    #
    #         # Turn
    #         if(self.flag_turn):
    #             self.velocity_left = -0.3
    #             self.velocity_right = 0.3
    #             if(np.min(self.inputs[0:3])< 0.35):
    #                 self.flag_turn = 0
    #         else:
    #             self.obstacle_flag = False
    #             # Check end of line
    #             if((np.min(self.inputs[0:3])-np.min(self.inputsPrevious[0:3])) > 0.2):
    #                 self.flag_turn = 1
    #             else:
    #                 # Follow the line
    #                 if (not self.is_turning):
    #                     if (self.inputs[0] < self.inputs[1] and self.inputs[0] < self.inputs[2]):
    #                         self.velocity_left = 0.5
    #                         self.velocity_right = 1
    #                     elif (self.inputs[1] < self.inputs[0] and self.inputs[1] < self.inputs[2]):
    #                         self.velocity_left = 1
    #                         self.velocity_right = 1
    #                     elif (self.inputs[2] < self.inputs[0] and self.inputs[2] < self.inputs[1]):
    #                         self.velocity_left = 1
    #                         self.velocity_right = 0.5
    #
    #     self.left_motor.setVelocity(self.velocity_left)
    #     self.right_motor.setVelocity(self.velocity_right)
    #     self.is_turning = False

    def avoid_obstacle(self):
        front_wall = self.get_current_distance(self.inputs[11]) < 80 or self.get_current_distance(self.inputs[17]) < 80
        left_wall = self.get_current_distance(self.inputs[16]) < 80
        right_wall = self.get_current_distance(self.inputs[13]) < 80
        back_wall_right = self.get_current_distance(self.inputs[14]) < 80
        back_wall_left = self.get_current_distance(self.inputs[15]) < 80
        no_wall = not (front_wall or left_wall or right_wall or back_wall_right or back_wall_left)

        if (not self.light_detected):
            if (self.is_on_line() and self.passed_white_area):
                if (not (back_wall_right)):
                    self.velocity_left = -self.max_speed
                    self.velocity_right = self.max_speed

                if (back_wall_left or back_wall_right or no_wall):
                    print("Switch to line realign mode")
                    self.velocity_left = -self.max_speed
                    self.velocity_right = self.max_speed
                    self.realign_mode = True
                    self.avoid_obstacle_mode = False
                    self.passed_white_area = False
                    self.is_turning = False
            else:
                if (front_wall):
                    # Turn left in place
                    self.velocity_left = -self.max_speed
                    self.velocity_right = self.max_speed
                else:
                    if right_wall:
                        # Go forward
                        self.velocity_left = self.max_speed
                        self.velocity_right = self.max_speed
                    else:
                        # Turn right
                        self.velocity_left = self.max_speed
                        self.velocity_right = self.max_speed / 4
        else:
            if (self.is_on_line() and self.passed_white_area):
                if (not (back_wall_left)):
                    self.velocity_left = self.max_speed
                    self.velocity_right = -self.max_speed

                if (back_wall_left or back_wall_right or no_wall):
                    print("Switch to line realign mode")
                    self.velocity_left = self.max_speed
                    self.velocity_right = -self.max_speed
                    self.realign_mode = True
                    self.avoid_obstacle_mode = False
                    self.passed_white_area = False
                    self.is_turning = False
            else:
                if (front_wall):
                    # Turn left in place
                    self.velocity_left = self.max_speed
                    self.velocity_right = -self.max_speed
                else:
                    if left_wall:
                        # Go forward
                        self.velocity_left = self.max_speed
                        self.velocity_right = self.max_speed
                    else:
                        # Turn left
                        self.velocity_left = self.max_speed / 4
                        self.velocity_right = self.max_speed

        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)

    def realign(self):
        back_wall_right = self.get_current_distance(self.inputs[14]) < 80
        back_wall_left = self.get_current_distance(self.inputs[15]) < 80

        back_wall_detected = back_wall_left or back_wall_right
        realigned = True

        if back_wall_detected:
            if (self.alternate_realign_counter % 2) == 0:
                # Turn left in place
                self.velocity_left = self.max_speed / 4
                self.velocity_right = self.max_speed
            if (self.alternate_realign_counter % 2) == 1:
                # Turn left in place
                self.velocity_left = self.max_speed
                self.velocity_right = self.max_speed / 4

            self.alternate_realign_counter += 1

        # Moving too far away from the line on the right
        if (self.inputs[2] > 0.7):
            self.velocity_left = self.max_speed/4
            self.velocity_right = self.max_speed
        elif (self.inputs[0] > 0.7):
            self.velocity_left = self.max_speed
            self.velocity_right = self.max_speed / 4
            realigned = False

        else:
            self.velocity_left = self.max_speed
            self.velocity_right = self.max_speed

        if (self.inputs[0] < 0.35) and (self.inputs[2] < 0.35):
            self.realign_mode = False
            self.follow_line_mode = True

        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)
    def follow_line(self):
        # If robot is sensing something
        if (len(self.inputs) > 0 and len(self.inputsPrevious) > 0):

            ir_value = np.max(self.inputs[3:11])
            # Check for any possible collision
            if (ir_value > 0.4):

                if(self.is_at_distance(self.inputs[11])):
                    self.avoid_obstacle_mode = True
                    self.follow_line_mode = False
                    return

                # Check for Light
                if (self.inputs[3] == 0 and self.inputs[4] == 0 and self.inputs[9] == 0 and self.inputs[10] == 0):
                    self.light_detected = 1
                elif (self.inputs[5] == 0):
                    self.light_detected = 1
                elif (self.inputs[8] == 0):
                    self.light_detected = 1
                elif (self.inputs[6] == 0 and self.inputs[7] == 0):
                    self.light_detected = 1

                # Check for Fork
                self.fork_flag = np.isclose(self.inputs[0], 0.76, atol=0.1) and np.isclose(self.inputs[1], 0.76,
                                                                                           atol=0.1) and np.isclose(
                    self.inputs[2], 0.76, atol=0.1)

                # If light is on, go left
                if (self.light_detected and self.fork_flag):
                    print("Turning Left")
                    self.turn_left()

                # If light is off, go right
                if ((not self.light_detected) and self.fork_flag):
                    print("Turning Right")
                    self.turn_right()

            # Automatically adjust to follow line
            if (self.flag_turn):
                self.velocity_left = -0.3
                self.velocity_right = 0.3
                if (np.min(self.inputs[0:3]) < 0.35):
                    self.flag_turn = 0
            else:
                # Check end of line
                if ((np.min(self.inputs[0:3]) - np.min(self.inputsPrevious[0:3])) > 0.2):
                    self.flag_turn = 1
                else:
                    # Follow the line
                    if (not self.is_turning):
                        # left < center and left < right (i.e. left is on the line)
                        if (self.inputs[2] > 0.7):
                            # Turn left
                            self.velocity_left = self.max_speed / 4
                            self.velocity_right = self.max_speed

                        # center < left and center < right (i.e. center is on the line)
                        elif (self.inputs[1] < self.inputs[0] and self.inputs[1] < self.inputs[2]):
                            # Go straight
                            self.velocity_left = 1
                            self.velocity_right = 1

                        # right < left and right < center (i.e. right is on the line)
                        elif (self.inputs[0] > 0.7):
                            # Turn right
                            self.velocity_left = self.max_speed
                            self.velocity_right = self.max_speed / 4

        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)
        self.is_turning = False
    def run_robot(self):        
        # Main Loop
        count = 0
        inputs_avg = []
        while self.robot.step(self.time_step) != -1:
            # Read Ground Sensors
            self.inputs = []
            left = self.left_ir.getValue()
            center = self.center_ir.getValue()
            right = self.right_ir.getValue()

            # Adjust Values
            min_gs = 0
            max_gs = 1000
            if(left > max_gs): left = max_gs
            if(center > max_gs): center = max_gs
            if(right > max_gs): right = max_gs
            if(left < min_gs): left = min_gs
            if(center < min_gs): center = min_gs
            if(right < min_gs): right = min_gs
            
            # Save Data
            self.inputs.append((left-min_gs)/(max_gs-min_gs)) # 0
            self.inputs.append((center-min_gs)/(max_gs-min_gs)) # 1
            self.inputs.append((right-min_gs)/(max_gs-min_gs)) # 2
            #print("Ground Sensors \n    left {} center {} right {}".format(self.inputs[0],self.inputs[1],self.inputs[2]))

           # Read Light Sensors
            for i in range(8):       
                temp = self.light_sensors[i].getValue()
                # Adjust Values
                min_ls = 0
                max_ls = 4300
                if(temp > max_ls): temp = max_ls
                if(temp < min_ls): temp = min_ls
                # Save Data
                self.inputs.append((temp-min_ls)/(max_ls-min_ls)) # 2 + i
                #print("Light Sensors - Index: {}  Value: {}".format(i,self.light_sensors[i].getValue()))

            # Read Proximity Sensors
            for i in range(8):
                temp = self.proximity_sensors[i].getValue()
                # Adjust Values
                min_ls = 0
                max_ls = 4300
                if(temp > max_ls): temp = max_ls
                if(temp < min_ls): temp = min_ls
                # Save Data
                self.inputs.append((temp-min_ls)/(max_ls-min_ls))
                #print("Proximity Sensors - Index: {}  Value: {}".format(i,self.proximity_sensors[i].getValue()))

            # Smooth filter (Average)
            smooth = 10
            if(count == smooth):
                inputs_avg = [sum(x) for x in zip(*inputs_avg)]
                self.inputs = [x/smooth for x in inputs_avg]

                if self.follow_line_mode:
                    # Compute and actuate
                    self.follow_line()

                if self.realign_mode:
                    self.realign()

                if self.avoid_obstacle_mode:
                    # Compute and actuate
                    self.avoid_obstacle()


                # Reset
                count = 0
                inputs_avg = []
                self.inputsPrevious = self.inputs
            else:
                inputs_avg.append(self.inputs)
                count = count + 1
                
            
if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()
    