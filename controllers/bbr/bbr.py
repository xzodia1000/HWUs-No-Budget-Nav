from controller import Robot
import math
import numpy as np


class Controller:
    def __init__(self, robot):
        # Robot Parameters
        self.robot = robot
        self.time_step = 32  # ms
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

        # Emitters and receivers
        self.emitter = self.robot.getDevice("emitter")
        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(self.time_step)

        self.reward_coordinates = None

        self.distance_coef_a = 0.5
        self.distance_coef_b = -1.3

        # Data
        self.inputs = []
        self.inputsPrevious = []
        self.reward_coordinates = None

        # Line Following Flags
        self.follow_line_mode = True

        # Obstacle Avoidance Flags
        self.avoid_obstacle_mode = False
        self.distance = 0.0
        self.distance_tolerance = 0.1
        self.passed_white_area = False
        self.obstacles_passed = 0

        # Realign Flags
        self.realign_mode = False
        self.alternate_realign_counter = 1

        # Reward Zone Flags
        self.go_to_reward = False

        # Flag
        self.flag_turn = 0
        self.light_detected = 0
        self.is_turning = 0
        self.fork_flag = 0

        self.get_emitter_data()

    def is_in_zone(self, ir_value, ir_value_previous):
        current = self.get_current_distance(ir_value)
        previous = self.get_current_distance(ir_value_previous)

        # Check if the robot is at the reward zone
        if current < 10 and np.isclose(current, previous, atol=0.1):
            return True

        return False

    def get_emitter_data(self):
        # Read the data from the supervisor
        string_message = str("reward")
        string_message = string_message.encode("utf-8")
        self.emitter.send(string_message)

        if self.receiver.getQueueLength() > 0:
            self.reward_coordinates = self.receiver.getString().split(',')

    def turn_right(self):
        # Flags used to stop the robot from autocorrecting itself during the turn
        self.is_turning = True
        self.flag_turn = 0

        # Make the left wheel go forward and the right wheel go backwards
        self.velocity_left = 1.0
        self.velocity_right = -1.0

    def turn_left(self):
        self.velocity_left = 1.0
        self.velocity_right = -1.0

    def is_at_distance(self, ir_value):
        # Check if the robot is at a certain distance from the obstacle
        distance = self.distance_coef_a * math.pow(ir_value, self.distance_coef_b)

        if distance < 80:
            print("Switch to obstacle avoidance mode")
            self.distance = distance
            return True
        else:
            return False

    def is_on_line(self):
        # Check all pairs of sensors if they are on the line
        if (self.inputs[0] < 0.35 and self.inputs[1] < 0.35) or \
                (self.inputs[0] < 0.35 and self.inputs[2] < 0.35) or \
                (self.inputs[1] < 0.35 and self.inputs[2] < 0.35):
            return True
        else:
            self.passed_white_area = True
            return False

    def get_current_distance(self, ir_value):
        # Calculate the distance from the IR sensor
        return self.distance_coef_a * math.pow(ir_value, self.distance_coef_b)

    def avoid_obstacle(self):
        # Check the sides of the robot
        front_wall = self.get_current_distance(self.inputs[11]) < 80 or self.get_current_distance(self.inputs[17]) < 80
        left_wall = self.get_current_distance(self.inputs[16]) < 80
        right_wall = self.get_current_distance(self.inputs[13]) < 80
        back_wall_right = self.get_current_distance(self.inputs[14]) < 80
        back_wall_left = self.get_current_distance(self.inputs[15]) < 80
        no_wall = not (front_wall or left_wall or right_wall or back_wall_right or back_wall_left)

        if not self.light_detected:

            # If the robot has passed the obstacle
            if self.is_on_line() and self.passed_white_area:

                # Ensure that the robot is not too close to the wall
                if not back_wall_right:
                    self.velocity_left = -self.max_speed
                    self.velocity_right = self.max_speed

                # If the robot is far enough from the wall, switch to line realign mode
                if back_wall_left or back_wall_right or no_wall:
                    print("Switch to line realign mode")
                    self.velocity_left = -self.max_speed
                    self.velocity_right = self.max_speed
                    self.realign_mode = True
                    self.avoid_obstacle_mode = False
                    self.passed_white_area = False
                    self.is_turning = False

            # If the robot is still traversing the obstacle
            else:
                if front_wall:
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
            # If the robot has passed the obstacle
            if self.is_on_line() and self.passed_white_area:

                # Ensure that the robot is not too close to the wall
                if not back_wall_left:
                    self.velocity_left = self.max_speed
                    self.velocity_right = -self.max_speed

                # If the robot is far enough from the wall, switch to line realign mode
                if back_wall_left or back_wall_right or no_wall:
                    print("Switch to line realign mode")
                    self.velocity_left = self.max_speed
                    self.velocity_right = -self.max_speed
                    self.realign_mode = True
                    self.avoid_obstacle_mode = False
                    self.passed_white_area = False
                    self.is_turning = False
            else:
                if front_wall:
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
        # Check the sides of the robot
        back_wall_right = self.get_current_distance(self.inputs[14]) < 80
        back_wall_left = self.get_current_distance(self.inputs[15]) < 80

        back_wall_detected = back_wall_left or back_wall_right

        if back_wall_detected:
            # Keep moving left and right until the robot is aligned with the line
            if (self.alternate_realign_counter % 2) == 0:
                # Turn left
                self.velocity_left = self.max_speed / 4
                self.velocity_right = self.max_speed
            if (self.alternate_realign_counter % 2) == 1:
                # Turn right
                self.velocity_left = self.max_speed
                self.velocity_right = self.max_speed / 4

            self.alternate_realign_counter += 1

        # Moving too far away from the line on the right
        if self.inputs[2] > 0.7:
            self.velocity_left = self.max_speed / 4
            self.velocity_right = self.max_speed
        elif self.inputs[0] > 0.7:
            self.velocity_left = self.max_speed
            self.velocity_right = self.max_speed / 4

        else:
            self.velocity_left = self.max_speed
            self.velocity_right = self.max_speed

        # If the robot is back on the line, switch off the realign mode
        if (self.inputs[0] < 0.35) and (self.inputs[2] < 0.35):

            # If both obstacles have been passed, switch to head to reward zone mode
            if self.obstacles_passed == 1:
                self.obstacles_passed += 1
                self.realign_mode = False
            if self.obstacles_passed == 0:
                self.obstacles_passed += 1
                self.realign_mode = False
                self.follow_line_mode = True

        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)

    def follow_line(self):
        # If robot is sensing something
        if len(self.inputs) > 0 and len(self.inputsPrevious) > 0:

            ir_value = np.max(self.inputs[3:11])
            # Check for any possible collision
            if ir_value > 0.4:

                # Check for Obstacle, if so, switch to obstacle avoidance mode
                if self.is_at_distance(self.inputs[11]):
                    self.avoid_obstacle_mode = True
                    self.follow_line_mode = False
                    return

                # Check for Light
                if self.inputs[3] == 0 and self.inputs[4] == 0 and self.inputs[9] == 0 and self.inputs[10] == 0:
                    self.light_detected = 1
                elif self.inputs[5] == 0:
                    self.light_detected = 1
                elif self.inputs[8] == 0:
                    self.light_detected = 1
                elif self.inputs[6] == 0 and self.inputs[7] == 0:
                    self.light_detected = 1

                # Check for Fork
                self.fork_flag = np.isclose(self.inputs[0], 0.76, atol=0.1) and np.isclose(self.inputs[1], 0.76,
                                                                                           atol=0.1) and np.isclose(
                    self.inputs[2], 0.76, atol=0.1)

                # If light is on, go left
                if self.light_detected and self.fork_flag:
                    print("Turning Left")
                    self.turn_left()

                # If light is off, go right
                if (not self.light_detected) and self.fork_flag:
                    print("Turning Right")
                    self.turn_right()

            # Automatically adjust to follow line
            if self.flag_turn:
                self.velocity_left = -0.3
                self.velocity_right = 0.3
                if np.min(self.inputs[0:3]) < 0.35:
                    self.flag_turn = 0
            else:
                # Check end of line
                if (np.min(self.inputs[0:3]) - np.min(self.inputsPrevious[0:3])) > 0.2:
                    self.flag_turn = 1
                else:
                    # Follow the line
                    if not self.is_turning:
                        # left < center and left < right (i.e. left is on the line)
                        if self.inputs[2] > 0.7:
                            # Turn left
                            self.velocity_left = self.max_speed / 4
                            self.velocity_right = self.max_speed

                        # center < left and center < right (i.e. center is on the line)
                        elif self.inputs[1] < self.inputs[0] and self.inputs[1] < self.inputs[2]:
                            # Go straight
                            self.velocity_left = 1
                            self.velocity_right = 1

                        # right < left and right < center (i.e. right is on the line)
                        elif self.inputs[0] > 0.7:
                            # Turn right
                            self.velocity_left = self.max_speed
                            self.velocity_right = self.max_speed / 4

        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)
        self.is_turning = False

    def head_to_reward_zone(self):
        # If robot is sensing something
        if len(self.inputs) > 0 and len(self.inputsPrevious) > 0:

            # Check if the robot is at the reward zone
            if self.is_in_zone(self.inputs[11], self.inputsPrevious[11]):
                print("Robot Stopped")

                # Stop the robot
                self.velocity_left = 0
                self.velocity_right = 0
                self.time_step = 0

            ir_value = np.max(self.inputs[3:11])
            # Check for any possible collision
            if ir_value > 0.4:

                # Check for Light
                if self.inputs[3] == 0 and self.inputs[4] == 0 and self.inputs[9] == 0 and self.inputs[10] == 0:
                    self.light_detected = 1
                elif self.inputs[5] == 0:
                    self.light_detected = 1
                elif self.inputs[8] == 0:
                    self.light_detected = 1
                elif self.inputs[6] == 0 and self.inputs[7] == 0:
                    self.light_detected = 1

                # Check for Fork
                self.fork_flag = np.isclose(self.inputs[0], 0.76, atol=0.1) and np.isclose(self.inputs[1], 0.76,
                                                                                           atol=0.1) and np.isclose(
                    self.inputs[2], 0.76, atol=0.1)

                # If light is on, go left
                if self.light_detected and self.fork_flag:
                    print("Turning Left")
                    self.turn_left()

                # If light is off, go right
                if (not self.light_detected) and self.fork_flag:
                    print("Turning Right")
                    self.turn_right()

            # Automatically adjust to follow line
            if self.flag_turn:
                self.velocity_left = -0.3
                self.velocity_right = 0.3
                if np.min(self.inputs[0:3]) < 0.35:
                    self.flag_turn = 0
            else:
                # Check end of line
                if (np.min(self.inputs[0:3]) - np.min(self.inputsPrevious[0:3])) > 0.2:
                    self.flag_turn = 1
                else:
                    # Follow the line
                    if not self.is_turning:
                        # left < center and left < right (i.e. left is on the line)
                        if self.inputs[2] > 0.7:
                            # Turn left
                            self.velocity_left = self.max_speed / 4
                            self.velocity_right = self.max_speed

                        # center < left and center < right (i.e. center is on the line)
                        elif self.inputs[1] < self.inputs[0] and self.inputs[1] < self.inputs[2]:
                            # Go straight
                            self.velocity_left = 1
                            self.velocity_right = 1

                        # right < left and right < center (i.e. right is on the line)
                        elif self.inputs[0] > 0.7:
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
            if left > max_gs: left = max_gs
            if center > max_gs: center = max_gs
            if right > max_gs: right = max_gs
            if left < min_gs: left = min_gs
            if center < min_gs: center = min_gs
            if right < min_gs: right = min_gs

            # Save Data
            self.inputs.append((left - min_gs) / (max_gs - min_gs))  # 0
            self.inputs.append((center - min_gs) / (max_gs - min_gs))  # 1
            self.inputs.append((right - min_gs) / (max_gs - min_gs))  # 2

            # Read Light Sensors
            for i in range(8):
                temp = self.light_sensors[i].getValue()
                # Adjust Values
                min_ls = 0
                max_ls = 4300
                if temp > max_ls: temp = max_ls
                if temp < min_ls: temp = min_ls
                # Save Data
                self.inputs.append((temp - min_ls) / (max_ls - min_ls))  # 2 + i

            # Read Proximity Sensors
            for i in range(8):
                temp = self.proximity_sensors[i].getValue()
                # Adjust Values
                min_ls = 0
                max_ls = 4300
                if temp > max_ls: temp = max_ls
                if temp < min_ls: temp = min_ls
                # Save Data
                self.inputs.append((temp - min_ls) / (max_ls - min_ls))

            # Smooth filter (Average)
            smooth = 10
            if count == smooth:
                inputs_avg = [sum(x) for x in zip(*inputs_avg)]
                self.inputs = [x / smooth for x in inputs_avg]

                if self.reward_coordinates is None:
                    self.get_emitter_data()

                if self.follow_line_mode:
                    self.follow_line()

                if self.realign_mode:
                    self.realign()

                if self.avoid_obstacle_mode:
                    self.avoid_obstacle()

                if self.obstacles_passed == 2:
                    self.go_to_reward = True

                if self.go_to_reward:
                    self.head_to_reward_zone()

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
