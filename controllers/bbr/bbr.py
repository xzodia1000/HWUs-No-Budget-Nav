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
        
        # Data
        self.inputs = []
        self.inputsPrevious = []
        
        # Flag
        self.flag_turn = 0
        self.light_detected = 0
        self.is_turning = 0
        self.obstacle_cylinder_flag = False
        self.obstacle_box_flag = False
        self.left_cylinder_counter = 0
        self.left_box_counter = 0
        self.right_box_counter = 0
        self.right_cylinder_counter = 0
        self.crossed_obstacles = False

    def turn_right(self):
       # Adjust these values as needed
        self.is_turning = True
        self.flag_turn = 0
        self.velocity_left = 1.0
        self.velocity_right = -1.0

    def turn_left(self):
        # Adjust these values as needed
        self.velocity_left = 1.0
        self.velocity_right = -1.0

    def avoid_left_cylinder(self):
        if(self.left_cylinder_counter == 0):
            self.is_turning = True
            self.flag_turn = 0
            self.velocity_left = 2.0
            self.velocity_right = -2.0
        elif(self.left_cylinder_counter == 1):
            self.is_turning = True
            self.flag_turn = 0
            self.velocity_left = 2.0
            self.velocity_right = 2.0
        elif(self.left_cylinder_counter == 2):
            self.is_turning = True
            self.flag_turn = 0
            self.velocity_left = -1.0
            self.velocity_right = 1.0
        elif(self.left_cylinder_counter == 3):
            self.is_turning = True
            self.flag_turn = 0
            self.velocity_left = 2.0
            self.velocity_right = 2.0

    def avoid_left_box(self):
        if(self.left_box_counter == 0):
            self.is_turning = True
            self.flag_turn = 0
            self.velocity_left = 2.0
            self.velocity_right = -2.0
        elif(self.left_box_counter == 1):
            self.is_turning = True
            self.flag_turn = 0
            self.velocity_left = 2.0
            self.velocity_right = 2.0
        elif(self.left_box_counter == 2):
            self.is_turning = True
            self.flag_turn = 0
            self.velocity_left = -1.0
            self.velocity_right = 1.0
        elif(self.left_box_counter == 3):
            self.is_turning = True
            self.flag_turn = 0
            self.velocity_left = 2.0
            self.velocity_right = 2.0

    def avoid_right_box(self):
        if(self.right_obstacle_counter == 0):
            self.is_turning = True
            self.flag_turn = 0
            self.velocity_left = -2.0
            self.velocity_right = 2.0
        elif(self.right_obstacle_counter == 1):
            self.is_turning = True
            self.flag_turn = 0
            self.velocity_left = 4.0
            self.velocity_right = 4.0
        elif(self.right_obstacle_counter == 2):
            self.is_turning = True
            self.flag_turn = 0
            self.velocity_left = 1.0
            self.velocity_right = -1.0
        elif(self.right_obstacle_counter == 3):
            self.is_turning = True
            self.flag_turn = 0
            self.velocity_left = 3.0
            self.velocity_right = 3.0
        
    def clip_value(self,value,min_max):
        """
        Clip value between min and max

        Args:
            value (float): value to be clipped
            min_max (float): min and max value

        Returns:
            float: clipped value
        """
        if (value > min_max):
            return min_max;
        elif (value < -min_max):
            return -min_max;
        return value;

    def sense_compute_and_actuate(self):
        
        # If robot is sensing something
        if(len(self.inputs) > 0 and len(self.inputsPrevious) > 0):
            # Check for any possible collision
            #print(self.inputs)
            if(np.max(self.inputs[3:11]) > 0.4):
                # Time
                time = datetime.now()
                print("({} - {}) Object or walls detected with values {}".format(time.second, time.microsecond, self.inputs[11]))
                if(self.inputs[11] > 0.15):
                    self.obstacle_cylinder_flag = True
                elif (self.inputs[11] > 0.08):
                    self.obstacle_box_flag = True
            # Check for Light 
            if(self.inputs[3] == 0 and self.inputs[4] == 0 and self.inputs[9] == 0 and self.inputs[10] == 0):
                self.light_detected = 1
            elif(self.inputs[5] == 0):
                self.light_detected = 1
            elif(self.inputs[8] == 0):
                self.light_detected = 1
            elif(self.inputs[6] == 0 and self.inputs[7] == 0):
                self.light_detected = 1

            # Check for Fork and light
            # If true, go left
            if(self.light_detected and np.isclose(self.inputs[0], 0.76, atol=0.1) and np.isclose(self.inputs[1], 0.76, atol=0.1) and np.isclose(self.inputs[2], 0.76, atol=0.1)):
                print("Turning Left")
                self.turn_left()

            # Check for Fork and light
            # If false, go right
            if((not self.light_detected) and np.isclose(self.inputs[0], 0.76, atol=0.1) and np.isclose(self.inputs[1], 0.76, atol=0.1) and np.isclose(self.inputs[2], 0.76, atol=0.1)):
                print("Turning Right")
                self.turn_right()

            if (self.light_detected and self.obstacle_cylinder_flag):
                self.avoid_left_cylinder()
                self.left_cylinder_counter += 1

                if(np.min(self.inputs[0:3]) < 0.3 and self.left_cylinder_counter >= 3):
                    self.obstacle_cylinder_flag = False
                    self.is_turning = False
                    self.flag_turn = 1
                    # self.turn_right()
                    self.is_turning = True
                    self.flag_turn = 0
                    self.velocity_left = 5.0
                    self.velocity_right = -5.0

            if (self.light_detected and self.obstacle_box_flag):
                self.avoid_left_box()
                self.left_box_counter += 1

                if (np.min(self.inputs[0:3]) < 0.3 and self.left_box_counter >= 3):
                    self.obstacle_box_flag = False
                    self.is_turning = False
                    self.flag_turn = 1
                    # self.turn_right()
                    self.is_turning = True
                    self.flag_turn = 0
                    self.velocity_left = 5.0
                    self.velocity_right = -5.0
                    self.crossed_obstacles = True

            if (self.crossed_obstacles and np.isclose(self.inputs[0], 0.76, atol=0.1) and np.isclose(self.inputs[1], 0.76, atol=0.1) and np.isclose(self.inputs[2], 0.76, atol=0.1)):
                self.is_turning = True
                self.flag_turn = 0
                self.velocity_left = -2.0
                self.velocity_right = 2.0

            if (not self.light_detected and self.obstacle_box_flag):
                self.avoid_box_obstacle()
                self.right_box_counter += 1

                if (np.min(self.inputs[0:3]) < 0.3 and self.right_box_counter >= 3):
                    self.obstacle_box_flag = False
                    self.is_turning = False
                    self.flag_turn = 1
                    # self.turn_right()
                    self.is_turning = True
                    self.flag_turn = 0
                    self.velocity_left = -5.0
                    self.velocity_right = 5.0

            # Turn
            if(self.flag_turn):
                self.velocity_left = -0.3;
                self.velocity_right = 0.3;
                if(np.min(self.inputs[0:3])< 0.35):
                    self.flag_turn = 0
            else:
                self.obstacle_flag = False
                # Check end of line
                if((np.min(self.inputs[0:3])-np.min(self.inputsPrevious[0:3])) > 0.2):
                    self.flag_turn = 1
                else:    
                    # Follow the line    
                    if (not self.is_turning):
                        if (self.inputs[0] < self.inputs[1] and self.inputs[0] < self.inputs[2]):
                            self.velocity_left = 0.5;
                            self.velocity_right = 1;
                        elif (self.inputs[1] < self.inputs[0] and self.inputs[1] < self.inputs[2]):
                            self.velocity_left = 1;
                            self.velocity_right = 1;
                        elif (self.inputs[2] < self.inputs[0] and self.inputs[2] < self.inputs[1]):
                            self.velocity_left = 1;
                            self.velocity_right = 0.5;
     
        self.left_motor.setVelocity(self.velocity_left)
        self.right_motor.setVelocity(self.velocity_right)
        self.is_turning = False

    def run_robot(self):        
        # Main Loop
        count = 0;
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
            smooth = 30
            if(count == smooth):
                inputs_avg = [sum(x) for x in zip(*inputs_avg)]
                self.inputs = [x/smooth for x in inputs_avg]
                # Compute and actuate
                self.sense_compute_and_actuate()
                # Reset
                count = 0
                inputs_avg = []
                self.inputsPrevious = self.inputs
            else:
                inputs_avg.append(self.inputs)
                count = count + 1 

            # Compute and actuate
            #self.sense_compute_and_actuate()
                
            
if __name__ == "__main__":
    my_robot = Robot()
    controller = Controller(my_robot)
    controller.run_robot()
    