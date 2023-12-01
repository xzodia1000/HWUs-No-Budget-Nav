from controller import Supervisor
import random
import sys


class SupervisorController:
    def __init__(self):
        # Simulation Parameters
        # Please, do not change these parameters
        self.time_step = 32  # ms
        self.time_experiment = 60  # s

        # Initiate Supervisor Module
        self.supervisor = Supervisor()
        # Check if the robot node exists in the current world file
        self.robot_node = self.supervisor.getFromDef("Controller")
        if self.robot_node is None:
            sys.stderr.write("No DEF Controller node found in the current world file\n")
            sys.exit(1)
        # Get the robots translation and rotation current parameters
        self.trans_field = self.robot_node.getField("translation")
        self.rot_field = self.robot_node.getField("rotation")

        # Check Receiver and Emitter are enabled
        self.emitter = self.supervisor.getDevice("emitter")
        self.receiver = self.supervisor.getDevice("receiver")
        self.receiver.enable(self.time_step)

        # Get the reward node
        self.reward_node = self.supervisor.getFromDef("OBS_FINAL")

        if self.reward_node is None:
            sys.stderr.write("No DEF OBS_FINAL node found in the current world file\n")
            sys.exit(1)

        # Get the reward translation and rotation current parameters
        self.reward_field = self.reward_node.getField("translation").getSFVec3f()

        # Get the spot light node
        self.spot_node = self.supervisor.getFromDef("SPOTLIGHT")

        # Set the spot light node to be off randomly
        light_on_or_off = False if random.randint(0, 1) == 0 else True
        self.spot_node.getField("on").setSFBool(light_on_or_off)


        # Encode the message
        self.emitterData = str(self.reward_field[0])+ ',' + str(self.reward_field[1])


    def handle_receiver(self):
        #print(self.receiver.getQueueLength())
        while (self.receiver.getQueueLength() > 0):
            self.receivedData = self.receiver.getString()

            # Check Message
            if (self.receivedData == "reward"):
                self.handle_emitter()
            self.receiver.nextPacket()

    def handle_emitter(self):
        # Send the message
        string_message = str(self.emitterData)
        string_message = string_message.encode("utf-8")
        self.emitter.send(string_message)

if __name__ == "__main__":
    # Call Supervisor function to initiate the supervisor module
    supervisor_controller = SupervisorController()

    while supervisor_controller.supervisor.step(supervisor_controller.time_step) != -1:
        supervisor_controller.handle_receiver()
