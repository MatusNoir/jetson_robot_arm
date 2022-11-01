#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed under the
# terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time
import keyboard

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.messages import Base_pb2

class GripperCommandExample:
    def __init__(self, router, proportional_gain = 2.0):

        self.proportional_gain = proportional_gain
        self.router = router

        # Create base client using TCP router
        self.base = BaseClient(self.router)

    def ExampleSendGripperCommands(self, position):

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        print("Performing gripper test in position...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.finger_identifier = 1
        
        finger.value = position
        print("Going to position {:0.2f}...".format(finger.value))
        self.base.SendGripperCommand(gripper_command)


def main():
    # Import the utilities helper module
    import argparse
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    parser = argparse.ArgumentParser()
    args = utilities.parseConnectionArguments(parser)

    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        example = GripperCommandExample(router)
        position = 0.0

        while 1:
            
            key = keyboard.read_key()

            if key == "r":
                position += 0.1
                if position > 1.0:
                    position = 1.0

            elif key == "f":
                position -= 0.1
                if position < 0.0:
                    position = 0.0

            # home
            elif key == "g":
                position = 1.0
                while position > 0.0:
                    example.ExampleSendGripperCommands(position)
                    position -= 0.1
                    time.sleep(0.1)

                quit()

            example.ExampleSendGripperCommands(position)

if __name__ == "__main__":
    main()