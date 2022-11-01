#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###
import sys
import os
import time
import threading
import serial

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# make port
port_name = 'SerialPortForPython'
sp = serial.Serial(port_name, 9600)

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check
 
def example_move_to_home_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    # print(action_list)
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

stop_observe_feedback = False
def observe_feedback(base_cyclic):
    global stop_observe_feedback
    stop_observe_feedfack = False
    while not stop_observe_feedback:
        feedback = base_cyclic.RefreshFeedback()
        print('World Positoin(m) x {:.2f} y {:.2f} z {:.2f}'.format(feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z))
        print('World Eular Rotation(deg) x {:.2f} y {:.2f} z {:.2f}'.format(feedback.base.tool_pose_theta_x, feedback.base.tool_pose_theta_y, feedback.base.tool_pose_theta_z), end='\n\n')
        #print('Positoin x(m)', feedback.base.tool_pose_x , 'y(m)', feedback.base.tool_pose_y, 'z(m)', feedback.base.tool_pose_z, '\nEular x(deg)', feedback.base.tool_pose_theta_x, 'y(deg)', feedback.base.tool_pose_theta_y, 'z(deg)', feedback.base.tool_pose_theta_z, end='\n\n')
        time.sleep(0.1)

def example_cartesian_action_movement(base, base_cyclic, px, py, pz, eux, euy, euz):
    
    print("Starting Cartesian action movement ...")
    action = Base_pb2.Action()
    action.name = "Example Cartesian action movement"
    action.application_data = ""

    feedback = base_cyclic.RefreshFeedback()

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = px        # (meters)
    cartesian_pose.y = py        # (meters)
    cartesian_pose.z = pz        # (meters)
    cartesian_pose.theta_x = eux # (eular degrees)
    cartesian_pose.theta_y = euy # (eular degrees)
    cartesian_pose.theta_z = euz # (eular degrees)

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing action")
    base.ExecuteAction(action)
    
    global stop_observe_feedback
    stop_observe_feedback = False
    th_observe_feedback = threading.Thread(target=observe_feedback, args1=(base_cyclic,))
    # th_observe_feedback.start()

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        stop_observe_feedback = True
        print("Cartesian movement completed")
    else:
        print("Timeout on action notification wait")
        
    # th_observe_feedback.join()
    return finished

class GripperCommandExample:
    def __init__(self, base, proportional_gain = 2.0):

        self.proportional_gain = proportional_gain
        # self.router = router

        # Create base client using TCP router
        self.base = base
        
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
    # args1 = utilities.parseConnectionArguments()
    # parser = argparse.ArgumentParser()
    args1 = utilities.parseConnectionArguments()
    # cable
    # args1.ip = "192.168.2.13"
    # wireless
    # args1.ip = "192.168.1.10"
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args1) as router1:
        # with
        # router1 = utilities.DeviceConnection.createTcpConnection(args1)
        # router1.transport.connect(router1ipAddress, router1.port) 
        # Create required services
        base1 = BaseClient(router1)
        base_cyclic1 = BaseCyclicClient(router1)

        # Example core
        success = True

        success &= example_move_to_home_position(base1)
        feedback = base_cyclic1.RefreshFeedback()
        print('World Positoin(m) x {:.2f} y {:.2f} z {:.2f}'.format(feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z))
        print('World Eular Rotation(deg) x {:.2f} y {:.2f} z {:.2f}'.format(feedback.base.tool_pose_theta_x, feedback.base.tool_pose_theta_y, feedback.base.tool_pose_theta_z), end='\n\n')
        
        example = GripperCommandExample(base1)
        gpos = 0.0

        while 1:
            # px, py, pz, eux, euy, euz = map(float, input(">> ").split())
            
            # read from serialport
            # num = line.decode("utf-8")
            num = sp.read(7)

            px = float(num[0]) / 100
            py = float(num[1]) / 100
            pz = float(num[2]) / 100
            eux = float(num[3])
            euy = float(num[4])
            euz = float(num[5])
            gpos = float(num[6]) / 10

            success &= example_cartesian_action_movement(base1, base_cyclic1, px, py, pz, eux, euy, euz)
            example.ExampleSendGripperCommands(gpos)
            
            # success &= example_cartesian_action_movement(base1, base_cyclic1, px, py, pz, eux, euy, euz)
            # example.ExampleSendGripperCommands(gpos)
        # You can also refer to the 110-Waypoints examples if you want to execute
        # a trajectory defined by a series of waypoints in joint space or in Cartesian space

        sp.close()
        return 0 if success else 1

        # with end

if __name__ == "__main__":
    exit(main())
