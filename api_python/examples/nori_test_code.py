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
import math
import sys
import os
import time
import threading
import numpy as np

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

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
    
    #global stop_observe_feedback
    #stop_observe_feedback = False
    # th_observe_feedback = threading.Thread(target=observe_feedback, args=(base_cyclic,))
    # th_observe_feedback.start();

    #print("Waiting for movement to finish ...")
    #finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)
    time.sleep(0.1)
    #base.StopAction()
    return 1

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
        time.sleep(0.1)

[ox, oy, oz, grip] = [0, 0, 0, 0]

convert_count = 0

def clamp(n,smallest,largest):
    return max(smallest,min(n,largest))

def convert():
    global ox, oy, oz, grip, convert_count
    import serial
    ox_old = 0
    oy_old = 0
    oz_old = 0
    port_name = '../../serial_out'

    sp = serial.Serial(port_name, 9600)
    while True:
        line = sp.read(4)
        convert_count += 1
        if convert_count > 1000:
            convert_count = 0
        # ox = float(line[2]) / 10 + 0.14
        # oy = float(line[0]) / 10 + 0.26
        # oz = float(line[1]) / 10 + 0.34
        oxyz = np.array([line[0], line[1], line[2], line[3]], dtype='int8')
        ox = oxyz[2] / 10 - ox_old
        oy = -oxyz[0] / 10 - oy_old
        oz = oxyz[1] / 10 - oz_old
        grip_value = oxyz[3] / 10
        if grip_value > 0.5:
            grip = 1
        else:
            grip = 0
        # print('---------------------------------------------')
        # print(ox, oy, oz)
        # ox = clamp(ox,-1,-0.1);
        # oy = clamp(oy,-1,-0.1);
        # oz = clamp(oz,-1,-0.1);
       
        # print(ox, oy, oz)
        # print('---------------------------------------')
    sp.close()

def main():
    
    global ox, oy, oz, grip, convert_count
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base.ClearFaults()
        time.sleep(3)	
        base_cyclic = BaseCyclicClient(router)

        # Example core
        success = True

        success &= example_move_to_home_position(base)
        feedback = base_cyclic.RefreshFeedback()
        print('World Positoin(m) x {:.2f} y {:.2f} z {:.2f}'.format(feedback.base.tool_pose_x, feedback.base.tool_pose_y, feedback.base.tool_pose_z))
        print('World Eular Rotation(deg) x {:.2f} y {:.2f} z {:.2f}'.format(feedback.base.tool_pose_theta_x, feedback.base.tool_pose_theta_y, feedback.base.tool_pose_theta_z), end='\n\n')
        eux = 0.44
        euy = 0.19
        euz = 0
        temp = 0

        example = GripperCommandExample(base)

        while 1:
            if temp != convert_count:
                
                success &= example_cartesian_action_movement(base, base_cyclic, eux + ox, euy + oy, euz + oz, 90, 0, 90)
                print(eux + ox, euy + oy, euz + oz, grip)

                # grip
                example = GripperCommandExample(base)
                example.ExampleSendGripperCommands(grip)

                temp = convert_count
            
        # You can also refer to the 110-Waypoints examples if you want to execute
        # a trajectory defined by a series of waypoints in joint space or in Cartesian space

        return 0 if success else 1

if __name__ == "__main__":
    thread_convert = threading.Thread(target=convert)
    thread_convert.start()
    
    exit(main())
