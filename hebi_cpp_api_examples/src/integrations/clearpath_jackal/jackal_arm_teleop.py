#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from math import pi
from std_msgs.msg import String
from sensor_msgs.msg import Joy
from moveit_commander.conversions import pose_to_list

# Hardcoded waypoints:
DOOR = [-1.5, 1.35, 0.0, 1.2, -1.5, 0]
STOW = [-1.5, 2.9, 2.9, 1.57, -1.57, 0]
PICK = [-1.38, .95, 2.03, 2.71, -1.57, 0]
ABOVE_PICK = [-1.38, 1.45, 2.28, 2.81, -1.57, 0]
RIGHT = [-1.88, 1.45, 2.28, 2.81, -1.57, 0]
LEFT = [-0.88, 1.45, 2.28, 2.81, -1.57, 0]

# Arm buttons:
STOW_BUTTON = 14
LEFT_BUTTON = 11
RIGHT_BUTTON = 12
ABOVE_PICK_BUTTON = 13
PICK_BUTTON = 3
DOOR_BUTTON = 1

# Gripper button
GRIPPER_BUTTON = 0

# Button states
global last_buttons
last_buttons = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
global gripper_state
gripper_state = True

global demo

def joy_callback(data):
    global last_buttons
    global demo
    global gripper_state
    # Watch for edge triggers on relevant buttons
    if (data.buttons[STOW_BUTTON] == 1 and last_buttons[STOW_BUTTON] != 1):
        demo.go_to(STOW)
    elif (data.buttons[DOOR_BUTTON] == 1 and last_buttons[DOOR_BUTTON] != 1):
        demo.go_to(DOOR)
    elif (data.buttons[LEFT_BUTTON] == 1 and last_buttons[LEFT_BUTTON] != 1):
        demo.go_to(LEFT)
    elif (data.buttons[RIGHT_BUTTON] == 1 and last_buttons[RIGHT_BUTTON] != 1):
        demo.go_to(RIGHT)
    elif (data.buttons[PICK_BUTTON] == 1 and last_buttons[PICK_BUTTON] != 1):
        demo.go_to(PICK)
    elif (data.buttons[ABOVE_PICK_BUTTON] == 1 and last_buttons[ABOVE_PICK_BUTTON] != 1):
        demo.go_to(ABOVE_PICK)
    elif (data.buttons[GRIPPER_BUTTON] == 1 and last_buttons[GRIPPER_BUTTON] != 1):
        demo.change_gripper(gripper_state)
        gripper_state = not gripper_state
    last_buttons = data.buttons

class HebiClearpathDemo(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node('hebi_clearpath_demo', anonymous=True)
        rospy.Subscriber('bluetooth_teleop/joy', Joy, joy_callback)

        success = False
        while (not success and not rospy.is_shutdown()):
            try: 
                self.move_group = moveit_commander.MoveGroupCommander("hebi_arm", "/hebi/robot_description", ns="hebi")
                success = True
            except RuntimeError:
                print "Could not connect to move group action server for arm; trying again"

        success = False
        while (not success and not rospy.is_shutdown()):
            try: 
                self.gripper_move_group = moveit_commander.MoveGroupCommander("hand", "/hebi/robot_description", ns="hebi")
                success = True
            except RuntimeError:
                print "Could not connect to move group action server for gripper; trying again"
            

    def go_to(self, goal):
        self.move_group.go(goal, wait=True)
        self.move_group.stop()

    def change_gripper(self, gripper_state):
        if (gripper_state):
            # Closed
            self.gripper_move_group.go([0.022494, 0.022494, 1.1675, 0.403302, 1.1675, 0.403302])
        else:
            # Open
            self.gripper_move_group.go([-1.145, -1.145, 0.0, 1.570796, 0.0, 1.570796])

def main():

    try:
        global demo
        print "Starting HEBI Clearpath Demo"
        demo = HebiClearpathDemo()

        while(not rospy.is_shutdown()):
            pass

    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
