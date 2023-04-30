#! /usr/bin/env python
from __future__ import print_function
import rospy

import actionlib
import robot_main.msg

from RS485Gripper.rs485gp import RS485BusGripper
import time


def initializeGripper(timeout, gripper):
    gripper.InitializeGripper()
    nowTime = 0
    while(gripper.GetInitializeStatus() is not True):
        time.sleep(0.1)
        nowTime += 0.1
        if(nowTime > timeout):
            return False
    return True

class DHGripperAction(object):

    _feedback = robot_main.msg.dhGripperActionFeedback()
    _result = robot_main.msg.dhGripperActionResult()

    def __init__(self, name, gipper):
        self._action_name = name
        self._gripper = gripper
        self._as = actionlib.SimpleActionServer(self._action_name, robot_main.msg.dhGripperActionAction, 
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self._timeout = 10

    def execute_cb(self, goal):
        # Set 30% Grip force
        self._gripper.SetForceLimitPercentage_20_100(goal.force)

        # Set 50% Grip force
        self._gripper.SetSpeedLimitPercentage_1_100(goal.speed)

        # Go to position 500, (0-1000)
        self._gripper.SetPosition_0_1000(goal.position)

        success = True
        nowTime = 0
        while(self._gripper.ReadGraspState() != "Grasped"):
            time.sleep(0.1)
            nowTime += 0.1
            if(nowTime > self._timeout):
                success = False
                break
            self._feedback.moving = True
            self._as.publish_feedback(self._feedback)
        # Read the grasp status of the gripper
        # 'Moving':     Gripper moving
        # 'NoObject':   No griping load detected
        # 'Grasped':    Object grip force detected
        # 'Object Fall':No grip force when no release action is done
        self._result.grasped = success
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    gripper = RS485BusGripper(name='RS485Gripper',device='/dev/ttyUSB0', RSAddress=1,Baud = 115200)
    if(not initializeGripper(3, gripper)):
        rospy.logerr("Initilize Gripper Timeout")
    else:
        rospy.init_node('dhgripper_action')
        rospy.on_shutdown(gripper.Quit)
        server = DHGripperAction(rospy.get_name(), gripper)
        rospy.spin()
    
        
