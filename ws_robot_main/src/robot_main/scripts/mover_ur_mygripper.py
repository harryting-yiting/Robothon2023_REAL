#!/usr/bin/env python

from __future__ import print_function
from cmath import pi
from gettext import translation
from logging.handlers import WatchedFileHandler
#from multiprocessing.connection import wait
import re
from shutil import move
from turtle import position
from xml.etree.ElementTree import PI

#from torch import matmul
#from ros_tcp_endpoint.src.ros_tcp_endpoint.publisher import RosPublisher
#from ros_tcp_endpoint.src.ros_tcp_endpoint.publisher import RosPublisher

#from torch import dtype
#from multiprocessing.connection import wait
#from multiprocessing.connection import wait

import rospy
import numpy as np
import sys
import copy
import math
import moveit_commander

import moveit_msgs.msg
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion, Pose
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from std_msgs.msg import Bool
import tf
import tf.transformations as tfTrans
import tf2_ros
import tf_conversions

import actionlib

#from niryo_moveit.srv import MoverService, MoverServiceRequest, MoverServiceResponse

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Between Melodic and Noetic, the return type of plan() changed. moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan
        
"""
    Given the start angles of the robot, plan a trajectory that ends at the destination pose.
"""
def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    current_joint_state = JointState()
    current_joint_state.name = joint_names
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state
    move_group.set_start_state(moveit_robot_state)

    move_group.set_pose_target(destination_pose)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)

    return planCompat(plan)


"""
    Creates a pick and place plan using the four states below.
    
    1. Pre Grasp - position gripper directly above target object
    2. Grasp - lower gripper so that fingers are on either side of object
    3. Pick Up - raise gripper back to the pre grasp position
    4. Place - move gripper to desired placement position

    Gripper behaviour is handled outside of this trajectory planning.
        - Gripper close occurs after 'grasp' position has been achieved
        - Gripper open occurs after 'place' position has been achieved

    https://github.com/ros-planning/moveit/blob/master/moveit_commander/src/moveit_commander/move_group.py
"""
def plan_multiple_pick_and_place(req):
    response = MoverServiceResponse()
    response.planningResult.data = False

    group_name = "robot"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    for i in range(0, req.vector_len):
        current_robot_joint_configuration = move_group.get_current_joint_values()

        if(not single_pick_place_excute(move_group, current_robot_joint_configuration, req.pick_pose_vector[i], req.place_pose_vector[i])):
            return response

    response.planningResult.data = True
    return response

def convertGeoMsgVectorToArray(msgVector):
    return [msgVector.x, msgVector.y, msgVector.z]

def convertGeoMsgQuaternionToArray(msgQuaternion):
    return [msgQuaternion.x, msgQuaternion.y, msgQuaternion.z, msgQuaternion.w]

def convertGeometryPoseToArray(geomPoseMsg):
    return convertGeoMsgVectorToArray(geomPoseMsg.position), convertGeoMsgQuaternionToArray(geomPoseMsg.orientation)

def convertGeometryTransformToArray(geomTransfMsg):
    return convertGeoMsgVectorToArray(geomTransfMsg.translation), convertGeoMsgQuaternionToArray(geomTransfMsg.rotation)


# def transObjPoseToGripper(objPose):

#     objMatrix = tf_conversions.toMatrix(tf_conversions.fromMsg(objPose))
#     orintation2 = tfTrans.euler_matrix(0, pi/2, -pi/2, "rxyz")

    # return np.matmul(objMatrix, orintation2)

def transGripperToFlange(gripperMatrix):
    
    tfBuffer = tf2_ros.Buffer()
    tfTransformer = tf2_ros.TransformListener(tfBuffer)
    #if tfTransformer.canTransform('/link6_flange','/gripper_pick_center', rospy.Time(0)):
    gpcToFlangeTansform =  tfBuffer.lookup_transform('link6_flange','gripper_pick_center', rospy.Time(0),rospy.Duration(0.2))
    pose = geometry_msgs.msg.Pose()
    pose.orientation = gpcToFlangeTansform.transform.rotation
    pose.position = gpcToFlangeTansform.transform.translation
    rospy.loginfo('Transform from gpc %s', pose )
    gripperToFlange = tf_conversions.toMatrix( tf_conversions.fromMsg(pose))

    wToGPC =np.matmul( gripperMatrix, gripperToFlange)

    return tf_conversions.toMsg(tf_conversions.fromMatrix(wToGPC))

# def transObjPoseToFlangePose(objPose):

#     gripperMatrix= transObjPoseToGripper(objPose)

#     rospy.loginfo("gripper pose")
#     rospy.loginfo('%s', tf_conversions.toMsg( tf_conversions.fromMatrix(gripperMatrix)))
#     return transGripperToFlange(gripperMatrix)

def transObjPoseToFlangePose(objPose):
    objMatrix = tf_conversions.toMatrix(tf_conversions.fromMsg(objPose))
    moveup = tfTrans.translation_matrix([0, 0, 0.04])
    rotatey180 = tfTrans.euler_matrix(0, pi, 0, 'rxyz')

    return tf_conversions.toMsg(tf_conversions.fromMatrix(np.matmul(objMatrix,np.matmul(moveup, rotatey180))))

def single_pick_place_excute(move_group, current_joints ,pick_pose_obj, place_pose_obj):
    
    rospy.loginfo("[Planning] Planning Trajectory 1")
    rospy.loginfo("%s",pick_pose_obj)
    rospy.loginfo("%s",transObjPoseToFlangePose(pick_pose_obj))
    pre_grasp_pose = plan_trajectory(move_group, transObjPoseToFlangePose(pick_pose_obj), current_joints)

    # If the trajectory has no points, planning has failed and we return an empty response
    if not pre_grasp_pose.joint_trajectory.points:
        return False

    previous_ending_joint_angles = pre_grasp_pose.joint_trajectory.points[-1].positions

     # Grasp - lower gripper so that fingers are on either side of object
    pick_pose = copy.deepcopy(pick_pose_obj)
    pick_pose.position.z -= (0.09 ) # Static value coming from Unity, TODO: pass along with request
    rospy.loginfo("[Planning] Planning Trajectory 2")
    grasp_pose = plan_trajectory(move_group, transObjPoseToFlangePose(pick_pose), previous_ending_joint_angles)
    
    if not grasp_pose.joint_trajectory.points:
        return False

    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions

    # Pick Up - raise gripper back to the pre grasp position
    print("[Planning] Planning Trajectory 3")
    pick_up_pose = plan_trajectory(move_group,  transObjPoseToFlangePose(pick_pose_obj), previous_ending_joint_angles)
    
    if not pick_up_pose.joint_trajectory.points:
        return False

    previous_ending_joint_angles = pick_up_pose.joint_trajectory.points[-1].positions

    # Place - move gripper to desired placement position
    print("[Planning] Planning Trajectory 4")
    place_pose = plan_trajectory(move_group, transObjPoseToFlangePose(place_pose_obj), previous_ending_joint_angles)

    if not place_pose.joint_trajectory.points:
        return False
    # If trajectory planning worked for all pick and place stages, add plan to response

    pre_grasp_pose.joint_trajectory.points[0].positions = move_group.get_current_joint_values()

    move_group.execute(pre_grasp_pose, wait=True)

    tool_name = "gripper"
    tool_group = moveit_commander.MoveGroupCommander(tool_name)

    tool_group.set_named_target("open")
    tool_group.go(wait=True)

    grasp_pose.joint_trajectory.points[0].positions = move_group.get_current_joint_values()
    move_group.execute(grasp_pose, wait=True)

    tool_group.set_named_target("close")
    tool_group.go(wait=True)

    pick_up_pose.joint_trajectory.points[0].positions = move_group.get_current_joint_values()
    move_group.execute(pick_up_pose, wait=True)

    place_pose.joint_trajectory.points[0].positions = move_group.get_current_joint_values()
    move_group.execute(place_pose, wait=True)

    tool_group.set_named_target("open")
    tool_group.go(wait=True)

    return True

def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('niryo_moveit_server')

    s = rospy.Service('niryo_moveit', MoverService, plan_multiple_pick_and_place)
    print("Ready to plan")
    rospy.spin()


if __name__ == "__main__":
    print("Ready to plan")
    moveit_server()
