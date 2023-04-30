# take command from cosolo ( start planning)
# move to camera pose
# Take image positions and catgory
# if two founded
#   transform to pose to robot base
#   move to star pose - offset
#   move to star pose - offset2 (Cartision)
#   open gripper
#   wait
#   close gripper
#   wait
#   move to end pose - offset
#   move to end pose - offset2 (Cartision)
#   open gripper
#   wait
#   close gripper
#   wait
#   move to camera pose
# repeat to 
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np
from math import pi
from std_msgs.msg import String
from std_msgs.msg import Bool
from moveit_commander.conversions import pose_to_list
import tf.transformations as tfTrans
import tf2_ros
import tf_conversions

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

from gripper.gripper import gripper_ctrl #Import Lib of gripper
import time

from detection_d455.msg import ObjectXY

def deg2rad(jps):
    newjps = []
    for j in jps:
        newjps.append( j*pi / 180)
    return newjps

def pixelToRobotBase(xPxiel, yPxiel):
    # x: 1280
    # y: 800
    x = 50/93 * yPxiel + 16000/93 -1
    y = 75/137 * xPxiel + -87675/137 -1.5

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = -x/1000 # Planning is based on Base Link, while, pose is on base
    pose_goal.position.y = -y/1000
    pose_goal.position.z = 0.13
    print(pose_goal.position.x,pose_goal.position.y)
    return pose_goal

def transObjPoseToFlangePose(objPose):
    objMatrix = tf_conversions.toMatrix(tf_conversions.fromMsg(objPose))
    #moveup = tfTrans.translation_matrix([0, 0,0])
    rotatey180 = tfTrans.euler_matrix(0, pi, 0, 'rxyz')

    return tf_conversions.toMsg(tf_conversions.fromMatrix(np.matmul(objMatrix, rotatey180)))

def goToGoalCartisian(goalPose, move_group):
    waypoints = []
    #waypoints.append(move_group.get_current_pose().pose)
    waypoints.append(copy.deepcopy(goalPose))
    (plan, fraction) = move_group.compute_cartesian_path(
    waypoints, 0.05, 0.0 ) # waypoints to follow  # eef_step
    move_group.execute(plan, wait=True)
    time.sleep(1)
    # move_group.stop()
    move_group.clear_pose_targets()
    
    return


def runCartesianWaypoints(cameraJPS, pickObjPose, placeObjPose, move_group):

    gripper_length = 0.11
    offset_hight = 0.05
    flangeOffset = gripper_length + offset_hight

    pickOffsetPose = copy.deepcopy(pickObjPose)
    pickOffsetPose.position.z += flangeOffset

    placeOffsetPose = copy.deepcopy(placeObjPose)
    placeOffsetPose.position.z += flangeOffset

    move_group.clear_pose_targets()
    move_group.go(deg2rad(cameraJPS), wait=True)
    move_group.clear_pose_targets()
    # go to offset
    goToGoalCartisian(pickOffsetPose,move_group)
    #move_group.set
    graspObject.release()
    time.sleep(0.2)
    graspObject.release()
    time.sleep(0.3)

    # go to Pick + deeper

    pickDeeper = copy.deepcopy(pickObjPose)
    pickDeeper.position.z -= 0.05
    goToGoalCartisian(pickObjPose,move_group)
    graspObject.grasp()
    time.sleep(0.5)

    # go to offset
    goToGoalCartisian(pickOffsetPose,move_group)

    # go to camera
    move_group.go(deg2rad(cameraJPS), wait=True)
    move_group.clear_pose_targets()
    # go to place offset
    goToGoalCartisian(placeOffsetPose,move_group)
    
    # go to place higher
    placeHigher = copy.deepcopy(placeObjPose)
    placeHigher.position.z += 0.03
    goToGoalCartisian(placeHigher,move_group)
    graspObject.release()
    time.sleep(0.2)
    graspObject.release()
    time.sleep(0.3)

    # go to place offset
    goToGoalCartisian(placeOffsetPose,move_group)
    # to go camera
    move_group.go(deg2rad(cameraJPS), wait=True)
    move_group.clear_pose_targets()

    return  

class moveRobot(object):
    def __init__(self, moveit_commander) -> None:
        self.startPlanning = False
        self.objecstPixelPose = ObjectXY()
        self.startPlanningSub = rospy.Subscriber("/StartPlanning",Bool, self.startPlanningCallback)
        self.pickPlacePixelSub = rospy.Subscriber("/pick_and_place_xy_position",ObjectXY, self.objectPixelPose)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.cameraJPS = [-243.17, -100.99, 112.79, -101.72,-89.86, -63.30]

        #debug
        self.pickPosePub = rospy.Publisher("pickPose", geometry_msgs.msg.PoseStamped, queue_size=1)
        self.pickPoseTransPub = rospy.Publisher("pickPoseTrans", geometry_msgs.msg.PoseStamped,queue_size=1)
        self.placePosePub = rospy.Publisher("placePose", geometry_msgs.msg.PoseStamped,queue_size=1)
        self.placePosetransPub = rospy.Publisher("placePoseTrans", geometry_msgs.msg.PoseStamped,queue_size=1)
        # print(self.move_group.get_planning_frame())
        return
    
    def drawPoses(self, pose1, pose2, pose3, pose4):
        poseStamped = geometry_msgs.msg.PoseStamped()
        poseStamped.header.frame_id = "base_link"
        poseStamped.pose = pose1
        self.pickPosePub.publish(poseStamped)
        poseStamped.pose = pose2
        self.pickPoseTransPub.publish(poseStamped)
        poseStamped.pose = pose3
        self.placePosePub.publish(poseStamped)
        poseStamped.pose = pose4
        self.placePosetransPub.publish(poseStamped)


    def startPlanningCallback(self, start: Bool):

        # debug      
        # while(1):
        #     pose_goal = geometry_msgs.msg.PoseStamped()
        #     pose_goal.header.frame_id="base_link"
        #     pose_goal.pose.orientation.w = 1.0
        #     pose_goal.pose.position.x = 0.7
        #     pose_goal.pose.position.y = -0.3
        #     pose_goal.pose.position.z = 0.2
        #     self.pickPosePub.publish(pose_goal)
        #     pose_goalTrans = geometry_msgs.msg.PoseStamped()
        #     pose_goalTrans.header = pose_goal.header
        #     pose_goalTrans.pose = transObjPoseToFlangePose(pose_goal.pose)
        #     self.pickPoseTransPub.publish(pose_goalTrans)
        #     rospy.sleep(0.05)

        self.startPlanning = start.data        
 
        if(self.startPlanning == True):
            print("start")
            if((self.objecstPixelPose.pick_object_X >=0 and self.objecstPixelPose.pick_object_Y >= 0
               and self.objecstPixelPose.place_object_X >=0 and self.objecstPixelPose.place_object_Y >=0)):
                    print("start Planning")

                    # pose_goal = geometry_msgs.msg.Pose()
                    # pose_goal.orientation.w = 1.0
                    # pose_goal.position.x = -0.4
                    # pose_goal.position.y = +0.4
                    # pose_goal.position.z = 0

                    # pose_goal2 = geometry_msgs.msg.Pose()
                    # pose_goal2.orientation.w = 1.0
                    # pose_goal2.position.x = -0.6
                    # pose_goal2.position.y = +0.5
                    # pose_goal2.position.z = 0

                    pickPose = pixelToRobotBase(self.objecstPixelPose.pick_object_X, 
                                                 self.objecstPixelPose.pick_object_Y)
                    placePose = pixelToRobotBase(self.objecstPixelPose.place_object_X, 
                                                 self.objecstPixelPose.place_object_Y)

                    # # # 
                    # pickPose = geometry_msgs.msg.Pose()
                    # pickPose.orientation.w = 1.0
                    # pickPose.position.x = -0.4
                    # pickPose.position.y = +0.4
                    # pickPose.position.z = 0.2

                    # placePose = geometry_msgs.msg.Pose()
                    # placePose.orientation.w = 1.0
                    # placePose.position.x = -0.3
                    # placePose.position.y = +0.5
                    # placePose.position.z = 0.2
                    
                    runCartesianWaypoints(self.cameraJPS, transObjPoseToFlangePose(pickPose), 
                                          transObjPoseToFlangePose(placePose), self.move_group)
                    #runCartesianWaypoints(self.cameraJPS, transObjPoseToFlangePose(pose_goal), 
                     #                     transObjPoseToFlangePose(pose_goal2), self.move_group)
        self.startPlanning = False
        return

    def objectPixelPose(self, msg: ObjectXY):
        self.objecstPixelPose = copy.deepcopy(msg)
        pickPose = pixelToRobotBase(self.objecstPixelPose.pick_object_X, 
                                                 self.objecstPixelPose.pick_object_Y)
        placePose = pixelToRobotBase(self.objecstPixelPose.place_object_X, 
                                                 self.objecstPixelPose.place_object_Y)
        
        self.drawPoses(pickPose, transObjPoseToFlangePose(pickPose), placePose, transObjPoseToFlangePose(placePose))
        return


if __name__ == "__main__":
    graspObject = gripper_ctrl(7,'/dev/ttyUSB0') 
    rospy.init_node("robothon_main", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    moveRobot(moveit_commander)
   
    
    rospy.spin()
    
    
    
    # pose_goal = geometry_msgs.msg.Pose()
    # pose_goal.orientation.w = 1.0
    # pose_goal.position.x = 0.4
    # pose_goal.position.y = 0.4
    # pose_goal.position.z = 0

    # pose_goal2 = geometry_msgs.msg.Pose()
    # pose_goal2.orientation.w = 1.0
    # pose_goal2.position.x = 0.2
    # pose_goal2.position.y = 0.2
    # pose_goal2.position.z = 0

    # runCartesianWaypoints(cameraJPS, transObjPoseToFlangePose(pose_goal), transObjPoseToFlangePose(pose_goal2), move_group)





