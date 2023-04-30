import roslib
import rospy
import rtde_control
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
from RS485Gripper.rs485gp import RS485BusGripper
from rtde_control import RTDEControlInterface as RTDEControl
from rtde_receive import RTDEReceiveInterface as RTDEReceive
import time
import math
import rospy
from geometry_msgs.msg import Pose2D

from scipy.linalg import expm
from detection_msgs.srv import DetectTriangles

def rotationMandXYZtoTranfM(rotationM: np.ndarray, xyz)-> np.ndarray :
    transformaiton = np.identity(4)
    transformaiton[0:3, 3] = xyz[0:3]
    transformaiton[0:3, 0:3] = rotationM
    return transformaiton


def xyzRpyToMatrix(xyzrpy, degree: bool, extrinsic: bool) -> np.ndarray: 
    # x,y,z, RPY, extrinsic
    if extrinsic:
        rotationM = R.from_euler('xyz',xyzrpy[3:6] , degree).as_matrix()
    else:
        rotationM = R.from_euler('XYZ',xyzrpy[3:6] , degree).as_matrix()
    return rotationMandXYZtoTranfM(rotationM, xyzrpy[0:3])


def matrixToXyzRotationAngle(transf: np.ndarray):
    xyzrpy = [0,0,0,0,0,0]
    xyzrpy[0:3] = transf[0:3, 3].tolist()
    return R.from_matrix(transf).as_rotvec().tolist()


def taskPoses(baseXyzRpy, relativePoses):
    # TM: tranformation matrix
    baseTM = xyzRpyToMatrix(baseXyzRpy, False, False)
    taskPoseList = []
    for rp in relativePoses:
        taskPoseList.append(baseTM @ xyzRpyToMatrix(rp, False, False))
    return taskPoseList


def moveAbove(TM, zOffset):
    return  xyzRpyToMatrix([0, 0, zOffset, 0, 0, 0], False, True) @ TM


def tcpToFlange(mission):
    # mission: 0: press button
    #          1: slider
    #          2: insert hole
    #          3: open gate
    #          4: pick pen
    if mission == 0:
        return [-0.005, -0.0682,0.1646, 0, 0, 0]
    if mission == 1:
        return [0, -0.042, 0.16393, 0,0,0]
    if mission == 2:
        return [0, -0.047, 0.1646, 0,0,0]
    if mission == 4:
        return [0, -0.072, 0.1585, 0, 0, 0]
    return [0, 0, 0, 0 ,0, 0]


def taskPoseToTcpPose(taskPose):
    raletiveP = xyzRpyToMatrix([0,0,0,0,180,90], True, False)
    return taskPose @ raletiveP


def taskPoseListToTcpList(taskPoseList):
    tcpList = []
    for tp in taskPoseList:
        tcpList.append(taskPoseToTcpPose(tp))
    return tcpList


def tmToXyzRA(tm):
    rotvec = R.from_matrix(tm[0:3, 0:3]).as_rotvec().tolist()
    t = tm[0:3, 3].tolist()
    return t + rotvec

def tmListToTRA(tm_list):
    TRA_list = []
    for tm in tm_list:
        TRA_list.append(tmToXyzRA(tm))
    return TRA_list

def addSpeedAccBlendToPath(path, speed, acc, blend):
    newPath = []
    for point in path:
        point.extend([speed, acc, blend])
        newPath.append(point)
    return newPath

def transfIntrinsric(tm, xyzrpy):
    return tm @ xyzRpyToMatrix(xyzrpy, True, False)

def transfIntrinsicList(tm_list, xyzrpy):
    new_tms = []
    for tm in tm_list:
        new_tms.append(transfIntrinsric(tm, xyzrpy))
    return new_tms

def transfExtrensic(tm, xyzrpy):
    return xyzRpyToMatrix(xyzrpy, True, True) @ tm


def w_hat(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def e_w(w, theta):
    return expm(w_hat(w)*theta)


def TM_about_a_line(w, point, theta):
    w = np.array(w)
    point = np.array(point)
    v = -np.cross(w, point)
    rotatM = e_w(w, theta)
    trans = (np.identity(3) - rotatM) @ (np.cross(w, v))
    tm = np.identity(4)
    tm[0:3, 0:3] = rotatM
    tm[0:3, 3] = trans
    return tm

 
def rotate_about_a_line(w, w_p, source_p, angle):
    source_p = source_p+[1]
    final_p = np.dot(TM_about_a_line(w, w_p, angle), np.array([source_p]).reshape(4,1))
    return final_p[0:3]


def rotate_line_path(w, w_p, source_p, start_angle, end_angle, step):
    points = []
    for a in np.arange(start_angle,end_angle,step):
        final_p = rotate_about_a_line(w, w_p, source_p, a)
        test = np.vstack((final_p, np.array([[0],[0],[0]])))
        points.extend(np.transpose( test).tolist())
    return points


def intersection_two_lines_2D(point1, rotate1, point2, rotate2):
    k0 = math.tan(rotate1)
    k1 = math.tan(rotate2)

    x0 = point1[0]
    y0 = point1[1]

    x1 = point2[0]
    y1 = point2[1]

    x = (k0*x0 -k1*x1 + y1 - y0) / (k0 - k1)
    y = k0 * (x - x0) + y0
    return [x, y]


def computeVectorAngleDegree(point_head, point_tail):
    head = np.array(point_head).reshape((2))
    tail = np.array(point_tail).reshape((2))
    v = head - tail
    return math.atan2(v[1], v[0])


def rotat_around_Z_extrinsic(vector, rad):
    v = e_w([0,0,1], rad) @ np.array(vector).reshape((3,1))
    return v.reshape(1, 3).tolist()


def degreeListToRad(dl):
    rad = []
    for d in dl:
        rad.append(math.radians(d))
    return rad


recived = False
boardPose = Pose2D()
arrivePhotoPose = False
def forceMoveZDown(msg):
    global arrivePhotoPose
    if arrivePhotoPose:
        global boardPose
        boardPose = msg
        global recived
        recived = True
    return


# x,y,z,r,p,y, intrinsic
blueBotton =   [0      , 0     , 0     , 0, 0, 0]
redBotton =    [-0.018      , 0     , 0     , 0, 0, 0]
blackHole =    [0      , 0.0579, 0.005 , 0, 0, 0]
redHole =      [-0.0245, 0.0579, 0.005 , 0, 0, 0]
#doorHandle = [-0.012, 0.143, 0.0, 0,0,0]
#sliderInit = [-0.093, ,0.009, 0,0,0]
#sliderMiddle =[-0.079, ,0.009, 0, 0, 0]
penInit =      [-0.0521, 0.1979 , 0.021 , 0, 0, 0]
measurePoint = [-0.055 , 0.155 , 0.00  , 0, 0, 0]
#measurePoint = [-0.057 , 0.153 , 0.04  , 0, 0, 0]
gateHandle =   [-0.011 , 0.1433, 0.0088 + 0.005, 0, 0, 0]   # may not accurate
slider_y = 0.035
slider_z = 0.011
slider_width = 0.011
slider_white_init  = [-0.094 , slider_y, slider_z, 0, 0, 0]   # may not accurate
slider_white_mid   = [-0.079 , slider_y, slider_z, 0, 0, 0]
slider_white_end   = [-0.064 , slider_y, slider_z, 0, 0, 0]

contact_x =    [0      , -0.035 , -0.002, 0, 0, 0]
contact_x_2 =  [-0.08  , -0.035 , -0.002, 0, 0, 0]
contact_y =    [-0.13    , 0.05 , -0.002, 0, 0, 0]


gateRotateAxis = [0, -1, 0]
gateRotateAxisPoint = [-0.075,0, 0]

pen_head_length = 0.04

leftEar =      []
rightEar =     []

class mission(object):

    def __init__(self):
        self.default_speed = 1.0
        self.default_acc = 0.8

        self.default_rot_speed = 3.1
        self.default_rot_acc = 3.0


        self.default_hight = 0.05

        self.G_close = 660
        self.G_open = 850
        self.G_open_little = 750

        self.G_open_move_slide = 680
        self.G_open_gate = 670
        self.G_wait_time = 0.5 # second
        self.Gripper = RS485BusGripper(name='RS485Gripper',device='/dev/ttyUSB0', RSAddress=1,Baud = 115200)

        # RTDE Connection
        rtde_frequency = 500.0
        ur_ip = "192.168.56.101"
        self.rtde_c = RTDEControl(ur_ip, rtde_frequency, RTDEControl.FLAG_USE_EXT_UR_CAP)
        self.rtde_r = RTDEReceive(ur_ip, rtde_frequency)

        # Pre-defined Poses
        # Two different kinds of poses
        # axis angel: xyzAA == TAA
        # eular: xyzRPY == TRPY
        
        self.task_relative_poses = [blueBotton,  blackHole, redHole, penInit, measurePoint, redBotton]
        self.task_poses = []
        self.task_tcp_poses = []

        self.flange_to_tcp = {'press_button': [-0.005, -0.0682, 0.1646 , 0, 0, 0],
                              'move_slider':  [0     , -0.0642 , 0.1621, 0, 0, 0],
                              'insert_hole':  [0     , -0.047 , 0.1646 , 0, 0, 0],
                              'pick_pen':     [0     , -0.0717 , 0.1576 , 0, 0, 0],
                              'measure':      [0     , -0.0717, 0.1576 , 0, 0, 0],
                              'open_door':   [0     , -0.0237, 0.1576 , 0, 0, 0],
                              'default':      [0     , 0      , 0      , 0, 0, 0]}

        self.resetJps = [-4.5, -71, -113, -85, 90, 23]
        # self.rotatePen_MidJPS = [-18.97, -70.65, -131.32, 200.90, -33.53, 1.20]
        self.rotatePen_MidJPS = [-3.98, -112, -70, -143, 64, -17]

    def forceMove(self, task_frame, selection_vector, wrench_down, force_type, limits, time):
        dt = 1.0/500  # 2ms
        # Execute 500Hz control loop for 4 seconds, each cycle is 2ms
        for i in range(int(time / dt)):
            t_start = self.rtde_c.initPeriod()
            # First move the robot down for 2 seconds, then up for 2 seconds
            self.rtde_c.forceMode(task_frame, selection_vector, wrench_down, force_type, limits)
            self.rtde_c.waitPeriod(t_start)
        return


    def forceMoveZDown(self,time, force, speed, xyComplience = 0):
        task_frame = [0, 0, 0, 0, 0, 0]
        selection_vector = [xyComplience, xyComplience, 1, 0, 0, 0]
        wrench_down = [0, 0, -force, 0, 0, 0]
        force_type = 2
        limits = [2, 2, speed, 1, 1, 1]
        self.forceMove(task_frame, selection_vector, wrench_down, force_type, limits, time)
        self.rtde_c.forceModeStop()
        return


    def forceMoveYTaskFrame(self,time, force, speed, task_frame, xComplience = 0):
        selection_vector = [xComplience, 1, 0, 0, 0, 0]
        wrench_down = [0, force, 0, 0, 0, 0]
        force_type = 2
        limits = [2, speed, 2, 1, 1, 1]
        self.forceMove(task_frame, selection_vector, wrench_down, force_type, limits, time)
        self.rtde_c.forceModeStop()
        return
    
    def dis_two_matrix(self, tm1, tm2):
        return np.linalg.norm(tm1, tm2)
    
    def dis_two_list(self, l1, l2):
        return np.linalg.norm(np.array(l1)-np.array(l2))
#-------------------------------Tasks--------------------------------#  
    def move_to_photo_pose(self, photo_tcp_TAA, start=True):
        self.rtde_c.setTcp([0,0,0,0,0,0])
        currentTRA = self.rtde_r.getActualTCPPose()
        if self.dis_two_list(currentTRA, photo_tcp_TAA) > 1e-1 :
            self.move_to_reset_JPS()
            print("distance initial")
            print(self.dis_two_list(currentTRA, photo_tcp_TAA))
        self.rtde_c.moveJ_IK(photo_tcp_TAA, self.default_rot_speed, self.default_rot_acc)
        return
    

    def initialize_gripper(self, force, speed):

        # force = 40
        # speed = 40
        self.Gripper.InitializeGripper()
        time.sleep(2)
            # Set 30% Grip force
        self.Gripper.SetForceLimitPercentage_20_100(force)
        # Set 50% Grip force
        self.Gripper.SetSpeedLimitPercentage_1_100(speed)
        # Go to position 500, (0-1000)
        self.Gripper.SetPosition_0_1000(self.G_close)
        
        return
    

    def move_to_reset_JPS(self):
        currentJPS = self.rtde_r.getActualQ()
        print(currentJPS)
        targetJPS = degreeListToRad(self.resetJps)
        currentJPS[-1] = targetJPS[-1]
        self.rtde_c.moveJ(currentJPS, self.default_rot_speed, self.default_rot_acc)
        self.rtde_c.moveJ(targetJPS, self.default_rot_speed, self.default_rot_acc)


    def move_to_rotatePen_mid_point(self):
        self.rtde_c.moveJ(degreeListToRad(self.rotatePen_MidJPS), self.default_rot_speed, self.default_rot_acc)


    def task_press_button(self, button_pose_tcp):

        self.Gripper.SetPosition_0_1000(self.G_close)
        targetTcp = button_pose_tcp
        self.rtde_c.setTcp(self.flange_to_tcp['press_button'])
        #self.rtde_c.moveL(tmToXyzRA(moveAbove(targetTcp, 0.05)), self.default_speed, self.default_acc)
        self.rtde_c.moveL(tmToXyzRA(moveAbove(transfIntrinsric(targetTcp, [0,0,0,0,0,90]), 0.05)), self.default_speed, self.default_acc)
        self.rtde_c.moveL(tmToXyzRA(moveAbove(transfIntrinsric(targetTcp, [0,0,0,0,0,90]), 0.015)), self.default_speed, self.default_acc)
        self.forceMoveZDown(1.7, 10, 3)
        self.rtde_c.moveL(tmToXyzRA(moveAbove(transfIntrinsric(targetTcp, [0,0,0,0,0,90]), self.default_hight)), self.default_speed, self.default_acc)


    def task_press_blue_button(self):
        targetTcp = self.task_tcp_poses[0]
        self.task_press_button(targetTcp)
        return
    

    def task_press_red_button(self):
        targetTcp = self.task_tcp_poses[5]
        self.task_press_button(targetTcp)
        return
    

    def task_pick_black_insert_red_hole(self):

        self.rtde_c.setTcp(self.flange_to_tcp['insert_hole'])
        black_hole_tcp =  self.task_tcp_poses[1]
        red_hole_tcp =  self.task_tcp_poses[2]

        # [Pick Black]
        self.rtde_c.moveL(tmToXyzRA(moveAbove(black_hole_tcp, self.default_hight)),self.default_speed, self.default_acc)
        self.Gripper.SetPosition_0_1000(self.G_open_little)
        time.sleep(self.G_wait_time)
        self.rtde_c.moveL(tmToXyzRA(black_hole_tcp), 0.25, 0.3)
        # Go to position 500, (0-1000)
        self.Gripper.SetPosition_0_1000(self.G_close)
        time.sleep(self.G_wait_time)

        self.rtde_c.moveL(tmToXyzRA(moveAbove(black_hole_tcp, self.default_hight)), self.default_speed, self.default_acc)

        # [Insert Red]
        self.rtde_c.moveL(tmToXyzRA(moveAbove(red_hole_tcp, self.default_hight)), self.default_speed, self.default_acc)
        self.rtde_c.moveL(tmToXyzRA(moveAbove(red_hole_tcp, self.default_hight / 2)), self.default_speed, self.default_acc)
        

        # task_frame = [0, 0, 0, 0, 0, 0]
        # selection_vector = [0, 0, 1, 0, 0, 0]
        # wrench_down = [0, 0, -15, 0, 0, 0]
        # force_type = 2
        # limits = [2, 2, 2, 1, 1, 1]
        # dt = 1.0/500  # 2ms
        # # Execute 500Hz control loop for 4 seconds, each cycle is 2ms
        # for i in range(500):
        #     t_start = rtde_c.initPeriod()
        #     # First move the robot down for 2 seconds, then up for 2 seconds
        #     rtde_c.forceMode(task_frame, selection_vector, wrench_down, force_type, limits)
        #     rtde_c.waitPeriod(t_start)
        
        self.forceMoveZDown(1, 15, 2)

        # Spiral
        task_frame = [0, 0, 0, 0, 0, 0]
        selection_vector = [1, 1, 1, 0, 0, 0]
        wrench_down = [0, 0, -20, 0, 0, 0]
        limits = [2, 2, 2, 1, 1, 1]
        force_type = 2
        spiralTurns = 5
        spiralForce = 6
        # dt = 2ms
        for i in range(700):
            t_start = self.rtde_c.initPeriod()
            # First move the robot down for 2 seconds, then up for 2 seconds
            wrench_down[0] = spiralForce * math.sin(i/2000*spiralTurns*6.28)
            wrench_down[1] = spiralForce * math.cos(i/2000*spiralTurns*6.28)
            self.rtde_c.forceMode(task_frame, selection_vector, wrench_down, force_type, limits)
            self.rtde_c.waitPeriod(t_start)
        self.rtde_c.forceModeStop()


        # r#edHoleRotate = transfIntrinsric(tcpList[2], [0,0,0,0,0,90])
        # rtde_c.moveL(tmToXyzRA(redHoleRotate), default_speed, default_acc)
        self.Gripper.SetPosition_0_1000(self.G_open)
        time.sleep(self.G_wait_time)

        self.rtde_c.moveL(tmToXyzRA(moveAbove(red_hole_tcp, self.default_hight)), self.default_speed, self.default_acc)
        return
    

    def task_pull_pen(self):

        self.rtde_c.setTcp(self.flange_to_tcp['pick_pen'])
        pull_pen_tcp = self.task_tcp_poses[3]
        penOffset = transfIntrinsric(pull_pen_tcp, [0,0.01,0,0,0,0])
        # previous actoin many be very high
        targetTRA = tmToXyzRA(penOffset)
        currentTRA = self.rtde_r.getActualTCPPose()
        targetTRA[2] = currentTRA[2]
        self.rtde_c.moveL(targetTRA, self.default_speed, self.default_acc)
        self.Gripper.SetPosition_0_1000(self.G_open)
        time.sleep(0.2)

        self.rtde_c.moveL(tmToXyzRA(penOffset), self.default_speed, self.default_acc)

        # Close Pen A little bit 
        self.Gripper.SetPosition_0_1000(690)
        time.sleep(0.2)

        self.forceMoveYTaskFrame(1, -10, 3, tmToXyzRA(penOffset))
        penOutOffset = transfIntrinsric(pull_pen_tcp, [0,0.03,0,0,0,0])
        self.Gripper.SetPosition_0_1000(self.G_close)
        time.sleep(0.3)

        self.rtde_c.moveL(tmToXyzRA(penOutOffset), self.default_speed, self.default_acc)

        penOutAbove = moveAbove( penOutOffset, 0.2)
        self.rtde_c.moveL(tmToXyzRA(penOutAbove), self.default_speed, self.default_acc)


    def task_insert_pen_back(self):
        self.rtde_c.setTcp(self.flange_to_tcp['pick_pen'])
        pull_pen_tcp = self.task_tcp_poses[3]
        penOffset = transfIntrinsric(pull_pen_tcp, [0,0.08,0,0,0,0])
        self.rtde_c.moveL(tmToXyzRA(moveAbove(penOffset, 0.05)), self.default_speed, self.default_acc)
        self.rtde_c.moveL(tmToXyzRA(penOffset), self.default_speed, self.default_acc)
        penOffset = transfIntrinsric(pull_pen_tcp, [0,0.03,0,0,0,0])
        self.rtde_c.moveL(tmToXyzRA(penOffset), self.default_speed, self.default_acc)

        self.forceMoveYTaskFrame(1.8, -15, 4, tmToXyzRA(penOffset))
 
        self.Gripper.SetPosition_0_1000(self.G_open)
        time.sleep(0.2)

        # out
        penOutOffset = transfIntrinsric(pull_pen_tcp, [0,0.03,0,0,0,0])
        self.rtde_c.moveL(tmToXyzRA(penOutOffset), self.default_speed, self.default_acc)
        penOutAbove = moveAbove( penOutOffset, self.default_hight)
        self.rtde_c.moveL(tmToXyzRA(penOutAbove), self.default_speed, self.default_acc)


    def task_prepare_probe(self):   

        self.move_to_rotatePen_mid_point()
        probo_tcp = self.task_tcp_poses[4]
        #rotatedPen = transfIntrinsric(probo_tcp, [0, 0, 0, -90, 180.0 - boardPose.theta*180.0/3.14159 + 45.0, 0])
        probo_tcp_TRA = tmToXyzRA(probo_tcp)
        probo_position = probo_tcp_TRA[0:3]
        probo_position.extend([0,0,0])
        rotatedPen = transfIntrinsric(xyzRpyToMatrix(probo_position, False, True), [0, 0, 0, 90, -45, 0])
        self.rtde_c.moveJ_IK(tmToXyzRA(moveAbove(rotatedPen, 0.1)), self.default_rot_speed, self.default_rot_acc)

        self.rtde_c.moveL(tmToXyzRA(moveAbove(rotatedPen, pen_head_length - 0.003)),self.default_speed, self.default_acc)
        self.forceMoveZDown(2, 10, 2, 0)
        # hole insert -15.5mm
        # self.rtde_c.moveJ_IK(tmToXyzRA(moveAbove(rotatedPen, 0.2)), self.default_speed, self.default_acc)
        self.rtde_c.moveL(tmToXyzRA(moveAbove(rotatedPen, pen_head_length - 0.003 + 0.2)),self.default_speed, self.default_acc)
        self.move_to_rotatePen_mid_point()
        self.move_to_reset_JPS()
        # self.move_to_reset_JPS()
        #self.forceMoveZDown(3,3,2)
        # self.Gripper.SetPosition_0_1000(self.G_open)
        # time.sleep(self.G_wait_time)
        return

    def move_to_contact_and_retract_xy(self, prepare_contact_local, speed_local, gripper_rotate_degree, up_hight, retreat_hight):
        # speed_local: 3d list, like [0, 0 ,1]
        contact_acc = 0.02
        contact_poses = taskPoses(self.board_base_XyzRpy, [prepare_contact_local])
        contact_tcp   = taskPoseListToTcpList(contact_poses)

        contact_rotate = transfIntrinsric(contact_tcp[0], [0,0,0,0,0,gripper_rotate_degree])

        # move to x axis
        self.rtde_c.moveL(tmToXyzRA(moveAbove(contact_rotate, up_hight)), self.default_speed, self.default_acc)
        self.rtde_c.moveL(tmToXyzRA(contact_rotate), self.default_speed, self.default_acc)

        speed_task = rotat_around_Z_extrinsic(speed_local, self.board_base_XyzRpy[5])#.extend([0,0,0])
        b = speed_task[0]
        b.extend([0.0,0.0,0.0])
        #contact_direction = self.rtde_r.getTargetTCPSpeed()

       
        self.rtde_c.moveUntilContact(b, b, contact_acc)
        contact_pose = self.rtde_r.getActualTCPPose()

        self.rtde_c.moveL(tmToXyzRA(contact_rotate), self.default_speed, self.default_acc)
        self.rtde_c.moveL(tmToXyzRA(moveAbove(contact_rotate, retreat_hight)), self.default_speed, self.default_acc)


        return [contact_pose[0], contact_pose[1]]


    def task_contact_measure(self):

        self.rtde_c.setTcp(self.flange_to_tcp['measure'])
        self.Gripper.SetPosition_0_1000(self.G_close)
        # time.sleep(self.G_wait_time)
        contact_x_point   = self.move_to_contact_and_retract_xy(contact_x, [0, 1, 0], 90, self.default_hight, 0)
        contact_x_2_point = self.move_to_contact_and_retract_xy(contact_x_2, [0, 1, 0], 90, 0, self.default_hight)
        contact_y_point   = self.move_to_contact_and_retract_xy(contact_y, [1, 0, 0], 180, self.default_hight, self.default_hight)

        theta = computeVectorAngleDegree(contact_x_point, contact_x_2_point)

        intersect_point = intersection_two_lines_2D(contact_x_point, theta, 
                                                    contact_y_point, theta + math.radians(90))


        intersect_point.extend([self.board_base_XyzRpy[2],0,0,theta])
        new_blue_point = transfIntrinsric(xyzRpyToMatrix(intersect_point, False, True), [0.12,0.03025,0,0,0,0])                             
        return tmToXyzRA(new_blue_point)


    def task_open_door(self):

        self.rtde_c.setTcp(self.flange_to_tcp['open_door'])
        circel_path = rotate_line_path(gateRotateAxis, gateRotateAxisPoint, gateHandle[0:3], 0, math.radians(110), math.radians(0.1))
        circel_task_pose = taskPoses(self.board_base_XyzRpy, circel_path)
        circel_tcp_pose = taskPoseListToTcpList(circel_task_pose)
        path = tmListToTRA(circel_tcp_pose)

        self.rtde_c.moveL(tmToXyzRA(moveAbove(circel_tcp_pose[0],self.default_hight)), self.default_speed, self.default_acc)
        self.Gripper.SetPosition_0_1000(self.G_open)
        time.sleep(0.2)

      
        self.rtde_c.moveL(path[0], self.default_speed, self.default_acc)

        close_gate = self.G_close
        self.Gripper.SetPosition_0_1000(close_gate)
        time.sleep(self.G_wait_time)

        # for target in path:
        #     self.rtde_c.moveL(path)

        # path = addSpeedAccBlendToPath(tmListToTRA(circel_tcp_pose), self.default_speed, self.default_acc, 0.02)
        # self.rtde_c.moveL(path)
        # print("im done")

        #Move to initial joint position with a regular moveJ
        
                # Spiral
    
        velocity = 0.5
        acceleration = 0.5
        dt = 1.0/500  # 2ms
        lookahead_time = 0.1
        gain = 300
        # Execute 500Hz control loop for 2 seconds, each cycle is 2ms
        for target in path:
            t_start = self.rtde_c.initPeriod()
            # First move the robot down for 2 seconds, then up for 2 seconds
            #self.rtde_c.forceMode(task_frame, selection_vector, wrench_down, force_type, limits)
            self.rtde_c.servoL(target, velocity, acceleration, dt, lookahead_time, gain)
            self.rtde_c.waitPeriod(t_start)
        self.rtde_c.servoStop()

        self.Gripper.SetPosition_0_1000(self.G_open)
        time.sleep(0.2)

        moveForward = transfIntrinsric(circel_tcp_pose[-1], [0, -0.05, 0, 0, 0, 0])
        self.rtde_c.moveL(tmToXyzRA(moveForward), self.default_speed, self.default_acc)
        self.rtde_c.moveL(tmToXyzRA(moveAbove(moveForward, 0.05)), self.default_speed, self.default_acc)
        return
    
    def task_slider_move_back_force_x(self, flange_to_tcp_up,flange_to_tcp_down, slider_init_tcp,target_dis_to_up, back_force_dis, speed, acc):
        slider_target_back = transfIntrinsric(slider_init_tcp, [target_dis_to_up+back_force_dis/2, 0, 0, 0, 0, 0])
        slider_target_force = transfIntrinsric(slider_init_tcp, [target_dis_to_up-back_force_dis/2, 0, 0, 0, 0, 0])
        self.rtde_c.setTcp(flange_to_tcp_down)
        self.rtde_c.moveL(tmToXyzRA(slider_target_force),speed, acc)
        for i in range(0, 2):
            self.rtde_c.setTcp(flange_to_tcp_up)
            self.rtde_c.moveL(tmToXyzRA(slider_target_back),speed, acc)
            self.rtde_c.setTcp(flange_to_tcp_down)
            self.rtde_c.moveL(tmToXyzRA(slider_target_force),speed, acc)

    def task_move_slider(self):

        # set two different tcps for each finger
        flange_to_tcp_up = [-0.007 + slider_width/2, -0.0327, 0.1576, 0, 0 ,0]
        flange_to_tcp_down = [0.007 - slider_width/2, -0.0327, 0.1576, 0, 0, 0]
        self.rtde_c.setTcp(flange_to_tcp_up)
        girpper_close_slider = 670

        # compute target tcp poses
        slider_task_Poses = taskPoses(self.board_base_XyzRpy,[slider_white_init, slider_white_mid, slider_white_end])
        tcp_poses_rotated = transfIntrinsicList(taskPoseListToTcpList(slider_task_Poses), [0,0,0,0,0,90])
        slider_init_tcp = tcp_poses_rotated[0]
        slider_mid_tcp = tcp_poses_rotated[1]
        # init_away = transfIntrinsric(slider_init_tcp, [-0.002 ,0, 0, 0,0,0])
        init_away = transfIntrinsric(slider_init_tcp, [0 ,0, 0, 0,0,0])


        
        # move to start point
        self.rtde_c.moveL(tmToXyzRA(moveAbove(init_away, self.default_hight)),self.default_speed, self.default_acc)
        self.Gripper.SetPosition_0_1000(girpper_close_slider)
        time.sleep(self.G_wait_time)
        self.rtde_c.moveL(tmToXyzRA(init_away), self.default_speed, self.default_acc)
        # self.Gripper.SetPosition_0_1000(670)
        # time.sleep(self.G_wait_time)

        # move slider speed
        move_slider_speed = 0.005
        move_slider_acc = 0.2
        # move to mid 
        self.rtde_c.moveL(tmToXyzRA(slider_mid_tcp), move_slider_speed, move_slider_acc)
        time.sleep(1)
        self.task_slider_move_back_force_x(flange_to_tcp_up,flange_to_tcp_down, slider_mid_tcp,0.0, 0.006, move_slider_speed, move_slider_acc)
        time.sleep(0.2)

        # move back to initial point
        self.rtde_c.setTcp(flange_to_tcp_down)
        self.rtde_c.moveL(tmToXyzRA(slider_init_tcp),self.default_speed, self.default_acc)
        
        # move to camera point
        self.rtde_c.setTcp(flange_to_tcp_up)
        #prepare_camera = moveAbove(slider_init_tcp, 0.045)
        prepare_camera = moveAbove(slider_init_tcp, 0.025)
        self.rtde_c.moveL(tmToXyzRA(prepare_camera),self.default_speed, self.default_acc)  

        camera_relative = transfIntrinsric(prepare_camera, [0.05, -0.085, 0, 0, 0, 0])
        self.rtde_c.moveL(tmToXyzRA(camera_relative),self.default_speed, self.default_acc) 
        # request
        time.sleep(1)
        rospy.wait_for_service("/yolov5_ros/bboxlocalization")

        detect_triangles = rospy.ServiceProxy("/yolov5_ros/bboxlocalization", DetectTriangles)
        resp1 = detect_triangles(True)
        slider_length = slider_white_end[0] - slider_white_init[0]
        ratio = 1
        print("ratio,  flage")
        print(resp1.results.flag)
        print(resp1.results.ratio)
        print(slider_length)

        self.rtde_c.moveL(tmToXyzRA(prepare_camera),self.default_speed, self.default_acc)

        self.rtde_c.moveL(tmToXyzRA(slider_init_tcp),self.default_speed, self.default_acc)

        # move to the end slowly
        if resp1.results.flag is 2:
            print('not detected')
            self.rtde_c.setTcp(flange_to_tcp_up)
            self.rtde_c.moveL(tmToXyzRA(tcp_poses_rotated[2]),0.001, move_slider_acc)
            self.rtde_c.moveL(tmToXyzRA(moveAbove(tcp_poses_rotated[2], self.default_hight)),self.default_speed, self.default_acc)

        else:
            print("detected")
            ratio = 1-resp1.results.ratio
            if ratio > 1 or ratio < 0:
                ratio = 0
            x = ratio * slider_length
            # move to recognised 
            self.rtde_c.moveL(tmToXyzRA(transfIntrinsric( slider_init_tcp, [x, 0, 0, 0, 0, 0])),move_slider_speed, move_slider_acc)
            time.sleep(1)
            self.task_slider_move_back_force_x(flange_to_tcp_up,flange_to_tcp_down, slider_init_tcp,x, 0.006, move_slider_speed, move_slider_acc)
            self.rtde_c.setTcp(flange_to_tcp_up)
            self.rtde_c.moveL(tmToXyzRA(moveAbove(tcp_poses_rotated[2], self.default_hight)),self.default_speed, self.default_acc)

        # ratio = 0.3
        # x = ratio * slider_length
        # # move to recognised 
        # self.rtde_c.moveL(tmToXyzRA(transfIntrinsric( slider_init_tcp, [x, 0, 0, 0, 0, 0])),move_slider_speed, move_slider_acc)
        # time.sleep(1)
        # self.task_slider_move_back_force_x(flange_to_tcp_up,flange_to_tcp_down, slider_init_tcp,x, 0.006, move_slider_speed, move_slider_acc)
        # self.rtde_c.setTcp(flange_to_tcp_up)
        # self.rtde_c.moveL(tmToXyzRA(moveAbove(tcp_poses_rotated[2], self.default_hight)),move_slider_speed, move_slider_acc)
        # move up
        # move to camera poin
        # self.rtde_c.setTcp(flange_to_tcp_up)
        # prepare_camera = moveAbove(slider_init_tcp, self.default_hight)
        # self.rtde_c.moveL(tmToXyzRA(prepare_camera),self.default_speed, self.default_acc)  
  


        # if 0
        # move around
        # if not
        # move to target and move around

        
        return
    

    def task_rotate_rope(self):

        self.rtde_c.setTcp(self.flange_to_tcp['pick_pen'])
        rad_target = math.radians(10)
        strat_target = math.radians(70)
        relativePoses2 = [[-0.0026,  0.0613,  0.0404, 0, 0, rad_target],
                         [-0.0026,  0.0613,  0.0034, 0, 0, rad_target],
                         [ 0.0204,  0.0678,  0.0034, 0, 0, rad_target],
                         [ 0.0204,  0.0678,  0.0304, 0, 0, rad_target],
                         [ 0.0514, -0.0112,  0.0244, 0, 0,   0],
                         [ 0.0704, -0.0172, -0.0176, 0, 0,   0],
                         [ 0.0704,  0.0108, -0.0496, 0, 0,   0],
                         [ 0.0644,  0.1728, -0.0176, 0, 0,   0],
                         [ 0.0575,  0.1878, -0.0076, 0, 0,   0],
                         [ 0.0575,  0.1878,  0.0104, 0, 0,   0],
                         [ 0.0404,  0.1868,  0.0300, 0, 0,   0],
                         [ 0.0404,  0.1488,  0.0584, 0, 0,   0],
                         [ 0.0304, -0.0162,  0.0264, 0, 0,   0],
                         [ 0.0509, -0.0162, -0.0089, 0, 0,   0],
                         [ 0.0604,  0.0053, -0.0179, 0, 0,   0]]
        
        strategy2 = [[-0.0026,  0.0613,  0.0404, 0, 0, rad_target],
                    [-0.0026,  0.0613,  0.0034, 0, 0, rad_target],
                    [ 0.0204,  0.0678,  0.0034, 0, 0, rad_target],
                    [ 0.2500,  0.1000,  0.37, 0, 0, strat_target],
                    [ 0.3000,  0.1000,  0.20, 0, 0, strat_target]
                    ]
        relativePoses = []
        for lp in relativePoses2:
            temp = lp
            temp[2] = temp[2] + 0.007
            relativePoses.append(temp)

        strategy = []
        for lp in strategy2:
            temp = lp
            temp[2] = temp[2] + 0.007
            strategy.append(temp)
        
        # Smoothing cable

        basePoses = taskPoses(self.board_base_XyzRpy, strategy)
        tcpPoses = taskPoseListToTcpList(basePoses)

        self.Gripper.SetPosition_0_1000(900)
        time.sleep(self.G_wait_time)
        for i in range(0, 2):
            self.rtde_c.moveL(tmToXyzRA(tcpPoses[i]), self.default_speed, self.default_acc)

        self.Gripper.SetPosition_0_1000(620)
        time.sleep(self.G_wait_time) 
        for i in range(2, 5):
            self.rtde_c.moveL(tmToXyzRA(tcpPoses[i]), self.default_speed, self.default_acc)
        self.Gripper.SetPosition_0_1000(1000)
        time.sleep(self.G_wait_time)



        # Grab cable


        basePoses = taskPoses(self.board_base_XyzRpy, relativePoses)
        tcpPoses = taskPoseListToTcpList(basePoses)

        self.Gripper.SetPosition_0_1000(900)
        for i in range(0, 2):
            self.rtde_c.moveL(tmToXyzRA(tcpPoses[i]), self.default_speed, self.default_acc)

        self.Gripper.SetPosition_0_1000(610)
        time.sleep(self.G_wait_time) 

        for i in range(2, 5):
            self.rtde_c.moveL(tmToXyzRA(tcpPoses[i]), self.default_speed, self.default_acc)
        
        for i in range(0, 2):
            for j in range(5, 13):
                self.rtde_c.moveL(tmToXyzRA(tcpPoses[j]), self.default_speed, self.default_acc)
        
        for i in range(13, 15):
            self.rtde_c.moveL(tmToXyzRA(tcpPoses[i]), self.default_speed, self.default_acc)


        self.Gripper.SetPosition_0_1000(900)
        time.sleep(self.G_wait_time) 
        self.rtde_c.moveL(tmToXyzRA(moveAbove(tcpPoses[i], 0.1)), self.default_speed, self.default_acc)
        return

    def tranfer_rp(self):

        redPen = [0, 0.055,0.03, 0, 0, 0]
        blackPen = [0,0,0,0,0,0]
        battery_1 = [0.164, 0.060, -0.03, 0, 0, 0]
        battery_center = [0, 0, 0, 0,0 ,0]
        box_good = [0.08, 0.12, -0.025, 0, 0, 0]
        box_bad = [0.08, -0.12, -0.025, 0, 0 ,0] 
        rp = [redPen, blackPen, battery_1, battery_center, box_good, box_bad]
    
        return rp


    def moveL_tm_defult(self, tm):
        self.rtde_c.moveL(tmToXyzRA(tm), self.default_speed, self.default_hight)
    
    def moveL_above_tm_defult(self, tm, hight):
        self.moveL_tm_defult(moveAbove(tm, hight))

    def close_gripper(self):
        self.Gripper.SetPosition_0_1000(self.G_close)
        time.sleep(self.G_wait_time)
        return
    
    def open_gripper(self):
        self.Gripper.SetPosition_0_1000(self.G_open)
        time.sleep(self.G_wait_time)
        return
    
    ## tranfer-bility demo
    def task_pick_and_place_battery(self, battery_num, battery_width):
        self.rtde_c.setTcp(self.flange_to_tcp['pick_pen'])
        

        rotate_tcp = [0, 0, 0, -90, 180, 0]
        retack_tcp = [0, 0, self.default_hight, 0, 0, 0]
    
        batter_n_rp = self.tranfer_rp()[2]
        batter_n_rp[1] = batter_n_rp[1] - battery_num * battery_width

        batter_place_rp = self.tranfer_rp()[3]
        battery_pick_place_tool = taskPoseListToTcpList(taskPoses(self.board_base_XyzRpy,[batter_n_rp, batter_place_rp]))
        battery_pick_place_tool_rotate = transfIntrinsicList(battery_pick_place_tool, rotate_tcp)

        pick_tcp = battery_pick_place_tool_rotate[0]
        place_tcp = battery_pick_place_tool_rotate[1]
        self.open_gripper()

        self.moveL_above_tm_defult(pick_tcp, self.default_hight)
        self.moveL_tm_defult(pick_tcp)
        self.close_gripper()
        self.moveL_above_tm_defult(pick_tcp, self.default_hight)

        self.moveL_above_tm_defult(place_tcp, self.default_hight)
        self.moveL_tm_defult(place_tcp)
        self.open_gripper()
        self.moveL_above_tm_defult(place_tcp, self.default_hight)

        return
    
    def place_battery_box(self, good: bool):
        self.rtde_c.setTcp(self.flange_to_tcp['pick_pen'])


        rotate_tcp = [0, 0, 0, -90, 180, 0]
        retack_tcp = [0, 0, self.default_hight, 0, 0, 0]

        batter_box_rp = []
        if good:
            batter_box_rp = self.tranfer_rp()[4]
        else:
            batter_box_rp = self.tranfer_rp()[5]

        
        batter_pick_rp = self.tranfer_rp()[3]
        battery_pick_place_tool = taskPoseListToTcpList(taskPoses(self.board_base_XyzRpy,[batter_pick_rp, batter_box_rp]))
        battery_pick_place_tool_rotate = transfIntrinsicList(battery_pick_place_tool, rotate_tcp)

        pick_tcp = battery_pick_place_tool_rotate[0]
        place_tcp = battery_pick_place_tool_rotate[1]


        self.open_gripper()
        self.moveL_above_tm_defult(pick_tcp, self.default_hight+0.05)
        self.moveL_tm_defult(pick_tcp)
        self.close_gripper()
        self.moveL_above_tm_defult(pick_tcp, self.default_hight+0.05)

        self.moveL_above_tm_defult(place_tcp, self.default_hight+0.05)
        self.open_gripper()


    def probe_battery(self):
        self.rtde_c.setTcp(self.flange_to_tcp['pick_pen'])


        rotate_tcp = [0, 0, 0, -90, 180, 0]
        retack_tcp = [0, 0, -0.05, 0, 0, 0]

        batter_rp = self.tranfer_rp()[3]
        pen_rp = self.tranfer_rp()[0]
        
        battery_pick_place_tool = taskPoseListToTcpList(taskPoses(self.board_base_XyzRpy, [pen_rp, batter_rp]))
        battery_pick_place_tool_rotate = transfIntrinsicList(battery_pick_place_tool, rotate_tcp)
        battery_pick_place_tool_rotate_retract = transfIntrinsicList(battery_pick_place_tool_rotate, retack_tcp)


        self.open_gripper()

        # pick pen
        # approch
        self.moveL_above_tm_defult(battery_pick_place_tool_rotate_retract[0], self.default_hight)
        # above
        self.moveL_above_tm_defult(battery_pick_place_tool_rotate[0], self.default_hight)
        # down
        self.moveL_above_tm_defult(battery_pick_place_tool_rotate[0], 0.02)
        # Close Pen A little bit 
        self.Gripper.SetPosition_0_1000(680)
        time.sleep(0.2)

        self.forceMoveZDown(5, 10, 3)
        self.Gripper.SetPosition_0_1000(620)
        time.sleep(0.2)
        # up
        self.moveL_above_tm_defult(battery_pick_place_tool_rotate[0], self.default_hight)

        # probo above
        self.moveL_above_tm_defult(battery_pick_place_tool_rotate[1], self.default_hight+0.01)

        # probo down
        self.forceMoveZDown(5, 6, 3)
        
        # probo above
        self.moveL_above_tm_defult(battery_pick_place_tool_rotate[1], self.default_hight)

        # pen above
        self.moveL_above_tm_defult(battery_pick_place_tool_rotate[0], self.default_hight)
        # pen down
        self.moveL_above_tm_defult(battery_pick_place_tool_rotate[0], 0.02)
        self.forceMoveZDown(5, 15, 5 , 1)
        self.open_gripper()
        
        # retract
        self.moveL_above_tm_defult(battery_pick_place_tool_rotate_retract[0], self.default_hight)

        return 
    
    def task_move_battery_resetJPS(self):
        self.rtde_c.moveJ(degreeListToRad([5.46, -140.47, -56, -163, 50, 0]),self.default_rot_speed, self.default_rot_acc)
        return


    def run(self, board_base_TRPY):
        self.board_base_XyzRpy = board_base_TRPY
        self.rtde_c.forceModeStop()
        self.task_move_battery_resetJPS()
        battery_width = 0.03
        self.task_pick_and_place_battery(0, battery_width)
        self.probe_battery()
        self.place_battery_box(True)
        self.task_pick_and_place_battery(1, battery_width)
        self.probe_battery()
        self.place_battery_box(False)
        self.task_pick_and_place_battery(2, battery_width)
        self.probe_battery()
        self.place_battery_box(True)
        self.task_pick_and_place_battery(3, battery_width)
        self.probe_battery()
        self.place_battery_box(True)
        self.task_pick_and_place_battery(4, battery_width)
        self.probe_battery()
        self.place_battery_box(False)
        self.task_move_battery_resetJPS()


        # self.board_base_XyzRpy = self.task_contact_measure()
        # print("Measure Result; ")
        # print(self.board_base_XyzRpy)
        # self.task_poses = taskPoses(self.board_base_XyzRpy, self.task_relative_poses)
        # self.task_tcp_poses = taskPoseListToTcpList(self.task_poses)
        # # #print(self.task_relative_poses)
        # self.task_press_blue_button()
        # self.task_move_slider()
        # self.task_pick_black_insert_red_hole()
        # self.task_open_door()
        # self.task_pull_pen()
        # self.task_prepare_probe()
        # self.task_insert_pen_back()
        # self.task_rotate_rope()
        # self.task_press_red_button()
        # self.move_to_photo_pose(photo_flange, False)
       
        # self.task_pick_black_insert_red_hole()
        
        # self.task_pull_pen()
        
        # self.task_press_red_button()
       
            # Task Insert Pen

            # Task Open Door

            # Task Probe Hole

            # Task Rotate Line

            # Task Push Blue Button

            # Test movement is right

            # Test force mode is controllable
        self.rtde_c.stopScript()

        return


if __name__ == "__main__":

    rospy.init_node('robot_planning', anonymous=True)

    #rospy.Subscriber('/BoardLoc', Pose2D, forceMoveZDown)
    # Create a gripper object
  
    rate = rospy.Rate(5)


    photo_flange = [0.25,-0.3,0.5,2.221,2.221,0 ]

    robothon_mission = mission()
    # robothon_mission.move_to_photo_pose(photo_flange)
    robothon_mission.initialize_gripper(60, 90)
    arrivePhotoPose = True
    

    while not rospy.is_shutdown():
        # Take picture
        if True:
        #if False:
            boardBaseXyzRpy = np.array([1e-3*358.3, 1e-3 * -489, 1e-3*35, 0, 0, math.radians(45-2)])
            print(boardBaseXyzRpy)
            robothon_mission.run(boardBaseXyzRpy)
            break
        else:
            rate.sleep()
 

    




