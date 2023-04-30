#! /usr/bin/env python3

# prerequisite: 
# pip install pyrealsense2

# launch roscore
# run this file


# import pyrealsense2 as rs
import numpy as np
import cv2
import math
import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 

# LowerBlue  = np.array([53,205,132])
# UpperBlue = np.array([153,255,232])
# LowerRed = np.array([125,115,108])
# UpperRed = np.array([180,215,208])
# linearCalibrationPrams = [0,400,0,-400]

# Realsense Topic
COLOR_FRAME_TOPIC = '/camera/color/image_raw'
DEPTH_ALIGNED_TOPIC = '/camera/aligned_depth_to_color/image_raw'
CAMERA_INFO_TOPIC = '/camera/aligned_depth_to_color/camera_info'

class Board_Loc:
    
    def __init__(self):
        self.pubLoc = rospy.Publisher('BoardLoc', Pose2D, queue_size=1)
        self.LowerBlue  = np.array([53,205,130])
        self.UpperBlue = np.array([153,255,230])
        self.LowerRed = np.array([125,115,108])
        self.UpperRed = np.array([180,215,208])
        self.bridge = CvBridge()
        self.flagImageSub = False

        # ly modified
        self.color_image = None
        self.realsense_sub = rospy.Subscriber(COLOR_FRAME_TOPIC, Image, self.image_receive, queue_size=1)
        
        self.pose = Pose2D()
        self.find()

    def image_receive(self, data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.color_image = color_image.copy()
            rospy.loginfo("Frame recived...")
            self.flagImageSub = True
        except CvBridgeError as e:
            print(e)   
    
    def find_center(self,contours):
        if len(contours) > 0:
            size_max_index = -1
            size_max = 0
            x_mid_final = 0
            y_mid_final = 0
            for i, c in enumerate(contours):
                area = cv2.contourArea(contours[i])
                rect = cv2.minAreaRect(c)
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                x_mid = (box[0][0] + box[2][0] + box[1][0] + box[3][0]) / 4
                y_mid = (box[0][1] + box[2][1] + box[1][1] + box[3][1]) / 4
                w = math.sqrt((box[0][0] - box[1][0]) ** 2 + (box[0][1] - box[1][1]) ** 2)
                h = math.sqrt((box[0][0] - box[3][0]) ** 2 + (box[0][1] - box[3][1]) ** 2) 
                sizeTemp = w*h
                if area > size_max:
                    size_max_index = i
                    found = True
                    x_mid_final = x_mid
                    y_mid_final = y_mid
                # if size[i] > size_max:
            return((float)(x_mid_final),(float)(y_mid_final))
        return(-1,-1)

    def solveWorldCartisianLoc(self,x_mid_final,y_mid_final):
        a = [0]*2
        b = [0]*2
        a[0]=float(self.linearCalibrationPrams[0])
        a[1]=float(self.linearCalibrationPrams[1])
        b[0]=float(self.linearCalibrationPrams[2])
        b[1]=float(self.linearCalibrationPrams[3])
        world_x = a[0] * y_mid_final + a[1]
        world_y = b[0] * x_mid_final + b[1]
        return(world_x,world_y)
        
    def stop(self):
        '''Method to disconnect the subscribers from kinect2_bridge topics, to release
            some memory and avoid filling up the queue.'''
        self.realsense_sub.unregister()

    def find(self):

        # be replaced part
        # Configure depth and color streams
        # pipeline = rs.pipeline()
        # config = rs.config()
        # config.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)
        # # Start streaming
        # pipeline.start(config)
        # # Get the sensor once at the beginning. (Sensor index: 1)
        # sensor = pipeline.get_active_profile().get_device().query_sensors()[1]

        # # Set the exposure anytime during the operation
        # sensor.set_option(rs.option.exposure, 70.000)

        try:
            while True:
                # frames = pipeline.wait_for_frames()
                # color_frame = frames.get_color_frame()
                # color_image = np.asanyarray(color_frame.get_data())

                if (self.flagImageSub):
                    # Gaussian
                    color_image1 = cv2.GaussianBlur(self.color_image, (5, 5), 0)
                    # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                    # cv2.imshow('RealSense', color_image1)
                    hsv = cv2.cvtColor(color_image1, cv2.COLOR_BGR2HSV)


                    # Find Blue Button
                    mask = cv2.inRange(hsv, self.LowerBlue, self.UpperBlue)
                    # cv2.imshow('mask',mask)
                    mask = cv2.erode(mask, None, iterations=2)
                    # cv2.imshow('Blue Button',mask)
                    contours, hier = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    xBlue ,yBlue = self.find_center(contours)
                    # print('xB: ',xBlue)
                    # print('yB: ',yBlue)
                    # print(type(yBlue))
                    
                    h = 10
                    xBint = (int)(xBlue)
                    yBint = (int)(yBlue)
                    draw_0 = cv2.rectangle(color_image1, (xBint-h, yBint-h), (xBint+h, yBint+h), (200, 0, 0), 2)
                    # cv2.imshow('draw_0',draw_0)


                    # Find Red Button
                    crop_img_bw = cv2.dilate(mask, None, iterations=30)
                    crop_img = cv2.bitwise_and(hsv,hsv,mask = crop_img_bw)
                    # cv2.imshow('RedBarea',crop_img)
                    mask = cv2.inRange(crop_img, self.LowerRed, self.UpperRed)
                    # cv2.imshow('mask2',mask)
                    mask = cv2.erode(mask, None, iterations=2)
                    # cv2.imshow('Red Button',mask)
                    contours, hier = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                    xRed ,yRed = self.find_center(contours)
                    # print('xR: ',xRed)
                    # print('yR: ',yRed)
                    
                    xRint = (int)(xRed)
                    yRint = (int)(yRed)
                    draw_0 = cv2.rectangle(draw_0, (xRint-h, yRint-h), (xRint+h, yRint+h), (0, 0, 200), 2)
                    
                    
                    self.pose.x = xBlue
                    self.pose.y = yBlue
                    
                    
                    self.pose.x = -250.0/463.0 * yBlue + 279850.0/463.0-50-20
                    self.pose.y = -175.0/324.0 * xBlue + 4075.0/108.0
                    
                    #!!! Not Accurate
                    #!!! Need further verification
                    self.pose.theta = math.atan2(xBlue-xRed,yRed-yBlue)
                    #!!! Need further verification
                    self.pose.theta = -(self.pose.theta)*180.0/3.14159
                    
                    print(self.pose.x)
                    print(self.pose.y)
                    # print(-(self.pose.theta)*180.0/3.1416)
                    # print(self.pose.theta)
                    
                    xrol,ycol = hsv.shape[0:2]
                    img0 = np.zeros((xrol, ycol,3), dtype = np.uint8)
                    distance = ((xBlue-xRed)**2+(yBlue-yRed)**2)**0.5
                    M5_mask = cv2.circle(img0, ((int)(xBlue+(xRed-xBlue)*5.5), (int)(yBlue+(yRed-yBlue)*5.5)), (int)(3*distance), (255,255,255), -1)
                    # M5_crop_img = cv2.circle(img0, (100,100), 50, (255,255,255), -1)
                    M5_crop_img = cv2.bitwise_and(hsv,M5_mask)
                    # cv2.imshow('crop',M5_crop_img)
                    mask = cv2.inRange(M5_crop_img, self.LowerRed, self.UpperRed)
                    # cv2.imshow('redM5',mask)
                    
                    gray = cv2.cvtColor(color_image1, cv2.COLOR_BGR2GRAY)
                    edges = cv2.Canny(gray, 50, 150)
                    # cv2.imshow('edge',edges)
                    
                    lines = cv2.HoughLines(edges, 1, np.pi/180, 90)
                    thetaList = []
                    if lines is not None:
                        for line in lines:
                            rho = line[0][0]
                            theta = line[0][1]
                            thetaDeg = -theta*180.0/3.14159
                            # print(thetaDeg)
                            if ((thetaDeg-self.pose.theta)**2 < 10):
                                thetaList.append(thetaDeg)
                            elif ((thetaDeg-90-self.pose.theta)**2 < 10):
                                thetaList.append(thetaDeg-90)
                            elif (thetaDeg+90-self.pose.theta)**2 < 10:
                                thetaList.append(thetaDeg+90)
                            elif (thetaDeg-180-self.pose.theta)**2 < 10:
                                thetaList.append(thetaDeg-180)
                            elif (thetaDeg+180-self.pose.theta)**2 < 10:
                                thetaList.append(thetaDeg+180)
                            elif (thetaDeg-270-self.pose.theta)**2 < 10:
                                thetaList.append(thetaDeg-270)
                            elif (thetaDeg+270-self.pose.theta)**2 < 10:
                                thetaList.append(thetaDeg+270)
                            elif (thetaDeg-360-self.pose.theta)**2 < 10:
                                thetaList.append(thetaDeg-360)
                            elif (thetaDeg+360-self.pose.theta)**2 < 10:
                                thetaList.append(thetaDeg+360)
                            elif (thetaDeg-450-self.pose.theta)**2 < 10:
                                thetaList.append(thetaDeg-450)
                            elif (thetaDeg+450-self.pose.theta)**2 < 10:
                                thetaList.append(thetaDeg+450)

                            a = np.cos(theta)
                            b = np.sin(theta)
                            x0 = a*rho
                            y0 = b*rho
                            x1 = int(x0 + 1000*(-b))
                            y1 = int(y0 + 1000*(a))
                            x2 = int(x0 - 1000*(-b))
                            y2 = int(y0 - 1000*(a))
                            cv2.line(draw_0, (x1, y1), (x2, y2), (0, 0, 255), 1)
                        print('!-----!')
                        print(self.pose.theta)
                        # print('-----')
                        # print(thetaList)
                        print('-----')
                        if len(thetaList) is not 0:
                            thetaMean = sum(thetaList)/(float)(len(thetaList))
                            print(thetaMean)
                        else:
                            thetaMean = self.pose.theta
                    else:
                        thetaMean = self.pose.theta
                    # cv2.imshow('hough',gray)
                    
                    x, y = draw_0.shape[0:2]
                    draw_01 = cv2.resize(draw_0, (int(y / 2), int(x / 2)))
                    cv2.imshow('Result',draw_01)
                    # M5_image_rgb = cv2.bitwise_and(self.color_image,self.color_image,mask = mask)
                    
                    self.pose.theta = thetaMean
                    
                    self.pubLoc.publish(self.pose)
                    
                    
                    
                    key = cv2.waitKey(200)
                    # break
                    # Press esc or 'q' to close the image window
                    if key & 0xFF == ord('q') or key == 27:
                        cv2.destroyAllWindows()
                        self.stop()
                        exit(0)
                        break
                else:
                    rospy.loginfo("Waiting frame ...")
                    color_image = rospy.wait_for_message(COLOR_FRAME_TOPIC, Image, timeout=None)
                    colorFrame = self.bridge.imgmsg_to_cv2(color_image, desired_encoding="bgr8")
                    self.color_image = colorFrame.copy()
                    rospy.loginfo("Frame recived...")
        finally:
            # Stop streaming
            self.stop()

        
if __name__ == '__main__':
    try:
        rospy.init_node('BoardLoc', anonymous=False)
        rospy.loginfo("Init BoardLoc main")   
        Board_Loc()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("End BoardLoc main")
