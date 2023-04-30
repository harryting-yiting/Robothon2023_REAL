#!/usr/bin/env python
#!coding=utf-8
 
import rospy
import numpy as np
from sensor_msgs.msg import Image
from detection_msgs.msg import BoundingBox, BoundingBoxes, bboxtri
from detection_msgs.srv import DetectTriangles, DetectTrianglesResponse
import math
from std_srvs.srv import SetBool,SetBoolResponse

GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
BOLD = '\033[1m'
END = '\033[0m'


SERVICE_CALLBACK = GREEN + "Service call {} received" + END

SERVICE_NAME = "/yolov5_ros/bboxlocalization"

SUCCESSFUL = "Successfully detect bbox"
FAIL = "fail to detect bbox"

class BboxLocalization:
    def __init__(self):
        self.xdistance_new = 0.0
        self.ydistance_new = 0.0
        self.screendistance_new = 0.0
        self.ratio_bluewhite2screen = 0.0
        self.xminscreen = 0
        self.xmaxscreen = 0
        self.xminwhite_tri = 0
        self.xmaxwhite_tri = 0
        # flag about whether detect the blue_tri or not
        self.flag_blue_tri = False
        # self.flag_whlite_blue_overlap = 

        self.undetectedCount = 0

        self.bbox_of_class_perimage = []
        self.bbox_of_white_tri = []
        self.bbox_of_blue_tri = []
        self.bbox_of_screen = []

        self.yolobbox_sub = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.BboxCallback)
        self.srv_pub = rospy.Publisher('bboxtri/detections', bboxtri, queue_size=1)

        self.srv_pub_info = bboxtri()
        self.srv_pub_info.flag = 2  # represent go up/left or down/right (as shown in Fig), flag==-1 > redtri go up/left; flag==1 > redtri go down/right; flag==2 > still not detect the blue_tri; flag==0 > blue_tri is just matched to white_tri (very ideal)
        self.srv_pub_info.ratio = 2
        self.srv_pub_info.state = -1

    def ServiceCallback(self,request):
        rospy.loginfo(SERVICE_CALLBACK.format(SERVICE_NAME))
        # publish xdistance, slider need to go up or down
        #self.srv_pub.publish(self.srv_pub_info)

        rospy.loginfo(GREEN+"self.flag_blue_tri = "+END)
        print(self.flag_blue_tri)
        rospy.loginfo(GREEN+"self.srv_pub_info.flag = "+END)
        print(self.srv_pub_info.flag)
        rospy.loginfo(GREEN+"self.srv_pub_info.ratio = "+END)
        print(self.srv_pub_info.ratio)
        rospy.loginfo(GREEN+"self.srv_pub_info.state"+END)
        print(self.srv_pub_info.state)
        rospy.loginfo(GREEN+"PUBLISH FINISH!!"+END)
        # if self.undetectedCount > 50 or not(self.flag_blue_tri):

        if self.undetectedCount > 20:   # 50
            return DetectTrianglesResponse(False,self.srv_pub_info)
        else:
            return DetectTrianglesResponse(True, self.srv_pub_info)

    def BboxCallback(self,bboxdata):
        """
        Class: screen, red_tri, blue_tri, white_tri
        """
        print("-------------------------")
        print("number of bboxs per image =",len(bboxdata.bounding_boxes))
        print("-------------------------")
        self.flag_blue_tri = False
        self.undetectedCount = self.undetectedCount+1
        print("-------------------------")
        print("self.undetectedCount",self.undetectedCount)
        print("-------------------------")
        self.srv_pub_info.flag = 2
        if self.undetectedCount < 20:
            self.srv_pub_info.flag = 1

        if len(bboxdata.bounding_boxes) != 0:
            
            self.bbox_of_class_perimage = []
            self.bbox_of_white_tri = []
            self.bbox_of_blue_tri = []
            
            self.bbox_of_screen = []
            for bbox in bboxdata.bounding_boxes:
                
                self.bbox_of_class_perimage.append(bbox.Class)
                # print("-------------------------")
                # print("bbox_of_class_perimage=",self.bbox_of_class_perimage)
                # print("-------------------------")

                # correct image every class only have one [bbox.xmin,bbox.ymin,bbox.xmax,bbox.ymax]
                if bbox.Class == "white_tri":
                    self.bbox_of_white_tri.append([bbox.xmin,bbox.ymin,bbox.xmax,bbox.ymax])
                    self.xminwhite_tri = bbox.xmin
                    self.xmaxwhite_tri = bbox.xmax
                    # print("-------------------------")
                    # print("bbox_of_white_tri=",self.bbox_of_white_tri)
                    # print("-------------------------")
                if bbox.Class == "blue_tri":
                    self.flag_blue_tri = True
                    self.bbox_of_blue_tri.append([bbox.xmin,bbox.ymin,bbox.xmax,bbox.ymax])
                    # print("-------------------------")
                    # print("bbox_of_blue_tri=",self.bbox_of_blue_tri)
                    # print("-------------------------")
                    self.undetectedCount = 0
                if bbox.Class == "screen":
                    self.bbox_of_screen.append([bbox.xmin,bbox.ymin,bbox.xmax,bbox.ymax])
                    self.xminscreen = bbox.xmin
                    self.xmaxscreen = bbox.xmax
                if 'white_tri' in self.bbox_of_class_perimage and 'screen' in self.bbox_of_class_perimage:
                    rospy.loginfo(YELLOW+"detect the white_tri and screen"+END)
                    # keep the screendistance always be positive
                    screendistance = abs(self.bbox_of_screen[0][2]-self.bbox_of_screen[0][0])
                    self.screendistance_new = screendistance
                    # print("-------------------------")
                    # print("self.screendistance_new = ", self.screendistance_new)
                    # print("-------------------------")
                # if bbox_of_class_perimage.find('white_tri') and bbox_of_class_perimage.find('blue_tri'):
                if self.flag_blue_tri and ('white_tri' in self.bbox_of_class_perimage and 'blue_tri' in self.bbox_of_class_perimage):
                    rospy.loginfo(GREEN+"detect the white_tri and blue_tri"+END)

                    # debug about IndexError out of list
                    # xdistance1 = self.bbox_of_blue_tri[0][0]
                    # xdistance2 = self.bbox_of_blue_tri[0][2]
                    # xdistance3 = self.bbox_of_white_tri[0][0]
                    # xdistance4 = self.bbox_of_white_tri[0][2]

                    xdistance = 0.5*(self.bbox_of_blue_tri[0][0]+self.bbox_of_blue_tri[0][2])-0.5*(self.xminwhite_tri+self.xmaxwhite_tri)
                    ydistance = 0.5*(self.bbox_of_blue_tri[0][1]+self.bbox_of_blue_tri[0][3])-0.5*(self.bbox_of_white_tri[0][1]+self.bbox_of_white_tri[0][3])
                    self.xdistance_new = xdistance
                    self.ydistance_new = ydistance
                    if self.screendistance_new != 0.0:
                        # ratio_bluewhite2screen reprepsents , normal situation ratio belongs to [0,1]
                        # self.ratio_bluewhite2screen = self.xdistance_new / self.screendistance_new
                        self.ratio_bluewhite2screen = (0.5*(self.bbox_of_blue_tri[0][0]+self.bbox_of_blue_tri[0][2])-self.xminscreen) / self.screendistance_new
                        self.srv_pub_info.ratio = self.ratio_bluewhite2screen
                    # print("-------------------------")
                    # print("xdistance = ", xdistance)
                    # print("ydistance = ", ydistance)
                    # print("ratio_bluewhite2screen = ", self.ratio_bluewhite2screen)
                    # print("-------------------------")
                    # bbox_of_class_perimage = []
                    # bbox_of_white_tri = []
                    # bbox_of_blue_tri = []                    
                    if xdistance<0:
                        self.srv_pub_info.flag = -1
                    elif xdistance>0:
                        self.srv_pub_info.flag = 1
                    else:
                        self.srv_pub_info.flag = 0
                    # rospy.loginfo(GREEN+"self.srv_pub_info.flag = "+END)
                    # print(self.srv_pub_info.flag)
                    # print("-------------------------")

                elif 'blue_tri' in self.bbox_of_class_perimage and 'screen' in self.bbox_of_class_perimage:
                    rospy.loginfo(BOLD+"detect the screen and blue_tri"+END)
                    xdistance = 0.5*(self.bbox_of_blue_tri[0][0]+self.bbox_of_blue_tri[0][2]) - 0.5*(self.xminscreen+self.xmaxscreen)
                    self.xdistance_new = xdistance
                    if xdistance<0:
                        self.srv_pub_info.flag = -1
                    elif xdistance>0:
                        self.srv_pub_info.flag = 1
                    else:
                        self.srv_pub_info.flag = 0

                    
                if ('screen' in self.bbox_of_class_perimage) and (not('white_tri' in self.bbox_of_class_perimage)) and (not('blue_tri' in self.bbox_of_class_perimage)) and (not('red_tri' in self.bbox_of_class_perimage)):
                    if self.srv_pub_info.flag == -1 or self.srv_pub_info.flag == 1:
                        rospy.loginfo(GREEN+"TASK of bboxtri/detections FINISH!!"+END)
                        self.srv_pub_info.state = 1

                else:
                    continue
                    # consider if could not detect whrite_tri and blue_tri as the same time
        
            rospy.loginfo(GREEN+"self.flag_blue_tri = "+END)
            print(self.flag_blue_tri)
            rospy.loginfo(GREEN+"self.srv_pub_info.flag = "+END)
            print(self.srv_pub_info.flag)
            rospy.loginfo(GREEN+"self.srv_pub_info.ratio = "+END)
            print(self.srv_pub_info.ratio)
            rospy.loginfo(GREEN+"self.srv_pub_info.state"+END)
            print(self.srv_pub_info.state)
            # rospy.loginfo(GREEN+"PUBLISH FINISH!!"+END)
            print("-------------------------")

        else:
            rospy.loginfo(RED+"This Image does not detect any BBox"+END)
            self.srv_pub_info.state = -1

            rospy.loginfo(GREEN+"self.flag_blue_tri = "+END)
            print(self.flag_blue_tri)            
            rospy.loginfo(RED+"self.srv_pub_info.flag = "+END)
            print(self.srv_pub_info.flag)
            rospy.loginfo(RED+"self.srv_pub_info.ratio = "+END)
            print(self.srv_pub_info.ratio)
            rospy.loginfo(RED+"self.srv_pub_info.state"+END)
            print(self.srv_pub_info.state)
            # rospy.loginfo(GREEN+"PUBLISH FINISH!!"+END)
            print("-------------------------")
        """
        ## Solution One ----  Drop!!!!!!!!!!!!##
        # # such lonely append can bot ganunteed timestamp in each list is matched (X, drop!!!!!!)
        # # every bboxdata.bounding_boxes represent each image
        # # in every "for" iteration, bboxes represent the total bboxes in each image 
        # if len(bboxdata.bounding_boxes) != 0:
        #     for bboxes in bboxdata.bounding_boxes:
        #         if bboxes.Class == "screen":
        #             bbox_of_screen.append(bboxes[2:len(bboxes)])
        #         elif bboxes.Class == "white_tri":
        #             bbox_of_white_tri.append(bboxes[2:len(bboxes)])
        #         elif bboxes.Class == "red_tri":
        #             bbox_of_red_tri.append(bboxes[2:len(bboxes)])
        #         elif bboxes.Class == "blue_tri":
        #             bbox_of_blue_tri.append(bboxes[2:len(bboxes)])
        #         else:
        #             continue        
        """

 
if __name__ == '__main__':
    rospy.init_node('bbox_subsciber', anonymous=True)

    bboxlocalization = BboxLocalization()
    # in the class init has subscriber, out of the class has service
    rospy.Service(SERVICE_NAME,DetectTriangles,bboxlocalization.ServiceCallback)
    rospy.loginfo(GREEN+" /yolov5_ros/bboxlocalization Sever has activate "+END)
    rospy.spin()

# Readme
## run: cd ~/Desktop/robothon/Robothon_REAL/ws_vision/src/yolov5_ros/src
## run: python3 bbox_subscriber.py
## suppose: all workspaces which contain the follow packages are export into ~/.bashrc, otherwise you should go into the corresponding workspace then "source devel/setup.bash"
## run: roslaunch realsense2_camera rs_camera.launch
## run: roslaunch yolov5_ros yolov5.launch # this order will present the iamge detected by yolov5
## run: rosservice call /yolov5_ros/bboxlocalization True   # this service will call topic "/bboxtri/detections" once, message type: from detection_msgs.msg import bboxtri
## run: rostopic echo /bboxtri/detections   # you can run this to supervise the publish info

## all possibilities of response:
    ## success: True / False
    ## message: "Successfully detect bbox" / "fail to detect bbox"

## instruction from service("/yolov5_ros/bboxlocalization")
    ## if, success: False
    ## so, the information from topic "/bboxtri/detections" do not use!!!!!!

    ## if, success: True
    ## so, the information from topic "/bboxtri/detections" can be use!!!!!!
