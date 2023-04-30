## service name: "/yolov5_ros/bboxlocalization"
## this service will call topic "/bboxtri/detections" once
## output topic: "/bboxtri/detections"
## message of topic "/bboxtri/detections": bboxtri.msg, shown as below:
```bash
flag: 1 / -1 / 2 / 0
ratio: [0,1]
state: just ignore
```
## flag represents go up/left or down/right (as shown in Fig), 
```bash
flag==-1 > redtri go up/left
flag==1 > redtri go down/right
flag==2 > still not detect the blue_tri
flag==0 > blue_tri is just matched to white_tri (very ideal)
```
```bash
screendistance = abs(X_max_of_screen - X_min_of_screen)
ratio = (X_center_of_blue_tri - X_min_of_screen) / (screendistance)
```
## Asumptionthis Algorithm suppose the length of taskboard is parell to X-axis

## how to run, follow as belows:
## run: 
```bash
cd ~/Desktop/robothon/Robothon_REAL/ws_vision/src/robothon2023_vision/src
```

## run: 
```bash
python3 bbox_subscriber.py
```

## suppose: all workspaces which contain the used packages are export into ~/.bashrc, otherwise you should go into the corresponding workspace then "source devel/setup.bash"

## run: 
```bash
roslaunch realsense2_camera rs_camera.launch
```
## run: 
```bash
roslaunch yolov5_ros yolov5.launch 
```
# this order will present the iamge detected by yolov5
## run: 
```bash
rosservice call /yolov5_ros/bboxlocalization True 
```
# this service will call topic "/bboxtri/detections" once, message type: from detection_msgs.msg import bboxtri

## run: 
```bashrostopic echo /bboxtri/detections  
```
# you can run this to supervise the publish info

## all possibilities of response:
    ## success: True / False
    ## message: "Successfully detect bbox" / "fail to detect bbox"

## instruction from service("/yolov5_ros/bboxlocalization")
    ## if, success: False
    ## so, the information from topic "/bboxtri/detections" do not use!!!!!!

    ## if, success: True
    ## so, the information from topic "/bboxtri/detections" can be use!!!!!!
    
## save the history of the blue_tri, white_tri, screen_tri
## detect 
