1、运行主节点
roscore

2、在 /home/leech/Documents/metabot/robothon2023_ws/src/robothon2023_vision/src 下打开终端，创建服务的server
cd /home/leech/Documents/metabot/robothon2023_ws/src/robothon2023_vision/src
python BoardLocalization.py
or
roslaunch robothon2023_vision board_location_service.launch

3、命令行运行，创建服务请求
rosservice call /robothon2023/board_localization True

rosservice call /yolov5_ros/bboxlocalization True

4、查看图像话题
rosrun image_view image_view image:=/【topic名】

fix: 
1、tf变换要写在哪里
2、相机像素和实际距离的比例，分辨率
3、不畸变的topic
4、还没将需要的信息发布出来
5、整个任务需要识别出什么东西，红蓝按钮，滑动变阻器，屏幕中的小三角形，铁门的门把、两个绕线的卡槽
6、是要写成service的形式，还是topic的形式 —— service
7、现在识别率不算高
8、【找到两个距离相对较远的特征（x1,y1）(x2,y2)然后算斜率，rotation】——初步可以定为screen pos的长方形顶点还有按钮的中心,其实我直接把board的箱线的斜率算出来，然后跟screen和button的连线斜率比较，他们应该是平行的
9、【可以用这个做一做】在寻找红蓝按钮的函数getRedBlueButtonsNewVersion中加入颜色的判断作为联合检测，把kmeans改成是（HSV或者LAB）三维的颜色聚类

# 需要手动调参的部分
找红蓝按钮时，按钮的的半径（5~13 pixel）
通过面积排查极大值，极小值，这个调整空间不大，如果后面是在固定高度，就可以限制的更死一点



# Board Localization Repository
<p align="center">
  <img width="600" src="https://github.com/JRL-CARI-CNR-UNIBS/robothon2022_report/blob/master/images/Vision_System.png">
</p>
This package is for real time robothon-board localization: the vision system is used to identify the position of the board relative to the robot base. Specifically, an rgb frame is acquired as the first task, then features such as the center of the red button, key lock, and screen are identified. 

The features are recognized by applying border detection, color clustering, canny detection, Hough transform and custom designed vision algorithms. Once the features position is detected in the image frame, we move on to the camera reference system and finally that of the robot base, thanks to the instrinsic and extrinsic parameters estimated in the offline part. 

## Requirements
- **realsense-ros**: you can find the necessary package [here](https://github.com/IntelRealSense/realsense-ros)
