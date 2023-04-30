#!/usr/bin/env python

# Prerequisties:
# sudo apt install python3-pip
# pip install pyserial
# sudo usermod -aG dialout ???(Username)
# #restart


from gripper import gripper_ctrl #Import Lib of gripper
import time

graspObject = gripper_ctrl(7,'/dev/ttyUSB0') 
# 7                 is the motor ID, no need to change
# '/dev/ttyUSB0'    is the system USB device name, which might change

graspObject.grasp()
time.sleep(1.0)
graspObject.release()
time.sleep(1.0)
graspObject.grasp()
time.sleep(1.0)
graspObject.release()
time.sleep(1.0)
graspObject.grasp()
time.sleep(1.0)
graspObject.release()
time.sleep(0.2)
graspObject.close()
