import roslib
import rospy
import rtde_control
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
from RS485Gripper.rs485gp import RS485BusGripper
from rtde_control import RTDEControlInterface as RTDEControl
import time
import math

if __name__ == "__main__":
    # Create a gripper object
    Gripper = RS485BusGripper(name='RS485Gripper',device='/dev/ttyUSB0', RSAddress=1,Baud = 115200)

    # Initialize Gripper
    Gripper.InitializeGripper()
    time.sleep(3)

        # Set 30% Grip force
    Gripper.SetForceLimitPercentage_20_100(20)
    # Set 50% Grip force
    Gripper.SetSpeedLimitPercentage_1_100(50)
    # Go to position 500, (0-1000)

    # 600 close
    # 680 for almoset grip pen
    Gripper.SetPosition_0_1000(800)